#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#if defined(__x86_64__) || defined(__i386__)
#include <x86intrin.h>
#endif

#define DEFAULT_DEVICE "0000:88:00.0"
#define DEFAULT_BAR 1
#define DEFAULT_MAP_SIZE (64ULL * 1024ULL * 1024ULL)
#define DEFAULT_SECONDS 1.0
#define DEFAULT_LATENCY_ITERS 100000
#define DEFAULT_BW_ITERS 0
#define MAX_PACKET_SIZES 64

static const size_t default_packet_sizes[] = {
    4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536
};

typedef struct {
    const char *path;
    const char *device;
    int bar;
    bool wc;
    bool read_tests;
    bool write_tests;
    bool verify;
    bool csv;
    bool unsafe_writes;
    off_t offset;
    size_t map_size;
    double seconds;
    uint64_t latency_iters;
    uint64_t bw_iters;
    size_t packet_sizes[MAX_PACKET_SIZES];
    size_t packet_count;
} options_t;

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "PCIe BAR sysfs mmap latency/bandwidth benchmark.\n"
        "Default target: /sys/bus/pci/devices/0000:88:00.0/resource1\n"
        "\n"
        "Options:\n"
        "  -p, --path PATH          BAR resource path (overrides --device/--bar/--wc)\n"
        "  -d, --device BDF         PCI BDF, default %s\n"
        "  -b, --bar N              BAR resource number, default %d\n"
        "      --wc                 Use resourceN_wc instead of resourceN\n"
        "  -o, --offset BYTES       Offset inside BAR mapping, default 0\n"
        "  -m, --map-size BYTES     Bytes to map/test, default min(file size, 64MiB)\n"
        "  -s, --sizes LIST         Comma-separated packet sizes, default 4..65536\n"
        "  -t, --seconds SEC        Minimum seconds per bandwidth test, default %.1f\n"
        "  -n, --latency-iters N    Iterations per latency test, default %u\n"
        "      --bw-iters N         Fixed iterations per bandwidth test (0 = time based)\n"
        "      --read-only          Only run read tests\n"
        "      --write-only         Only run write tests\n"
        "      --unsafe-writes      Enable write tests without the interactive safety gate\n"
        "      --verify            Read back after writes and report compare errors\n"
        "      --csv               CSV output\n"
        "  -h, --help              Show this help\n"
        "\n"
        "WARNING: BAR writes can change device state. Use --unsafe-writes only on a\n"
        "scratch/test BAR region where arbitrary write patterns are safe.\n",
        prog, DEFAULT_DEVICE, DEFAULT_BAR, DEFAULT_SECONDS, DEFAULT_LATENCY_ITERS);
}

static uint64_t nsec_now(void) {
    struct timespec ts;
#ifdef CLOCK_MONOTONIC_RAW
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
    clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void mmio_fence(void) {
#if defined(__x86_64__) || defined(__i386__)
    _mm_mfence();
#else
    __sync_synchronize();
#endif
}

static void fill_pattern(uint8_t *buf, size_t len, uint32_t seed) {
    uint32_t x = seed ? seed : 0x12345678u;
    for (size_t i = 0; i < len; i++) {
        x = x * 1664525u + 1013904223u;
        buf[i] = (uint8_t)(x >> 24);
    }
}

static void touch_sink(const uint8_t *buf, size_t len, volatile uint64_t *sink) {
    uint64_t v = *sink;
    for (size_t i = 0; i < len; i += 64) {
        v += buf[i];
    }
    if (len > 0) {
        v += buf[len - 1];
    }
    *sink = v;
}

static int parse_u64(const char *s, uint64_t *out) {
    char *end = NULL;
    errno = 0;
    uint64_t v = strtoull(s, &end, 0);
    if (errno || end == s) return -1;
    uint64_t mult = 1;
    if (*end) {
        if (!strcasecmp(end, "k") || !strcasecmp(end, "kb")) mult = 1024ull;
        else if (!strcasecmp(end, "m") || !strcasecmp(end, "mb")) mult = 1024ull * 1024ull;
        else if (!strcasecmp(end, "g") || !strcasecmp(end, "gb")) mult = 1024ull * 1024ull * 1024ull;
        else return -1;
    }
    if (v > UINT64_MAX / mult) return -1;
    *out = v * mult;
    return 0;
}

static int parse_sizes(const char *s, options_t *opt) {
    char *copy = strdup(s);
    if (!copy) return -1;
    opt->packet_count = 0;
    for (char *tok = strtok(copy, ","); tok; tok = strtok(NULL, ",")) {
        if (opt->packet_count >= MAX_PACKET_SIZES) {
            free(copy);
            return -1;
        }
        uint64_t v;
        if (parse_u64(tok, &v) || v == 0 || v > SIZE_MAX) {
            free(copy);
            return -1;
        }
        opt->packet_sizes[opt->packet_count++] = (size_t)v;
    }
    free(copy);
    return opt->packet_count ? 0 : -1;
}

static void set_defaults(options_t *opt) {
    memset(opt, 0, sizeof(*opt));
    opt->device = DEFAULT_DEVICE;
    opt->bar = DEFAULT_BAR;
    opt->read_tests = true;
    opt->write_tests = true;
    opt->map_size = 0;
    opt->seconds = DEFAULT_SECONDS;
    opt->latency_iters = DEFAULT_LATENCY_ITERS;
    opt->bw_iters = DEFAULT_BW_ITERS;
    opt->packet_count = sizeof(default_packet_sizes) / sizeof(default_packet_sizes[0]);
    memcpy(opt->packet_sizes, default_packet_sizes, sizeof(default_packet_sizes));
}

static int parse_args(int argc, char **argv, options_t *opt) {
    set_defaults(opt);
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
#define NEED_ARG() do { if (++i >= argc) { fprintf(stderr, "%s requires an argument\n", a); return -1; } } while (0)
        if (!strcmp(a, "-p") || !strcmp(a, "--path")) { NEED_ARG(); opt->path = argv[i]; }
        else if (!strcmp(a, "-d") || !strcmp(a, "--device")) { NEED_ARG(); opt->device = argv[i]; }
        else if (!strcmp(a, "-b") || !strcmp(a, "--bar")) { uint64_t v; NEED_ARG(); if (parse_u64(argv[i], &v) || v > 5) return -1; opt->bar = (int)v; }
        else if (!strcmp(a, "--wc")) opt->wc = true;
        else if (!strcmp(a, "-o") || !strcmp(a, "--offset")) { uint64_t v; NEED_ARG(); if (parse_u64(argv[i], &v) || v > INT64_MAX) return -1; opt->offset = (off_t)v; }
        else if (!strcmp(a, "-m") || !strcmp(a, "--map-size")) { uint64_t v; NEED_ARG(); if (parse_u64(argv[i], &v) || v == 0 || v > SIZE_MAX) return -1; opt->map_size = (size_t)v; }
        else if (!strcmp(a, "-s") || !strcmp(a, "--sizes")) { NEED_ARG(); if (parse_sizes(argv[i], opt)) return -1; }
        else if (!strcmp(a, "-t") || !strcmp(a, "--seconds")) { NEED_ARG(); opt->seconds = strtod(argv[i], NULL); if (opt->seconds <= 0.0) return -1; }
        else if (!strcmp(a, "-n") || !strcmp(a, "--latency-iters")) { uint64_t v; NEED_ARG(); if (parse_u64(argv[i], &v) || v == 0) return -1; opt->latency_iters = v; }
        else if (!strcmp(a, "--bw-iters")) { uint64_t v; NEED_ARG(); if (parse_u64(argv[i], &v)) return -1; opt->bw_iters = v; }
        else if (!strcmp(a, "--read-only")) { opt->read_tests = true; opt->write_tests = false; }
        else if (!strcmp(a, "--write-only")) { opt->read_tests = false; opt->write_tests = true; }
        else if (!strcmp(a, "--unsafe-writes")) opt->unsafe_writes = true;
        else if (!strcmp(a, "--verify")) opt->verify = true;
        else if (!strcmp(a, "--csv")) opt->csv = true;
        else if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); exit(0); }
        else { fprintf(stderr, "Unknown option: %s\n", a); return -1; }
#undef NEED_ARG
    }
    return 0;
}

static char *build_path(const options_t *opt) {
    if (opt->path) return strdup(opt->path);
    char *p = NULL;
    if (asprintf(&p, "/sys/bus/pci/devices/%s/resource%d%s", opt->device, opt->bar, opt->wc ? "_wc" : "") < 0) {
        return NULL;
    }
    return p;
}

static uint64_t choose_bw_iters(const options_t *opt, size_t packet_size, size_t span) {
    if (opt->bw_iters) return opt->bw_iters;
    uint64_t chunks_per_pass = span / packet_size;
    if (!chunks_per_pass) chunks_per_pass = 1;
    uint64_t target_bytes = (uint64_t)(512.0 * 1024.0 * 1024.0 * opt->seconds);
    uint64_t iters = target_bytes / (packet_size * chunks_per_pass);
    if (iters < 1) iters = 1;
    return iters;
}

static void run_latency(volatile uint8_t *bar, uint8_t *buf, const options_t *opt, size_t packet_size, bool write, volatile uint64_t *sink) {
    uint64_t start = nsec_now();
    if (write) {
        for (uint64_t i = 0; i < opt->latency_iters; i++) {
            memcpy((void *)bar, buf, packet_size);
            mmio_fence();
        }
    } else {
        for (uint64_t i = 0; i < opt->latency_iters; i++) {
            memcpy(buf, (const void *)bar, packet_size);
            touch_sink(buf, packet_size, sink);
        }
    }
    uint64_t elapsed = nsec_now() - start;
    double ns_per_op = (double)elapsed / (double)opt->latency_iters;
    double mbps = ((double)packet_size * (double)opt->latency_iters) / ((double)elapsed / 1e9) / (1024.0 * 1024.0);
    if (opt->csv) {
        printf("latency,%s,%zu,%.2f,%.2f,\n", write ? "write" : "read", packet_size, ns_per_op, mbps);
    } else {
        printf("  %-5s latency: packet=%6zu B  %10.2f ns/op  %10.2f MiB/s\n",
               write ? "write" : "read", packet_size, ns_per_op, mbps);
    }
}

static void run_bandwidth(volatile uint8_t *bar, uint8_t *buf, const options_t *opt, size_t packet_size,
                          size_t span, bool write, volatile uint64_t *sink) {
    uint64_t chunks = span / packet_size;
    if (!chunks) chunks = 1;
    uint64_t target_iters = choose_bw_iters(opt, packet_size, span);
    uint64_t completed_iters = 0;
    uint64_t bytes = 0;
    uint64_t start = nsec_now();

    for (;;) {
        if (write) {
            for (uint64_t i = 0; i < chunks; i++) {
                memcpy((void *)(bar + i * packet_size), buf, packet_size);
            }
            mmio_fence();
        } else {
            for (uint64_t i = 0; i < chunks; i++) {
                memcpy(buf, (const void *)(bar + i * packet_size), packet_size);
                touch_sink(buf, packet_size, sink);
            }
        }

        completed_iters++;
        bytes += chunks * packet_size;
        if (opt->bw_iters) {
            if (completed_iters >= target_iters) break;
        } else if ((double)(nsec_now() - start) / 1e9 >= opt->seconds) {
            break;
        }
    }

    uint64_t elapsed = nsec_now() - start;
    double sec = (double)elapsed / 1e9;
    double mbps = (double)bytes / sec / (1024.0 * 1024.0);
    if (opt->csv) {
        printf("bandwidth,%s,%zu,%.6f,%.2f,%llu\n", write ? "write" : "read", packet_size, sec, mbps, (unsigned long long)bytes);
    } else {
        printf("  %-5s bandwidth: packet=%6zu B  %10.2f MiB/s  %.3f s  %llu bytes\n",
               write ? "write" : "read", packet_size, mbps, sec, (unsigned long long)bytes);
    }
}

static size_t max_packet_size(const options_t *opt) {
    size_t max = 0;
    for (size_t i = 0; i < opt->packet_count; i++) {
        if (opt->packet_sizes[i] > max) max = opt->packet_sizes[i];
    }
    return max;
}

static int verify_region(volatile uint8_t *bar, const uint8_t *expected, uint8_t *tmp, size_t packet_size) {
    memcpy(tmp, (const void *)bar, packet_size);
    return memcmp(tmp, expected, packet_size);
}

int main(int argc, char **argv) {
    options_t opt;
    if (parse_args(argc, argv, &opt)) {
        usage(argv[0]);
        return 2;
    }

    if (opt.write_tests && !opt.unsafe_writes) {
        fprintf(stderr, "Refusing BAR write tests without --unsafe-writes. Use --read-only for safe read tests.\n");
        return 2;
    }

    char *path = build_path(&opt);
    if (!path) {
        perror("build path");
        return 1;
    }

    int open_flags = opt.write_tests ? O_RDWR : O_RDONLY;
    int fd = open(path, open_flags | O_SYNC);
    if (fd < 0) {
        perror(path);
        free(path);
        return 1;
    }

    struct stat st;
    if (fstat(fd, &st) != 0) {
        perror("fstat");
        close(fd);
        free(path);
        return 1;
    }

    size_t map_size = opt.map_size ? opt.map_size : DEFAULT_MAP_SIZE;
    if (st.st_size > 0) {
        off_t available = st.st_size - opt.offset;
        if (available <= 0) {
            fprintf(stderr, "Offset is outside resource: offset=%lld size=%lld\n", (long long)opt.offset, (long long)st.st_size);
            close(fd);
            free(path);
            return 1;
        }
        if (!opt.map_size && (uint64_t)available < map_size) map_size = (size_t)available;
        if (opt.map_size && (uint64_t)opt.map_size > (uint64_t)available) {
            fprintf(stderr, "Requested map size exceeds resource: requested=%zu available=%llu\n", opt.map_size, (unsigned long long)available);
            close(fd);
            free(path);
            return 1;
        }
    }

    size_t max_pkt = max_packet_size(&opt);
    if (max_pkt > map_size) {
        fprintf(stderr, "Largest packet size (%zu) exceeds mapped size (%zu)\n", max_pkt, map_size);
        close(fd);
        free(path);
        return 1;
    }

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        perror("sysconf(_SC_PAGESIZE)");
        close(fd);
        free(path);
        return 1;
    }
    off_t map_offset = opt.offset & ~((off_t)page_size - 1);
    size_t page_delta = (size_t)(opt.offset - map_offset);
    size_t mmap_size = map_size + page_delta;
    if (mmap_size < map_size) {
        fprintf(stderr, "Mapping size overflow\n");
        close(fd);
        free(path);
        return 1;
    }

    int mmap_prot = opt.write_tests ? (PROT_READ | PROT_WRITE) : PROT_READ;
    void *map_base = mmap(NULL, mmap_size, mmap_prot, MAP_SHARED, fd, map_offset);
    if (map_base == MAP_FAILED) {
        perror("mmap");
        close(fd);
        free(path);
        return 1;
    }
    void *map = (uint8_t *)map_base + page_delta;

    uint8_t *buf = NULL;
    uint8_t *tmp = NULL;
    if (posix_memalign((void **)&buf, 64, max_pkt ? max_pkt : 64) ||
        posix_memalign((void **)&tmp, 64, max_pkt ? max_pkt : 64)) {
        perror("posix_memalign");
        munmap(map_base, mmap_size);
        close(fd);
        free(path);
        return 1;
    }
    fill_pattern(buf, max_pkt, 0x88u);

    volatile uint64_t sink = 0;
    volatile uint8_t *bar = (volatile uint8_t *)map;

    if (opt.csv) {
        printf("test,op,packet_bytes,seconds_or_ns,mib_per_sec,bytes\n");
    } else {
        printf("BAR path: %s\n", path);
        printf("Mapped: offset=%lld size=%zu bytes%s\n", (long long)opt.offset, map_size, opt.wc ? " (WC)" : "");
        printf("Tests: %s%s, latency_iters=%llu, bandwidth=%s\n",
               opt.read_tests ? "read" : "", (opt.read_tests && opt.write_tests) ? "+write" : (opt.write_tests ? "write" : ""),
               (unsigned long long)opt.latency_iters, opt.bw_iters ? "fixed iterations" : "time based");
    }

    for (size_t i = 0; i < opt.packet_count; i++) {
        size_t pkt = opt.packet_sizes[i];
        if (!opt.csv) printf("\npacket=%zu bytes\n", pkt);
        if (opt.read_tests) run_latency(bar, buf, &opt, pkt, false, &sink);
        if (opt.write_tests) {
            run_latency(bar, buf, &opt, pkt, true, &sink);
            if (opt.verify && verify_region(bar, buf, tmp, pkt) != 0) {
                fprintf(stderr, "verify failed after write latency test, packet=%zu\n", pkt);
            }
        }
        if (opt.read_tests) run_bandwidth(bar, buf, &opt, pkt, map_size - (map_size % pkt), false, &sink);
        if (opt.write_tests) {
            run_bandwidth(bar, buf, &opt, pkt, map_size - (map_size % pkt), true, &sink);
            if (opt.verify && verify_region(bar, buf, tmp, pkt) != 0) {
                fprintf(stderr, "verify failed after write bandwidth test, packet=%zu\n", pkt);
            }
        }
    }

    if (!opt.csv) printf("\nsink=%llu\n", (unsigned long long)sink);

    free(buf);
    free(tmp);
    munmap(map_base, mmap_size);
    close(fd);
    free(path);
    return 0;
}
