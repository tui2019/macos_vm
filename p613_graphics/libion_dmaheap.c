/*
 * SM-P613 Mainline Linux 7.3 libion.so compatibility layer over DMA-BUF Heaps.
 *
 * Replaces CAF 4.14 /dev/ion libion.so so existing Android gralloc
 * (allocator@4.0, mapper@4.0, mapper@3.0, gralloc.qcom.so) allocates genuine
 * mainline kernel DMA-BUF file descriptors from /dev/dma_heap/system and
 * /dev/dma_heap/system-uncached.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <android/log.h>

#define LOG_TAG "libion_dmaheap"
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

/* Mainline Linux <linux/dma-heap.h> UAPI */
struct dma_heap_allocation_data {
    uint64_t len;
    uint32_t fd;
    uint32_t fd_flags;
    uint64_t heap_flags;
};

#define DMA_HEAP_IOC_MAGIC 'H'
#define DMA_HEAP_IOCTL_ALLOC _IOWR(DMA_HEAP_IOC_MAGIC, 0x0, struct dma_heap_allocation_data)

/* Mainline Linux <linux/dma-buf.h> UAPI */
struct dma_buf_sync {
    uint64_t flags;
};

#define DMA_BUF_SYNC_READ      (1ULL << 0)
#define DMA_BUF_SYNC_WRITE     (2ULL << 0)
#define DMA_BUF_SYNC_RW        (DMA_BUF_SYNC_READ | DMA_BUF_SYNC_WRITE)
#define DMA_BUF_SYNC_START     (0ULL << 2)
#define DMA_BUF_SYNC_END       (1ULL << 2)
#define DMA_BUF_BASE           'b'
#define DMA_BUF_IOCTL_SYNC     _IOW(DMA_BUF_BASE, 0, struct dma_buf_sync)

/* Legacy ION flags */
#define ION_FLAG_CACHED 1

struct ion_heap_data {
    char name[32];
    uint32_t type;
    uint32_t heap_id;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
};

typedef int ion_user_handle_t;

static pthread_mutex_t g_heap_lock = PTHREAD_MUTEX_INITIALIZER;
static int g_uncached_fd = -2; /* -2 = not tried yet, -1 = unavailable */

static int get_uncached_heap_fd(void) {
    pthread_mutex_lock(&g_heap_lock);
    if (g_uncached_fd == -2) {
        g_uncached_fd = open("/dev/dma_heap/system-uncached", O_RDONLY | O_CLOEXEC);
        if (g_uncached_fd < 0) {
            ALOGI("system-uncached heap unavailable (%s), falling back to system heap",
                  strerror(errno));
            g_uncached_fd = -1;
        }
    }
    int fd = g_uncached_fd;
    pthread_mutex_unlock(&g_heap_lock);
    return fd;
}

__attribute__((visibility("default")))
int ion_open(void) {
    int fd = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        ALOGE("Failed to open /dev/dma_heap/system: %s", strerror(errno));
        return -1;
    }
    return fd;
}

__attribute__((visibility("default")))
int ion_close(int fd) {
    if (fd < 0)
        return -EBADF;
    int ret = close(fd);
    return (ret < 0) ? -errno : 0;
}

__attribute__((visibility("default")))
int ion_alloc_fd(int fd, size_t len, size_t align, unsigned int heap_mask,
                 unsigned int flags, int *handle_fd) {
    (void)heap_mask;
    if (!handle_fd)
        return -EINVAL;
    *handle_fd = -1;

    if (len == 0)
        len = 4096;

    size_t page = (size_t)sysconf(_SC_PAGESIZE);
    if (page == 0)
        page = 4096;
    if (align < page)
        align = page;
    /* Round len up to page boundary */
    len = (len + page - 1) & ~(page - 1);

    int heap_fd = fd;
    if (!(flags & ION_FLAG_CACHED)) {
        int ufd = get_uncached_heap_fd();
        if (ufd >= 0)
            heap_fd = ufd;
    }

    struct dma_heap_allocation_data data;
    memset(&data, 0, sizeof(data));
    data.len = (uint64_t)len;
    data.fd_flags = O_RDWR | O_CLOEXEC;
    data.heap_flags = 0;

    int ret = ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data);
    if (ret < 0 && heap_fd != fd && fd >= 0) {
        /* Fallback to primary system heap */
        memset(&data, 0, sizeof(data));
        data.len = (uint64_t)len;
        data.fd_flags = O_RDWR | O_CLOEXEC;
        ret = ioctl(fd, DMA_HEAP_IOCTL_ALLOC, &data);
    }
    if (ret < 0) {
        int err = errno ? errno : EIO;
        ALOGE("DMA_HEAP_IOCTL_ALLOC(len=%zu, flags=0x%x) failed: %s",
              len, flags, strerror(err));
        return -err;
    }

    *handle_fd = (int)data.fd;
    return 0;
}

__attribute__((visibility("default")))
int ion_alloc(int fd, size_t len, size_t align, unsigned int heap_mask,
              unsigned int flags, ion_user_handle_t *handle) {
    int dmabuf_fd = -1;
    int ret = ion_alloc_fd(fd, len, align, heap_mask, flags, &dmabuf_fd);
    if (ret == 0 && handle)
        *handle = dmabuf_fd;
    return ret;
}

__attribute__((visibility("default")))
int ion_free(int fd, ion_user_handle_t handle) {
    (void)fd;
    if (handle >= 0)
        close(handle);
    return 0;
}

__attribute__((visibility("default")))
int ion_map(int fd, ion_user_handle_t handle, size_t length, int prot,
            int flags, off_t offset, unsigned char **ptr, int *map_fd) {
    (void)fd;
    if (!ptr || !map_fd)
        return -EINVAL;
    int dup_fd = fcntl(handle, F_DUPFD_CLOEXEC, 0);
    if (dup_fd < 0)
        return -errno;
    void *addr = mmap(NULL, length, prot, flags, dup_fd, offset);
    if (addr == MAP_FAILED) {
        int err = errno;
        close(dup_fd);
        return -err;
    }
    *ptr = (unsigned char *)addr;
    *map_fd = dup_fd;
    return 0;
}

__attribute__((visibility("default")))
int ion_share(int fd, ion_user_handle_t handle, int *share_fd) {
    (void)fd;
    if (!share_fd)
        return -EINVAL;
    int dup_fd = fcntl(handle, F_DUPFD_CLOEXEC, 0);
    if (dup_fd < 0)
        return -errno;
    *share_fd = dup_fd;
    return 0;
}

__attribute__((visibility("default")))
int ion_import(int fd, int share_fd, ion_user_handle_t *handle) {
    (void)fd;
    if (!handle)
        return -EINVAL;
    int dup_fd = fcntl(share_fd, F_DUPFD_CLOEXEC, 0);
    if (dup_fd < 0)
        return -errno;
    *handle = dup_fd;
    return 0;
}

__attribute__((visibility("default")))
int ion_sync_fd(int fd, int handle_fd) {
    (void)fd;
    struct dma_buf_sync sync;
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    if (ioctl(handle_fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
        return -errno;
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    if (ioctl(handle_fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
        return -errno;
    return 0;
}

__attribute__((visibility("default")))
int ion_is_legacy(int fd) {
    (void)fd;
    return 0;
}

__attribute__((visibility("default")))
int ion_is_using_modular_heaps(int fd) {
    (void)fd;
    return 1;
}

__attribute__((visibility("default")))
int ion_query_heap_cnt(int fd, int *cnt) {
    (void)fd;
    if (!cnt)
        return -EINVAL;
    *cnt = 1;
    return 0;
}

__attribute__((visibility("default")))
int ion_query_get_heaps(int fd, int cnt, void *buffers) {
    (void)fd;
    if (!buffers || cnt <= 0)
        return -EINVAL;
    struct ion_heap_data *heaps = (struct ion_heap_data *)buffers;
    memset(&heaps[0], 0, sizeof(heaps[0]));
    strncpy(heaps[0].name, "system", sizeof(heaps[0].name) - 1);
    heaps[0].type = 0; /* ION_HEAP_TYPE_SYSTEM */
    heaps[0].heap_id = 0;
    return 0;
}
