/*
 * SM-P613 Mainline Linux 7.3 DRM/KMS HWC2 HAL (hwcomposer.drm.so)
 *
 * Clean pure-C HWC2.4 implementation talking directly to mainline Linux
 * DRM/KMS atomic modesetting (/dev/dri/card* -> msm_dpu) with hardware
 * VSync, DMA-BUF client-target scanout, IN_FENCE_FD / OUT_FENCE_PTR sync
 * fences, and sysfs backlight/power control.
 *
 * Plugs directly into Android's existing android.hardware.graphics.composer@2.4-service.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <android/log.h>
#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define LOG_TAG "hwcomposer_drm"
#define ALOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define ALOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
#define ALOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

/* ---- Minimal self-contained Android libhardware / HWC2 ABI definitions ---- */

#define MAKE_TAG_CONSTANT(A, B, C, D) (((A) << 24) | ((B) << 16) | ((C) << 8) | (D))
#define HARDWARE_MODULE_TAG MAKE_TAG_CONSTANT('H', 'W', 'M', 'T')
#define HARDWARE_DEVICE_TAG MAKE_TAG_CONSTANT('H', 'W', 'D', 'T')
#define HWC_HARDWARE_MODULE_ID "hwcomposer"
#define HWC_HARDWARE_COMPOSER "composer"
#define HWC_MODULE_API_VERSION_0_1 0x0001
#define HWC_DEVICE_API_VERSION_2_0 0x02000001

typedef struct native_handle {
    int version;
    int numFds;
    int numInts;
    int data[];
} native_handle_t;
typedef const native_handle_t *buffer_handle_t;

struct hw_module_t;
struct hw_device_t;

typedef struct hw_module_methods_t {
    int (*open)(const struct hw_module_t *module, const char *id,
                struct hw_device_t **device);
} hw_module_methods_t;

typedef struct hw_module_t {
    uint32_t tag;
    uint16_t module_api_version;
    uint16_t hal_api_version;
    const char *id;
    const char *name;
    const char *author;
    struct hw_module_methods_t *methods;
    void *dso;
    uint64_t reserved[32 - 7];
} hw_module_t;

typedef struct hw_device_t {
    uint32_t tag;
    uint32_t version;
    struct hw_module_t *module;
    uint64_t reserved[12];
    int (*close)(struct hw_device_t *device);
} hw_device_t;

typedef void (*hwc2_function_pointer_t)(void);
typedef void *hwc2_callback_data_t;
typedef uint64_t hwc2_display_t;
typedef uint64_t hwc2_layer_t;
typedef uint32_t hwc2_config_t;
typedef int32_t hwc2_vsync_period_t;

typedef struct hwc2_device {
    struct hw_device_t common;
    void (*getCapabilities)(struct hwc2_device *device, uint32_t *outCount,
                            int32_t *outCapabilities);
    hwc2_function_pointer_t (*getFunction)(struct hwc2_device *device,
                                           int32_t descriptor);
} hwc2_device_t;

typedef struct hwc_rect {
    int32_t left;
    int32_t top;
    int32_t right;
    int32_t bottom;
} hwc_rect_t;

typedef struct hwc_frect {
    float left;
    float top;
    float right;
    float bottom;
} hwc_frect_t;

typedef struct hwc_region {
    size_t numRects;
    const hwc_rect_t *rects;
} hwc_region_t;

typedef struct hwc_color {
    uint8_t r, g, b, a;
} hwc_color_t;

typedef struct hwc_vsync_period_change_constraints {
    int64_t desiredTimeNanos;
    uint8_t seamlessRequired;
} hwc_vsync_period_change_constraints_t;

typedef struct hwc_vsync_period_change_timeline {
    int64_t newVsyncAppliedTimeNanos;
    uint8_t refreshRequired;
    int64_t refreshTimeNanos;
} hwc_vsync_period_change_timeline_t;

typedef struct hwc_client_target_property {
    int32_t pixelFormat;
    int32_t dataspace;
} hwc_client_target_property_t;

/* HWC2 enums matching AOSP hardware/libhardware/include/hardware/hwcomposer2.h */
enum {
    HWC2_ERROR_NONE = 0,
    HWC2_ERROR_BAD_CONFIG = 1,
    HWC2_ERROR_BAD_DISPLAY = 2,
    HWC2_ERROR_BAD_LAYER = 3,
    HWC2_ERROR_BAD_PARAMETER = 4,
    HWC2_ERROR_HAS_CHANGES = 5,
    HWC2_ERROR_NO_RESOURCES = 6,
    HWC2_ERROR_NOT_VALIDATED = 7,
    HWC2_ERROR_UNSUPPORTED = 8,
    HWC2_ERROR_SEAMLESS_NOT_ALLOWED = 9,
    HWC2_ERROR_SEAMLESS_NOT_POSSIBLE = 10,
};

enum {
    HWC2_ATTRIBUTE_WIDTH = 1,
    HWC2_ATTRIBUTE_HEIGHT = 2,
    HWC2_ATTRIBUTE_VSYNC_PERIOD = 3,
    HWC2_ATTRIBUTE_DPI_X = 4,
    HWC2_ATTRIBUTE_DPI_Y = 5,
    HWC2_ATTRIBUTE_CONFIG_GROUP = 7,
};

enum {
    HWC2_CALLBACK_HOTPLUG = 1,
    HWC2_CALLBACK_REFRESH = 2,
    HWC2_CALLBACK_VSYNC = 3,
    HWC2_CALLBACK_VSYNC_2_4 = 4,
    HWC2_CALLBACK_VSYNC_PERIOD_TIMING_CHANGED = 5,
    HWC2_CALLBACK_SEAMLESS_POSSIBLE = 6,
};

enum {
    HWC2_COMPOSITION_CLIENT = 1,
    HWC2_COMPOSITION_DEVICE = 2,
    HWC2_COMPOSITION_SOLID_COLOR = 3,
    HWC2_COMPOSITION_CURSOR = 4,
    HWC2_COMPOSITION_SIDEBAND = 5,
};

enum {
    HWC2_CONNECTION_INVALID = 0,
    HWC2_CONNECTION_CONNECTED = 1,
    HWC2_CONNECTION_DISCONNECTED = 2,
};

enum {
    HWC2_DISPLAY_TYPE_PHYSICAL = 1,
    HWC2_DISPLAY_TYPE_VIRTUAL = 2,
};

enum {
    HWC2_POWER_MODE_OFF = 0,
    HWC2_POWER_MODE_DOZE = 1,
    HWC2_POWER_MODE_ON = 2,
    HWC2_POWER_MODE_DOZE_SUSPEND = 3,
};

enum {
    HWC2_VSYNC_ENABLE = 1,
    HWC2_VSYNC_DISABLE = 2,
};

enum {
    HWC2_FUNCTION_INVALID = 0,
    HWC2_FUNCTION_ACCEPT_DISPLAY_CHANGES,
    HWC2_FUNCTION_CREATE_LAYER,
    HWC2_FUNCTION_CREATE_VIRTUAL_DISPLAY,
    HWC2_FUNCTION_DESTROY_LAYER,
    HWC2_FUNCTION_DESTROY_VIRTUAL_DISPLAY,
    HWC2_FUNCTION_DUMP,
    HWC2_FUNCTION_GET_ACTIVE_CONFIG,
    HWC2_FUNCTION_GET_CHANGED_COMPOSITION_TYPES,
    HWC2_FUNCTION_GET_CLIENT_TARGET_SUPPORT,
    HWC2_FUNCTION_GET_COLOR_MODES,
    HWC2_FUNCTION_GET_DISPLAY_ATTRIBUTE,
    HWC2_FUNCTION_GET_DISPLAY_CONFIGS,
    HWC2_FUNCTION_GET_DISPLAY_NAME,
    HWC2_FUNCTION_GET_DISPLAY_REQUESTS,
    HWC2_FUNCTION_GET_DISPLAY_TYPE,
    HWC2_FUNCTION_GET_DOZE_SUPPORT,
    HWC2_FUNCTION_GET_HDR_CAPABILITIES,
    HWC2_FUNCTION_GET_MAX_VIRTUAL_DISPLAY_COUNT,
    HWC2_FUNCTION_GET_RELEASE_FENCES,
    HWC2_FUNCTION_PRESENT_DISPLAY,
    HWC2_FUNCTION_REGISTER_CALLBACK,
    HWC2_FUNCTION_SET_ACTIVE_CONFIG,
    HWC2_FUNCTION_SET_CLIENT_TARGET,
    HWC2_FUNCTION_SET_COLOR_MODE,
    HWC2_FUNCTION_SET_COLOR_TRANSFORM,
    HWC2_FUNCTION_SET_CURSOR_POSITION,
    HWC2_FUNCTION_SET_LAYER_BLEND_MODE,
    HWC2_FUNCTION_SET_LAYER_BUFFER,
    HWC2_FUNCTION_SET_LAYER_COLOR,
    HWC2_FUNCTION_SET_LAYER_COMPOSITION_TYPE,
    HWC2_FUNCTION_SET_LAYER_DATASPACE,
    HWC2_FUNCTION_SET_LAYER_DISPLAY_FRAME,
    HWC2_FUNCTION_SET_LAYER_PLANE_ALPHA,
    HWC2_FUNCTION_SET_LAYER_SIDEBAND_STREAM,
    HWC2_FUNCTION_SET_LAYER_SOURCE_CROP,
    HWC2_FUNCTION_SET_LAYER_SURFACE_DAMAGE,
    HWC2_FUNCTION_SET_LAYER_TRANSFORM,
    HWC2_FUNCTION_SET_LAYER_VISIBLE_REGION,
    HWC2_FUNCTION_SET_LAYER_Z_ORDER,
    HWC2_FUNCTION_SET_OUTPUT_BUFFER,
    HWC2_FUNCTION_SET_POWER_MODE,
    HWC2_FUNCTION_SET_VSYNC_ENABLED,
    HWC2_FUNCTION_VALIDATE_DISPLAY,
    HWC2_FUNCTION_SET_LAYER_FLOAT_COLOR,
    HWC2_FUNCTION_SET_LAYER_PER_FRAME_METADATA,
    HWC2_FUNCTION_GET_PER_FRAME_METADATA_KEYS,
    HWC2_FUNCTION_SET_READBACK_BUFFER,
    HWC2_FUNCTION_GET_READBACK_BUFFER_ATTRIBUTES,
    HWC2_FUNCTION_GET_READBACK_BUFFER_FENCE,
    HWC2_FUNCTION_GET_RENDER_INTENTS,
    HWC2_FUNCTION_SET_COLOR_MODE_WITH_RENDER_INTENT,
    HWC2_FUNCTION_GET_DATASPACE_SATURATION_MATRIX,
    HWC2_FUNCTION_GET_DISPLAY_IDENTIFICATION_DATA,
    HWC2_FUNCTION_GET_DISPLAY_CAPABILITIES,
    HWC2_FUNCTION_SET_LAYER_COLOR_TRANSFORM,
    HWC2_FUNCTION_GET_DISPLAYED_CONTENT_SAMPLING_ATTRIBUTES,
    HWC2_FUNCTION_SET_DISPLAYED_CONTENT_SAMPLING_ENABLED,
    HWC2_FUNCTION_GET_DISPLAYED_CONTENT_SAMPLE,
    HWC2_FUNCTION_SET_LAYER_PER_FRAME_METADATA_BLOBS,
    HWC2_FUNCTION_GET_DISPLAY_BRIGHTNESS_SUPPORT,
    HWC2_FUNCTION_SET_DISPLAY_BRIGHTNESS,
    HWC2_FUNCTION_GET_DISPLAY_CONNECTION_TYPE,
    HWC2_FUNCTION_GET_DISPLAY_VSYNC_PERIOD,
    HWC2_FUNCTION_SET_ACTIVE_CONFIG_WITH_CONSTRAINTS,
    HWC2_FUNCTION_SET_AUTO_LOW_LATENCY_MODE,
    HWC2_FUNCTION_GET_SUPPORTED_CONTENT_TYPES,
    HWC2_FUNCTION_SET_CONTENT_TYPE,
    HWC2_FUNCTION_GET_CLIENT_TARGET_PROPERTY,
    HWC2_FUNCTION_SET_LAYER_GENERIC_METADATA,
    HWC2_FUNCTION_GET_LAYER_GENERIC_METADATA_KEY,
};

typedef void (*HWC2_PFN_HOTPLUG)(hwc2_callback_data_t, hwc2_display_t, int32_t);
typedef void (*HWC2_PFN_REFRESH)(hwc2_callback_data_t, hwc2_display_t);
typedef void (*HWC2_PFN_VSYNC)(hwc2_callback_data_t, hwc2_display_t, int64_t);
typedef void (*HWC2_PFN_VSYNC_2_4)(hwc2_callback_data_t, hwc2_display_t, int64_t, hwc2_vsync_period_t);

/* HAL pixel formats */
#define HAL_PIXEL_FORMAT_RGBA_8888 1
#define HAL_PIXEL_FORMAT_RGBX_8888 2
#define HAL_PIXEL_FORMAT_RGB_888   3
#define HAL_PIXEL_FORMAT_RGB_565   4
#define HAL_PIXEL_FORMAT_BGRA_8888 5

/* Verified QCOM private_handle_t layout on SM-P613 (gmsm magic at offset 0x14) */
#define QCOM_GRALLOC_MAGIC (('g' << 24) | ('m' << 16) | ('s' << 8) | 'm')

/* ---- Internal state ---- */

#define PRIMARY_DISPLAY_ID 0ULL
#define PRIMARY_CONFIG_ID  0U
#define MAX_LAYERS         256
#define FB_CACHE_SIZE      16

typedef struct {
    bool in_use;
    hwc2_layer_t id;
    int32_t comp_type;
    int32_t pending_type;
    bool changed;
} drm_hwc_layer_t;

typedef struct {
    bool valid;
    uint64_t key;
    uint32_t gem_handle;
    uint32_t fb_id;
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint32_t format;
    uint64_t last_used;
} fb_cache_entry_t;

typedef struct {
    hwc2_device_t dev;
    pthread_mutex_t lock;

    /* DRM/KMS objects */
    int drm_fd;
    uint32_t conn_id;
    uint32_t crtc_id;
    uint32_t crtc_idx;
    uint32_t plane_id;
    uint32_t mode_blob_id;
    drmModeModeInfo mode;

    /* Display metrics */
    int32_t disp_w;
    int32_t disp_h;
    int32_t dpi_x;
    int32_t dpi_y;
    int32_t vsync_period_ns;

    /* Atomic property IDs */
    uint32_t prop_conn_crtc_id;
    uint32_t prop_crtc_active;
    uint32_t prop_crtc_mode_id;
    uint32_t prop_crtc_out_fence_ptr;
    uint32_t prop_plane_fb_id;
    uint32_t prop_plane_crtc_id;
    uint32_t prop_plane_crtc_x;
    uint32_t prop_plane_crtc_y;
    uint32_t prop_plane_crtc_w;
    uint32_t prop_plane_crtc_h;
    uint32_t prop_plane_src_x;
    uint32_t prop_plane_src_y;
    uint32_t prop_plane_src_w;
    uint32_t prop_plane_src_h;
    uint32_t prop_plane_in_fence_fd;

    /* Commit / power state */
    bool needs_modeset;
    int32_t power_mode;
    uint32_t active_fb_id;
    uint32_t prev_fb_id;
    uint64_t frame_seq;
    fb_cache_entry_t fb_cache[FB_CACHE_SIZE];

    /* Backlight sysfs */
    char bl_path[128];
    int bl_max;
    int bl_last_on;

    /* Client target for current frame */
    buffer_handle_t client_target;
    int client_acquire_fence;
    bool validated;

    /* Layers */
    hwc2_layer_t next_layer_id;
    drm_hwc_layer_t layers[MAX_LAYERS];

    /* Callbacks & VSync thread */
    HWC2_PFN_HOTPLUG cb_hotplug;
    hwc2_callback_data_t cb_hotplug_data;
    HWC2_PFN_REFRESH cb_refresh;
    hwc2_callback_data_t cb_refresh_data;
    HWC2_PFN_VSYNC cb_vsync;
    hwc2_callback_data_t cb_vsync_data;
    HWC2_PFN_VSYNC_2_4 cb_vsync_2_4;
    hwc2_callback_data_t cb_vsync_2_4_data;

    pthread_t vsync_thread;
    bool vsync_thread_running;
    bool vsync_enabled;
    pthread_cond_t vsync_cond;
} drm_hwc2_ctx_t;

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static uint32_t get_prop_id(int fd, uint32_t obj_id, uint32_t obj_type, const char *name) {
    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(fd, obj_id, obj_type);
    if (!props)
        return 0;
    uint32_t found = 0;
    for (uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyPtr p = drmModeGetProperty(fd, props->props[i]);
        if (p) {
            if (strcmp(p->name, name) == 0)
                found = p->prop_id;
            drmModeFreeProperty(p);
            if (found)
                break;
        }
    }
    drmModeFreeObjectProperties(props);
    return found;
}

static uint64_t get_prop_val(int fd, uint32_t obj_id, uint32_t obj_type, const char *name) {
    drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(fd, obj_id, obj_type);
    if (!props)
        return 0;
    uint64_t val = 0;
    for (uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyPtr p = drmModeGetProperty(fd, props->props[i]);
        if (p) {
            if (strcmp(p->name, name) == 0) {
                val = props->prop_values[i];
                drmModeFreeProperty(p);
                break;
            }
            drmModeFreeProperty(p);
        }
    }
    drmModeFreeObjectProperties(props);
    return val;
}

static void init_backlight(drm_hwc2_ctx_t *ctx) {
    const char *candidates[] = {
        "/sys/class/backlight/ae94000.dsi.0",
        "/sys/class/backlight/panel0-backlight",
        NULL
    };
    ctx->bl_path[0] = '\0';
    ctx->bl_max = 4095;
    ctx->bl_last_on = 2048;

    for (int i = 0; candidates[i]; i++) {
        char max_path[160];
        snprintf(max_path, sizeof(max_path), "%s/max_brightness", candidates[i]);
        FILE *f = fopen(max_path, "r");
        if (f) {
            int maxv = 4095;
            if (fscanf(f, "%d", &maxv) == 1 && maxv > 0)
                ctx->bl_max = maxv;
            fclose(f);
            snprintf(ctx->bl_path, sizeof(ctx->bl_path), "%s/brightness", candidates[i]);
            ctx->bl_last_on = ctx->bl_max / 2;
            ALOGI("Found backlight %s (max=%d)", ctx->bl_path, ctx->bl_max);
            break;
        }
    }
}

static void write_backlight(drm_hwc2_ctx_t *ctx, int val) {
    if (!ctx->bl_path[0])
        return;
    if (val < 0) val = 0;
    if (val > ctx->bl_max) val = ctx->bl_max;
    int fd = open(ctx->bl_path, O_WRONLY);
    if (fd >= 0) {
        char buf[16];
        int n = snprintf(buf, sizeof(buf), "%d", val);
        if (n > 0)
            (void)write(fd, buf, (size_t)n);
        close(fd);
    }
}

static int drm_init_kms(drm_hwc2_ctx_t *ctx) {
    for (int idx = 0; idx < 8; idx++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/dri/card%d", idx);
        int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0)
            continue;

        if (drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0 ||
            drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0) {
            close(fd);
            continue;
        }

        drmModeResPtr res = drmModeGetResources(fd);
        if (!res || res->count_connectors == 0 || res->count_crtcs == 0) {
            if (res) drmModeFreeResources(res);
            close(fd);
            continue;
        }

        drmModeConnectorPtr conn = NULL;
        for (int c = 0; c < res->count_connectors; c++) {
            drmModeConnectorPtr cand = drmModeGetConnector(fd, res->connectors[c]);
            if (cand && cand->connection == DRM_MODE_CONNECTED && cand->count_modes > 0) {
                conn = cand;
                break;
            }
            if (cand) drmModeFreeConnector(cand);
        }
        if (!conn) {
            drmModeFreeResources(res);
            close(fd);
            continue;
        }

        /* Find CRTC */
        uint32_t crtc_id = 0;
        uint32_t crtc_idx = 0;
        if (conn->encoder_id) {
            drmModeEncoderPtr enc = drmModeGetEncoder(fd, conn->encoder_id);
            if (enc) {
                if (enc->crtc_id) {
                    crtc_id = enc->crtc_id;
                    for (int k = 0; k < res->count_crtcs; k++) {
                        if (res->crtcs[k] == crtc_id) {
                            crtc_idx = (uint32_t)k;
                            break;
                        }
                    }
                } else {
                    for (int k = 0; k < res->count_crtcs; k++) {
                        if (enc->possible_crtcs & (1u << k)) {
                            crtc_id = res->crtcs[k];
                            crtc_idx = (uint32_t)k;
                            break;
                        }
                    }
                }
                drmModeFreeEncoder(enc);
            }
        }
        if (!crtc_id) {
            crtc_id = res->crtcs[0];
            crtc_idx = 0;
        }

        /* Find primary plane for crtc_idx */
        uint32_t plane_id = 0;
        drmModePlaneResPtr pres = drmModeGetPlaneResources(fd);
        if (pres) {
            for (uint32_t p = 0; p < pres->count_planes; p++) {
                drmModePlanePtr pl = drmModeGetPlane(fd, pres->planes[p]);
                if (!pl) continue;
                if (pl->possible_crtcs & (1u << crtc_idx)) {
                    uint64_t type = get_prop_val(fd, pl->plane_id, DRM_MODE_OBJECT_PLANE, "type");
                    if (type == DRM_PLANE_TYPE_PRIMARY) {
                        plane_id = pl->plane_id;
                        drmModeFreePlane(pl);
                        break;
                    }
                }
                drmModeFreePlane(pl);
            }
            drmModeFreePlaneResources(pres);
        }

        if (!plane_id) {
            drmModeFreeConnector(conn);
            drmModeFreeResources(res);
            close(fd);
            continue;
        }

        /* Pick preferred mode (or mode 0) */
        drmModeModeInfo mode = conn->modes[0];
        for (int m = 0; m < conn->count_modes; m++) {
            if (conn->modes[m].type & DRM_MODE_TYPE_PREFERRED) {
                mode = conn->modes[m];
                break;
            }
        }

        ctx->drm_fd = fd;
        ctx->conn_id = conn->connector_id;
        ctx->crtc_id = crtc_id;
        ctx->crtc_idx = crtc_idx;
        ctx->plane_id = plane_id;
        ctx->mode = mode;
        ctx->disp_w = mode.hdisplay;
        ctx->disp_h = mode.vdisplay;

        uint32_t vref = mode.vrefresh ? mode.vrefresh : 60;
        ctx->vsync_period_ns = (int32_t)(1000000000ULL / vref);
        if (conn->mmWidth > 0 && conn->mmHeight > 0) {
            ctx->dpi_x = (int32_t)((1000.0 * ctx->disp_w * 25.4) / conn->mmWidth);
            ctx->dpi_y = (int32_t)((1000.0 * ctx->disp_h * 25.4) / conn->mmHeight);
        } else {
            ctx->dpi_x = 225000;
            ctx->dpi_y = 225000;
        }

        drmModeCreatePropertyBlob(fd, &ctx->mode, sizeof(ctx->mode), &ctx->mode_blob_id);

        ctx->prop_conn_crtc_id = get_prop_id(fd, ctx->conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID");
        ctx->prop_crtc_active = get_prop_id(fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE");
        ctx->prop_crtc_mode_id = get_prop_id(fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID");
        ctx->prop_crtc_out_fence_ptr = get_prop_id(fd, ctx->crtc_id, DRM_MODE_OBJECT_CRTC, "OUT_FENCE_PTR");
        ctx->prop_plane_fb_id = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID");
        ctx->prop_plane_crtc_id = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID");
        ctx->prop_plane_crtc_x = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X");
        ctx->prop_plane_crtc_y = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y");
        ctx->prop_plane_crtc_w = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W");
        ctx->prop_plane_crtc_h = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H");
        ctx->prop_plane_src_x = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X");
        ctx->prop_plane_src_y = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y");
        ctx->prop_plane_src_w = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W");
        ctx->prop_plane_src_h = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H");
        ctx->prop_plane_in_fence_fd = get_prop_id(fd, ctx->plane_id, DRM_MODE_OBJECT_PLANE, "IN_FENCE_FD");

        drmModeFreeConnector(conn);
        drmModeFreeResources(res);

        ALOGI("Initialized KMS on %s: conn=%u crtc=%u(idx=%u) plane=%u mode=%dx%d@%uHz dpi=%d/%d",
              path, ctx->conn_id, ctx->crtc_id, ctx->crtc_idx, ctx->plane_id,
              ctx->disp_w, ctx->disp_h, vref, ctx->dpi_x, ctx->dpi_y);
        return 0;
    }

    ALOGE("Failed to find any connected KMS display on /dev/dri/card0..7");
    return -ENODEV;
}

static void fb_cache_clear(drm_hwc2_ctx_t *ctx) {
    for (int i = 0; i < FB_CACHE_SIZE; i++) {
        if (ctx->fb_cache[i].valid) {
            drmModeRmFB(ctx->drm_fd, ctx->fb_cache[i].fb_id);
            struct drm_gem_close gc = { .handle = ctx->fb_cache[i].gem_handle };
            drmIoctl(ctx->drm_fd, DRM_IOCTL_GEM_CLOSE, &gc);
            ctx->fb_cache[i].valid = false;
        }
    }
}

static uint32_t import_buffer_to_fb(drm_hwc2_ctx_t *ctx, buffer_handle_t handle,
                                    uint32_t *out_w, uint32_t *out_h) {
    if (!handle || handle->numFds < 1)
        return 0;

    int dmabuf_fd = handle->data[0];
    if (dmabuf_fd < 0)
        return 0;

    uint32_t w = (uint32_t)ctx->disp_w;
    uint32_t h = (uint32_t)ctx->disp_h;
    uint32_t stride_px = w;
    int hal_fmt = HAL_PIXEL_FORMAT_RGBA_8888;
    uint64_t cache_key = 0;

    /* Check QCOM private_handle_t ('gmsm' magic at byte offset 0x14) */
    const uint8_t *raw = (const uint8_t *)handle;
    if (handle->numFds >= 2 && handle->numInts >= 12) {
        int32_t magic = 0;
        memcpy(&magic, raw + 0x14, sizeof(magic));
        if ((uint32_t)magic == QCOM_GRALLOC_MAGIC) {
            int32_t aw = 0, ah = 0, uw = 0, uh = 0, fmt = 0;
            uint64_t id = 0;
            memcpy(&aw, raw + 0x1c, sizeof(aw));
            memcpy(&ah, raw + 0x20, sizeof(ah));
            memcpy(&uw, raw + 0x24, sizeof(uw));
            memcpy(&uh, raw + 0x28, sizeof(uh));
            memcpy(&fmt, raw + 0x2c, sizeof(fmt));
            memcpy(&id, raw + 0x38, sizeof(id));
            if (aw > 0) stride_px = (uint32_t)aw;
            if (uw > 0) w = (uint32_t)uw; else if (aw > 0) w = (uint32_t)aw;
            if (uh > 0) h = (uint32_t)uh; else if (ah > 0) h = (uint32_t)ah;
            if (fmt > 0) hal_fmt = fmt;
            cache_key = id;
        }
    }

    uint32_t bpp = (hal_fmt == HAL_PIXEL_FORMAT_RGB_565) ? 2 : 4;
    uint32_t pitch = stride_px * bpp;
    uint32_t drm_fmt = DRM_FORMAT_ABGR8888;
    switch (hal_fmt) {
        case HAL_PIXEL_FORMAT_RGBA_8888: drm_fmt = DRM_FORMAT_ABGR8888; break;
        case HAL_PIXEL_FORMAT_RGBX_8888: drm_fmt = DRM_FORMAT_XBGR8888; break;
        case HAL_PIXEL_FORMAT_BGRA_8888: drm_fmt = DRM_FORMAT_ARGB8888; break;
        case HAL_PIXEL_FORMAT_RGB_565:   drm_fmt = DRM_FORMAT_RGB565;   break;
        default:                         drm_fmt = DRM_FORMAT_XBGR8888; break;
    }

    *out_w = w;
    *out_h = h;

    /* Check cache if we have a unique gralloc buffer ID */
    if (cache_key != 0) {
        for (int i = 0; i < FB_CACHE_SIZE; i++) {
            fb_cache_entry_t *e = &ctx->fb_cache[i];
            if (e->valid && e->key == cache_key && e->width == w &&
                e->height == h && e->pitch == pitch && e->format == drm_fmt) {
                e->last_used = ++ctx->frame_seq;
                return e->fb_id;
            }
        }
    }

    uint32_t gem_handle = 0;
    if (drmPrimeFDToHandle(ctx->drm_fd, dmabuf_fd, &gem_handle) != 0) {
        ALOGE("drmPrimeFDToHandle(fd=%d) failed: %s", dmabuf_fd, strerror(errno));
        return 0;
    }

    uint32_t handles[4] = { gem_handle, 0, 0, 0 };
    uint32_t pitches[4] = { pitch, 0, 0, 0 };
    uint32_t offsets[4] = { 0, 0, 0, 0 };
    uint32_t fb_id = 0;

    int ret = drmModeAddFB2(ctx->drm_fd, w, h, drm_fmt, handles, pitches, offsets, &fb_id, 0);
    if (ret != 0 && drm_fmt == DRM_FORMAT_ABGR8888) {
        drm_fmt = DRM_FORMAT_XBGR8888;
        ret = drmModeAddFB2(ctx->drm_fd, w, h, drm_fmt, handles, pitches, offsets, &fb_id, 0);
    }
    if (ret != 0) {
        ALOGE("drmModeAddFB2(%ux%u pitch=%u fmt=0x%08x) failed: %s",
              w, h, pitch, drm_fmt, strerror(errno));
        struct drm_gem_close gc = { .handle = gem_handle };
        drmIoctl(ctx->drm_fd, DRM_IOCTL_GEM_CLOSE, &gc);
        return 0;
    }

    /* Insert into cache, evicting oldest unused slot if full */
    int slot = -1;
    uint64_t oldest = UINT64_MAX;
    for (int i = 0; i < FB_CACHE_SIZE; i++) {
        if (!ctx->fb_cache[i].valid) {
            slot = i;
            break;
        }
        if (ctx->fb_cache[i].fb_id != ctx->active_fb_id &&
            ctx->fb_cache[i].fb_id != ctx->prev_fb_id &&
            ctx->fb_cache[i].last_used < oldest) {
            oldest = ctx->fb_cache[i].last_used;
            slot = i;
        }
    }
    if (slot >= 0) {
        if (ctx->fb_cache[slot].valid) {
            drmModeRmFB(ctx->drm_fd, ctx->fb_cache[slot].fb_id);
            struct drm_gem_close gc = { .handle = ctx->fb_cache[slot].gem_handle };
            drmIoctl(ctx->drm_fd, DRM_IOCTL_GEM_CLOSE, &gc);
        }
        ctx->fb_cache[slot].valid = true;
        ctx->fb_cache[slot].key = cache_key;
        ctx->fb_cache[slot].gem_handle = gem_handle;
        ctx->fb_cache[slot].fb_id = fb_id;
        ctx->fb_cache[slot].width = w;
        ctx->fb_cache[slot].height = h;
        ctx->fb_cache[slot].pitch = pitch;
        ctx->fb_cache[slot].format = drm_fmt;
        ctx->fb_cache[slot].last_used = ++ctx->frame_seq;
    }
    return fb_id;
}

/* ---- VSync thread ---- */

static void *vsync_thread_fn(void *arg) {
    drm_hwc2_ctx_t *ctx = (drm_hwc2_ctx_t *)arg;
    while (1) {
        pthread_mutex_lock(&ctx->lock);
        while (ctx->vsync_thread_running && !ctx->vsync_enabled) {
            pthread_cond_wait(&ctx->vsync_cond, &ctx->lock);
        }
        if (!ctx->vsync_thread_running) {
            pthread_mutex_unlock(&ctx->lock);
            break;
        }
        int drm_fd = ctx->drm_fd;
        uint32_t crtc_idx = ctx->crtc_idx;
        int32_t period = ctx->vsync_period_ns;
        pthread_mutex_unlock(&ctx->lock);

        int64_t ts_ns = 0;
        drmVBlank vbl;
        memset(&vbl, 0, sizeof(vbl));
        vbl.request.type = DRM_VBLANK_RELATIVE;
        if (crtc_idx == 1)
            vbl.request.type |= DRM_VBLANK_SECONDARY;
        else if (crtc_idx > 1)
            vbl.request.type |= (crtc_idx << DRM_VBLANK_HIGH_CRTC_SHIFT);
        vbl.request.sequence = 1;

        if (drmWaitVBlank(drm_fd, &vbl) == 0) {
            ts_ns = (int64_t)vbl.reply.tval_sec * 1000000000LL +
                    (int64_t)vbl.reply.tval_usec * 1000LL;
            if (ts_ns <= 0)
                ts_ns = (int64_t)now_ns();
        } else {
            struct timespec req = { 0, period > 0 ? period : 16666667L };
            nanosleep(&req, NULL);
            ts_ns = (int64_t)now_ns();
        }

        pthread_mutex_lock(&ctx->lock);
        bool enabled = ctx->vsync_enabled && ctx->vsync_thread_running;
        HWC2_PFN_VSYNC_2_4 cb24 = ctx->cb_vsync_2_4;
        hwc2_callback_data_t data24 = ctx->cb_vsync_2_4_data;
        HWC2_PFN_VSYNC cb = ctx->cb_vsync;
        hwc2_callback_data_t data = ctx->cb_vsync_data;
        pthread_mutex_unlock(&ctx->lock);

        if (enabled) {
            if (cb24)
                cb24(data24, PRIMARY_DISPLAY_ID, ts_ns, period);
            else if (cb)
                cb(data, PRIMARY_DISPLAY_ID, ts_ns);
        }
    }
    return NULL;
}

/* ---- HWC2 API Implementations ---- */

static inline drm_hwc2_ctx_t *to_ctx(hwc2_device_t *dev) {
    return (drm_hwc2_ctx_t *)dev;
}

static drm_hwc_layer_t *find_layer(drm_hwc2_ctx_t *ctx, hwc2_layer_t id) {
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (ctx->layers[i].in_use && ctx->layers[i].id == id)
            return &ctx->layers[i];
    }
    return NULL;
}

static int32_t hwc2_accept_display_changes(hwc2_device_t *dev, hwc2_display_t display) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    pthread_mutex_lock(&ctx->lock);
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (ctx->layers[i].in_use && ctx->layers[i].changed) {
            ctx->layers[i].comp_type = ctx->layers[i].pending_type;
            ctx->layers[i].changed = false;
        }
    }
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_create_layer(hwc2_device_t *dev, hwc2_display_t display,
                                 hwc2_layer_t *outLayer) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (!outLayer) return HWC2_ERROR_BAD_PARAMETER;
    pthread_mutex_lock(&ctx->lock);
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (!ctx->layers[i].in_use) {
            ctx->layers[i].in_use = true;
            ctx->layers[i].id = ++ctx->next_layer_id;
            ctx->layers[i].comp_type = HWC2_COMPOSITION_CLIENT;
            ctx->layers[i].pending_type = HWC2_COMPOSITION_CLIENT;
            ctx->layers[i].changed = false;
            *outLayer = ctx->layers[i].id;
            pthread_mutex_unlock(&ctx->lock);
            return HWC2_ERROR_NONE;
        }
    }
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NO_RESOURCES;
}

static int32_t hwc2_destroy_layer(hwc2_device_t *dev, hwc2_display_t display,
                                  hwc2_layer_t layer) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    pthread_mutex_lock(&ctx->lock);
    drm_hwc_layer_t *l = find_layer(ctx, layer);
    if (!l) {
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_BAD_LAYER;
    }
    l->in_use = false;
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_create_virtual_display(hwc2_device_t *dev, uint32_t w, uint32_t h,
                                           int32_t *fmt, hwc2_display_t *outDisp) {
    (void)dev; (void)w; (void)h; (void)fmt; (void)outDisp;
    return HWC2_ERROR_NO_RESOURCES;
}

static int32_t hwc2_destroy_virtual_display(hwc2_device_t *dev, hwc2_display_t display) {
    (void)dev; (void)display;
    return HWC2_ERROR_BAD_DISPLAY;
}

static void hwc2_dump(hwc2_device_t *dev, uint32_t *outSize, char *outBuffer) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    char msg[256];
    int n = snprintf(msg, sizeof(msg),
                     "SM-P613 Mainline DRM HWC2: %dx%d conn=%u crtc=%u plane=%u fb=%u\n",
                     ctx->disp_w, ctx->disp_h, ctx->conn_id, ctx->crtc_id,
                     ctx->plane_id, ctx->active_fb_id);
    if (!outBuffer) {
        *outSize = (uint32_t)n;
    } else {
        uint32_t copy = (*outSize < (uint32_t)n) ? *outSize : (uint32_t)n;
        memcpy(outBuffer, msg, copy);
        *outSize = copy;
    }
}

static uint32_t hwc2_get_max_virtual_display_count(hwc2_device_t *dev) {
    (void)dev;
    return 0;
}

static int32_t hwc2_get_active_config(hwc2_device_t *dev, hwc2_display_t display,
                                      hwc2_config_t *outConfig) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (!outConfig) return HWC2_ERROR_BAD_PARAMETER;
    *outConfig = PRIMARY_CONFIG_ID;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_changed_composition_types(hwc2_device_t *dev, hwc2_display_t display,
                                                  uint32_t *outNumElements,
                                                  hwc2_layer_t *outLayers,
                                                  int32_t *outTypes) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    pthread_mutex_lock(&ctx->lock);
    uint32_t count = 0;
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (ctx->layers[i].in_use && ctx->layers[i].changed) {
            if (outLayers && outTypes && count < *outNumElements) {
                outLayers[count] = ctx->layers[i].id;
                outTypes[count] = ctx->layers[i].pending_type;
            }
            count++;
        }
    }
    if (!outLayers || !outTypes)
        *outNumElements = count;
    else if (count < *outNumElements)
        *outNumElements = count;
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_client_target_support(hwc2_device_t *dev, hwc2_display_t display,
                                              uint32_t width, uint32_t height,
                                              int32_t format, int32_t dataspace) {
    (void)dev; (void)width; (void)height; (void)dataspace;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (format == HAL_PIXEL_FORMAT_RGBA_8888 ||
        format == HAL_PIXEL_FORMAT_RGBX_8888 ||
        format == HAL_PIXEL_FORMAT_BGRA_8888)
        return HWC2_ERROR_NONE;
    return HWC2_ERROR_UNSUPPORTED;
}

static int32_t hwc2_get_color_modes(hwc2_device_t *dev, hwc2_display_t display,
                                    uint32_t *outNumModes, int32_t *outModes) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (!outModes) {
        *outNumModes = 1;
    } else if (*outNumModes >= 1) {
        outModes[0] = 0; /* HAL_COLOR_MODE_NATIVE */
        *outNumModes = 1;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_attribute(hwc2_device_t *dev, hwc2_display_t display,
                                          hwc2_config_t config, int32_t attribute,
                                          int32_t *outValue) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (config != PRIMARY_CONFIG_ID) return HWC2_ERROR_BAD_CONFIG;
    if (!outValue) return HWC2_ERROR_BAD_PARAMETER;

    switch (attribute) {
        case HWC2_ATTRIBUTE_WIDTH:        *outValue = ctx->disp_w; break;
        case HWC2_ATTRIBUTE_HEIGHT:       *outValue = ctx->disp_h; break;
        case HWC2_ATTRIBUTE_VSYNC_PERIOD: *outValue = ctx->vsync_period_ns; break;
        case HWC2_ATTRIBUTE_DPI_X:        *outValue = ctx->dpi_x; break;
        case HWC2_ATTRIBUTE_DPI_Y:        *outValue = ctx->dpi_y; break;
        case HWC2_ATTRIBUTE_CONFIG_GROUP: *outValue = 0; break;
        default:                          return HWC2_ERROR_BAD_PARAMETER;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_configs(hwc2_device_t *dev, hwc2_display_t display,
                                        uint32_t *outNumConfigs, hwc2_config_t *outConfigs) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (!outConfigs) {
        *outNumConfigs = 1;
    } else if (*outNumConfigs >= 1) {
        outConfigs[0] = PRIMARY_CONFIG_ID;
        *outNumConfigs = 1;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_name(hwc2_device_t *dev, hwc2_display_t display,
                                     uint32_t *outSize, char *outName) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    const char *name = "SM-P613-DSI";
    uint32_t len = (uint32_t)strlen(name);
    if (!outName) {
        *outSize = len;
    } else {
        uint32_t copy = (*outSize < len) ? *outSize : len;
        memcpy(outName, name, copy);
        *outSize = copy;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_requests(hwc2_device_t *dev, hwc2_display_t display,
                                         int32_t *outDisplayRequests,
                                         uint32_t *outNumElements,
                                         hwc2_layer_t *outLayers,
                                         int32_t *outLayerRequests) {
    (void)dev; (void)outLayers; (void)outLayerRequests;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outDisplayRequests) *outDisplayRequests = 0;
    if (outNumElements) *outNumElements = 0;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_type(hwc2_device_t *dev, hwc2_display_t display,
                                     int32_t *outType) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    *outType = HWC2_DISPLAY_TYPE_PHYSICAL;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_doze_support(hwc2_device_t *dev, hwc2_display_t display,
                                     int32_t *outSupport) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    *outSupport = 0;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_hdr_capabilities(hwc2_device_t *dev, hwc2_display_t display,
                                         uint32_t *outNumTypes, int32_t *outTypes,
                                         float *outMaxLum, float *outMaxAvgLum,
                                         float *outMinLum) {
    (void)dev; (void)outTypes;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outNumTypes) *outNumTypes = 0;
    if (outMaxLum) *outMaxLum = 0.0f;
    if (outMaxAvgLum) *outMaxAvgLum = 0.0f;
    if (outMinLum) *outMinLum = 0.0f;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_release_fences(hwc2_device_t *dev, hwc2_display_t display,
                                       uint32_t *outNumElements,
                                       hwc2_layer_t *outLayers, int32_t *outFences) {
    (void)dev; (void)outLayers; (void)outFences;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outNumElements) *outNumElements = 0;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_present_display(hwc2_device_t *dev, hwc2_display_t display,
                                    int32_t *outPresentFence) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outPresentFence) *outPresentFence = -1;

    pthread_mutex_lock(&ctx->lock);
    if (!ctx->validated) {
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_NOT_VALIDATED;
    }

    int acquire_fence = ctx->client_acquire_fence;
    ctx->client_acquire_fence = -1;
    buffer_handle_t target = ctx->client_target;

    if (!target || ctx->power_mode == HWC2_POWER_MODE_OFF) {
        if (acquire_fence >= 0) close(acquire_fence);
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_NONE;
    }

    uint32_t buf_w = 0, buf_h = 0;
    uint32_t fb_id = import_buffer_to_fb(ctx, target, &buf_w, &buf_h);
    if (!fb_id) {
        if (acquire_fence >= 0) close(acquire_fence);
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_NO_RESOURCES;
    }

    drmModeAtomicReqPtr req = drmModeAtomicAlloc();
    if (!req) {
        if (acquire_fence >= 0) close(acquire_fence);
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_NO_RESOURCES;
    }

    int64_t out_fence = -1;
    uint32_t flags = 0;

    if (ctx->needs_modeset) {
        drmModeAtomicAddProperty(req, ctx->conn_id, ctx->prop_conn_crtc_id, ctx->crtc_id);
        drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_active, 1);
        drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_mode_id, ctx->mode_blob_id);
        flags |= DRM_MODE_ATOMIC_ALLOW_MODESET;
    }

    if (ctx->prop_crtc_out_fence_ptr) {
        drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_out_fence_ptr,
                                 (uint64_t)(uintptr_t)&out_fence);
    }

    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_fb_id, fb_id);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_crtc_id, ctx->crtc_id);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_src_x, 0);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_src_y, 0);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_src_w, (uint64_t)buf_w << 16);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_src_h, (uint64_t)buf_h << 16);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_crtc_x, 0);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_crtc_y, 0);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_crtc_w, (uint64_t)ctx->disp_w);
    drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_crtc_h, (uint64_t)ctx->disp_h);
    if (acquire_fence >= 0 && ctx->prop_plane_in_fence_fd) {
        drmModeAtomicAddProperty(req, ctx->plane_id, ctx->prop_plane_in_fence_fd,
                                 (uint64_t)acquire_fence);
    }

    int ret = drmModeAtomicCommit(ctx->drm_fd, req, flags, NULL);
    if (ret != 0 && !(flags & DRM_MODE_ATOMIC_ALLOW_MODESET)) {
        /* Retry once with ALLOW_MODESET in case CRTC state changed */
        drmModeAtomicAddProperty(req, ctx->conn_id, ctx->prop_conn_crtc_id, ctx->crtc_id);
        drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_active, 1);
        drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_mode_id, ctx->mode_blob_id);
        ret = drmModeAtomicCommit(ctx->drm_fd, req, flags | DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
    }
    drmModeAtomicFree(req);

    if (acquire_fence >= 0)
        close(acquire_fence);

    if (ret != 0) {
        ALOGE("drmModeAtomicCommit(fb=%u) failed: %s", fb_id, strerror(errno));
        if (out_fence >= 0) close((int)out_fence);
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_NO_RESOURCES;
    }

    ctx->needs_modeset = false;
    ctx->prev_fb_id = ctx->active_fb_id;
    ctx->active_fb_id = fb_id;

    if (outPresentFence)
        *outPresentFence = (out_fence >= 0) ? (int32_t)out_fence : -1;
    else if (out_fence >= 0)
        close((int)out_fence);

    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_register_callback(hwc2_device_t *dev, int32_t descriptor,
                                      hwc2_callback_data_t callbackData,
                                      hwc2_function_pointer_t pointer) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    bool fire_hotplug = false;

    pthread_mutex_lock(&ctx->lock);
    switch (descriptor) {
        case HWC2_CALLBACK_HOTPLUG:
            ctx->cb_hotplug = (HWC2_PFN_HOTPLUG)pointer;
            ctx->cb_hotplug_data = callbackData;
            fire_hotplug = (pointer != NULL);
            break;
        case HWC2_CALLBACK_REFRESH:
            ctx->cb_refresh = (HWC2_PFN_REFRESH)pointer;
            ctx->cb_refresh_data = callbackData;
            break;
        case HWC2_CALLBACK_VSYNC:
            ctx->cb_vsync = (HWC2_PFN_VSYNC)pointer;
            ctx->cb_vsync_data = callbackData;
            break;
        case HWC2_CALLBACK_VSYNC_2_4:
            ctx->cb_vsync_2_4 = (HWC2_PFN_VSYNC_2_4)pointer;
            ctx->cb_vsync_2_4_data = callbackData;
            break;
        case HWC2_CALLBACK_VSYNC_PERIOD_TIMING_CHANGED:
        case HWC2_CALLBACK_SEAMLESS_POSSIBLE:
            break;
        default:
            pthread_mutex_unlock(&ctx->lock);
            return HWC2_ERROR_BAD_PARAMETER;
    }
    pthread_mutex_unlock(&ctx->lock);

    if (fire_hotplug) {
        ALOGI("Firing initial hotplug CONNECTED for primary display 0");
        ((HWC2_PFN_HOTPLUG)pointer)(callbackData, PRIMARY_DISPLAY_ID, HWC2_CONNECTION_CONNECTED);
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_active_config(hwc2_device_t *dev, hwc2_display_t display,
                                      hwc2_config_t config) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (config != PRIMARY_CONFIG_ID) return HWC2_ERROR_BAD_CONFIG;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_client_target(hwc2_device_t *dev, hwc2_display_t display,
                                      buffer_handle_t target, int32_t acquireFence,
                                      int32_t dataspace, hwc_region_t damage) {
    (void)dataspace; (void)damage;
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) {
        if (acquireFence >= 0) close(acquireFence);
        return HWC2_ERROR_BAD_DISPLAY;
    }
    pthread_mutex_lock(&ctx->lock);
    if (ctx->client_acquire_fence >= 0)
        close(ctx->client_acquire_fence);
    ctx->client_target = target;
    ctx->client_acquire_fence = acquireFence;
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_color_mode(hwc2_device_t *dev, hwc2_display_t display, int32_t mode) {
    (void)dev; (void)mode;
    return (display == PRIMARY_DISPLAY_ID) ? HWC2_ERROR_NONE : HWC2_ERROR_BAD_DISPLAY;
}

static int32_t hwc2_set_color_mode_with_intent(hwc2_device_t *dev, hwc2_display_t display,
                                               int32_t mode, int32_t intent) {
    (void)dev; (void)mode; (void)intent;
    return (display == PRIMARY_DISPLAY_ID) ? HWC2_ERROR_NONE : HWC2_ERROR_BAD_DISPLAY;
}

static int32_t hwc2_set_color_transform(hwc2_device_t *dev, hwc2_display_t display,
                                        const float *matrix, int32_t hint) {
    (void)dev; (void)matrix; (void)hint;
    return (display == PRIMARY_DISPLAY_ID) ? HWC2_ERROR_NONE : HWC2_ERROR_BAD_DISPLAY;
}

static int32_t hwc2_set_output_buffer(hwc2_device_t *dev, hwc2_display_t display,
                                      buffer_handle_t buffer, int32_t releaseFence) {
    (void)dev; (void)display; (void)buffer;
    if (releaseFence >= 0) close(releaseFence);
    return HWC2_ERROR_UNSUPPORTED;
}

static int32_t hwc2_set_power_mode(hwc2_device_t *dev, hwc2_display_t display, int32_t mode) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (mode < HWC2_POWER_MODE_OFF || mode > HWC2_POWER_MODE_DOZE_SUSPEND)
        return HWC2_ERROR_BAD_PARAMETER;

    pthread_mutex_lock(&ctx->lock);
    int32_t prev = ctx->power_mode;
    ctx->power_mode = mode;

    if (mode == HWC2_POWER_MODE_OFF && prev != HWC2_POWER_MODE_OFF) {
        write_backlight(ctx, 0);
        drmModeAtomicReqPtr req = drmModeAtomicAlloc();
        if (req) {
            drmModeAtomicAddProperty(req, ctx->crtc_id, ctx->prop_crtc_active, 0);
            drmModeAtomicCommit(ctx->drm_fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, NULL);
            drmModeAtomicFree(req);
        }
        ctx->needs_modeset = true;
    } else if (mode != HWC2_POWER_MODE_OFF && prev == HWC2_POWER_MODE_OFF) {
        ctx->needs_modeset = true;
        write_backlight(ctx, ctx->bl_last_on);
    }
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_vsync_enabled(hwc2_device_t *dev, hwc2_display_t display, int32_t enabled) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (enabled != HWC2_VSYNC_ENABLE && enabled != HWC2_VSYNC_DISABLE)
        return HWC2_ERROR_BAD_PARAMETER;

    pthread_mutex_lock(&ctx->lock);
    ctx->vsync_enabled = (enabled == HWC2_VSYNC_ENABLE);
    pthread_cond_signal(&ctx->vsync_cond);
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_validate_display(hwc2_device_t *dev, hwc2_display_t display,
                                     uint32_t *outNumTypes, uint32_t *outNumRequests) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;

    pthread_mutex_lock(&ctx->lock);
    uint32_t changed = 0;
    for (int i = 0; i < MAX_LAYERS; i++) {
        if (ctx->layers[i].in_use) {
            if (ctx->layers[i].comp_type != HWC2_COMPOSITION_CLIENT) {
                ctx->layers[i].pending_type = HWC2_COMPOSITION_CLIENT;
                ctx->layers[i].changed = true;
                changed++;
            } else {
                ctx->layers[i].changed = false;
            }
        }
    }
    ctx->validated = true;
    pthread_mutex_unlock(&ctx->lock);

    if (outNumTypes) *outNumTypes = changed;
    if (outNumRequests) *outNumRequests = 0;
    return changed ? HWC2_ERROR_HAS_CHANGES : HWC2_ERROR_NONE;
}

/* Layer property setters (all layers are CLIENT-composited) */
static int32_t hwc2_set_layer_composition_type(hwc2_device_t *dev, hwc2_display_t display,
                                               hwc2_layer_t layer, int32_t type) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    pthread_mutex_lock(&ctx->lock);
    drm_hwc_layer_t *l = find_layer(ctx, layer);
    if (!l) {
        pthread_mutex_unlock(&ctx->lock);
        return HWC2_ERROR_BAD_LAYER;
    }
    l->comp_type = type;
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_layer_buffer(hwc2_device_t *dev, hwc2_display_t display,
                                     hwc2_layer_t layer, buffer_handle_t buffer,
                                     int32_t acquireFence) {
    (void)dev; (void)display; (void)layer; (void)buffer;
    if (acquireFence >= 0) close(acquireFence);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_layer_noop_i32(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, int32_t a) {
    (void)d; (void)disp; (void)l; (void)a; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_u32(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, uint32_t a) {
    (void)d; (void)disp; (void)l; (void)a; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_f32(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, float a) {
    (void)d; (void)disp; (void)l; (void)a; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_2i32(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, int32_t a, int32_t b) {
    (void)d; (void)disp; (void)l; (void)a; (void)b; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_rect(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, hwc_rect_t r) {
    (void)d; (void)disp; (void)l; (void)r; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_frect(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, hwc_frect_t r) {
    (void)d; (void)disp; (void)l; (void)r; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_region(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, hwc_region_t r) {
    (void)d; (void)disp; (void)l; (void)r; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_color(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, hwc_color_t c) {
    (void)d; (void)disp; (void)l; (void)c; return HWC2_ERROR_NONE;
}
static int32_t hwc2_set_layer_noop_ptr(hwc2_device_t *d, hwc2_display_t disp, hwc2_layer_t l, const void *p) {
    (void)d; (void)disp; (void)l; (void)p; return HWC2_ERROR_NONE;
}

/* HWC 2.2 / 2.3 / 2.4 optional functions */
static int32_t hwc2_get_render_intents(hwc2_device_t *dev, hwc2_display_t display,
                                       int32_t mode, uint32_t *outNumIntents, int32_t *outIntents) {
    (void)dev; (void)mode;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (!outIntents) {
        *outNumIntents = 1;
    } else if (*outNumIntents >= 1) {
        outIntents[0] = 0; /* COLORIMETRIC */
        *outNumIntents = 1;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_capabilities(hwc2_device_t *dev, hwc2_display_t display,
                                             uint32_t *outNumCaps, uint32_t *outCaps) {
    (void)dev; (void)outCaps;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outNumCaps) *outNumCaps = 0;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_brightness_support(hwc2_device_t *dev, hwc2_display_t display,
                                                   bool *outSupport) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outSupport) *outSupport = (ctx->bl_path[0] != '\0');
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_display_brightness(hwc2_device_t *dev, hwc2_display_t display,
                                           float brightness) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (brightness < 0.0f && brightness != -1.0f) return HWC2_ERROR_BAD_PARAMETER;
    if (brightness > 1.0f) return HWC2_ERROR_BAD_PARAMETER;

    pthread_mutex_lock(&ctx->lock);
    int val = (brightness < 0.0f) ? 0 : (int)lroundf(brightness * (float)ctx->bl_max);
    if (val > 0) ctx->bl_last_on = val;
    write_backlight(ctx, val);
    pthread_mutex_unlock(&ctx->lock);
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_connection_type(hwc2_device_t *dev, hwc2_display_t display,
                                                uint32_t *outType) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outType) *outType = 0; /* HWC2_DISPLAY_CONNECTION_TYPE_INTERNAL */
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_get_display_vsync_period(hwc2_device_t *dev, hwc2_display_t display,
                                             hwc2_vsync_period_t *outPeriod) {
    drm_hwc2_ctx_t *ctx = to_ctx(dev);
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outPeriod) *outPeriod = ctx->vsync_period_ns;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_active_config_with_constraints(
    hwc2_device_t *dev, hwc2_display_t display, hwc2_config_t config,
    hwc_vsync_period_change_constraints_t *constraints,
    hwc_vsync_period_change_timeline_t *outTimeline) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (config != PRIMARY_CONFIG_ID) return HWC2_ERROR_BAD_CONFIG;
    if (outTimeline) {
        outTimeline->newVsyncAppliedTimeNanos =
            constraints ? constraints->desiredTimeNanos : (int64_t)now_ns();
        outTimeline->refreshRequired = 0;
        outTimeline->refreshTimeNanos = 0;
    }
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_auto_low_latency_mode(hwc2_device_t *dev, hwc2_display_t display, bool on) {
    (void)dev; (void)display; (void)on;
    return HWC2_ERROR_UNSUPPORTED;
}

static int32_t hwc2_get_supported_content_types(hwc2_device_t *dev, hwc2_display_t display,
                                                uint32_t *outNum, uint32_t *outTypes) {
    (void)dev; (void)outTypes;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outNum) *outNum = 0;
    return HWC2_ERROR_NONE;
}

static int32_t hwc2_set_content_type(hwc2_device_t *dev, hwc2_display_t display, int32_t type) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    return (type == 0) ? HWC2_ERROR_NONE : HWC2_ERROR_UNSUPPORTED;
}

static int32_t hwc2_get_client_target_property(hwc2_device_t *dev, hwc2_display_t display,
                                               hwc_client_target_property_t *outProp) {
    (void)dev;
    if (display != PRIMARY_DISPLAY_ID) return HWC2_ERROR_BAD_DISPLAY;
    if (outProp) {
        outProp->pixelFormat = HAL_PIXEL_FORMAT_RGBA_8888;
        outProp->dataspace = 0; /* UNKNOWN */
    }
    return HWC2_ERROR_NONE;
}

static void hwc2_get_capabilities(hwc2_device_t *dev, uint32_t *outCount, int32_t *outCaps) {
    (void)dev; (void)outCaps;
    if (outCount) *outCount = 0;
}

static hwc2_function_pointer_t hwc2_get_function(hwc2_device_t *dev, int32_t descriptor) {
    (void)dev;
    switch (descriptor) {
        case HWC2_FUNCTION_ACCEPT_DISPLAY_CHANGES:    return (hwc2_function_pointer_t)hwc2_accept_display_changes;
        case HWC2_FUNCTION_CREATE_LAYER:              return (hwc2_function_pointer_t)hwc2_create_layer;
        case HWC2_FUNCTION_CREATE_VIRTUAL_DISPLAY:    return (hwc2_function_pointer_t)hwc2_create_virtual_display;
        case HWC2_FUNCTION_DESTROY_LAYER:             return (hwc2_function_pointer_t)hwc2_destroy_layer;
        case HWC2_FUNCTION_DESTROY_VIRTUAL_DISPLAY:   return (hwc2_function_pointer_t)hwc2_destroy_virtual_display;
        case HWC2_FUNCTION_DUMP:                      return (hwc2_function_pointer_t)hwc2_dump;
        case HWC2_FUNCTION_GET_ACTIVE_CONFIG:         return (hwc2_function_pointer_t)hwc2_get_active_config;
        case HWC2_FUNCTION_GET_CHANGED_COMPOSITION_TYPES: return (hwc2_function_pointer_t)hwc2_get_changed_composition_types;
        case HWC2_FUNCTION_GET_CLIENT_TARGET_SUPPORT: return (hwc2_function_pointer_t)hwc2_get_client_target_support;
        case HWC2_FUNCTION_GET_COLOR_MODES:           return (hwc2_function_pointer_t)hwc2_get_color_modes;
        case HWC2_FUNCTION_GET_DISPLAY_ATTRIBUTE:     return (hwc2_function_pointer_t)hwc2_get_display_attribute;
        case HWC2_FUNCTION_GET_DISPLAY_CONFIGS:       return (hwc2_function_pointer_t)hwc2_get_display_configs;
        case HWC2_FUNCTION_GET_DISPLAY_NAME:          return (hwc2_function_pointer_t)hwc2_get_display_name;
        case HWC2_FUNCTION_GET_DISPLAY_REQUESTS:      return (hwc2_function_pointer_t)hwc2_get_display_requests;
        case HWC2_FUNCTION_GET_DISPLAY_TYPE:          return (hwc2_function_pointer_t)hwc2_get_display_type;
        case HWC2_FUNCTION_GET_DOZE_SUPPORT:          return (hwc2_function_pointer_t)hwc2_get_doze_support;
        case HWC2_FUNCTION_GET_HDR_CAPABILITIES:      return (hwc2_function_pointer_t)hwc2_get_hdr_capabilities;
        case HWC2_FUNCTION_GET_MAX_VIRTUAL_DISPLAY_COUNT: return (hwc2_function_pointer_t)hwc2_get_max_virtual_display_count;
        case HWC2_FUNCTION_GET_RELEASE_FENCES:        return (hwc2_function_pointer_t)hwc2_get_release_fences;
        case HWC2_FUNCTION_PRESENT_DISPLAY:           return (hwc2_function_pointer_t)hwc2_present_display;
        case HWC2_FUNCTION_REGISTER_CALLBACK:         return (hwc2_function_pointer_t)hwc2_register_callback;
        case HWC2_FUNCTION_SET_ACTIVE_CONFIG:         return (hwc2_function_pointer_t)hwc2_set_active_config;
        case HWC2_FUNCTION_SET_CLIENT_TARGET:         return (hwc2_function_pointer_t)hwc2_set_client_target;
        case HWC2_FUNCTION_SET_COLOR_MODE:            return (hwc2_function_pointer_t)hwc2_set_color_mode;
        case HWC2_FUNCTION_SET_COLOR_TRANSFORM:       return (hwc2_function_pointer_t)hwc2_set_color_transform;
        case HWC2_FUNCTION_SET_CURSOR_POSITION:       return (hwc2_function_pointer_t)hwc2_set_layer_noop_2i32;
        case HWC2_FUNCTION_SET_LAYER_BLEND_MODE:      return (hwc2_function_pointer_t)hwc2_set_layer_noop_i32;
        case HWC2_FUNCTION_SET_LAYER_BUFFER:          return (hwc2_function_pointer_t)hwc2_set_layer_buffer;
        case HWC2_FUNCTION_SET_LAYER_COLOR:           return (hwc2_function_pointer_t)hwc2_set_layer_noop_color;
        case HWC2_FUNCTION_SET_LAYER_COMPOSITION_TYPE:return (hwc2_function_pointer_t)hwc2_set_layer_composition_type;
        case HWC2_FUNCTION_SET_LAYER_DATASPACE:       return (hwc2_function_pointer_t)hwc2_set_layer_noop_i32;
        case HWC2_FUNCTION_SET_LAYER_DISPLAY_FRAME:   return (hwc2_function_pointer_t)hwc2_set_layer_noop_rect;
        case HWC2_FUNCTION_SET_LAYER_PLANE_ALPHA:     return (hwc2_function_pointer_t)hwc2_set_layer_noop_f32;
        case HWC2_FUNCTION_SET_LAYER_SIDEBAND_STREAM: return (hwc2_function_pointer_t)hwc2_set_layer_noop_ptr;
        case HWC2_FUNCTION_SET_LAYER_SOURCE_CROP:     return (hwc2_function_pointer_t)hwc2_set_layer_noop_frect;
        case HWC2_FUNCTION_SET_LAYER_SURFACE_DAMAGE:  return (hwc2_function_pointer_t)hwc2_set_layer_noop_region;
        case HWC2_FUNCTION_SET_LAYER_TRANSFORM:       return (hwc2_function_pointer_t)hwc2_set_layer_noop_i32;
        case HWC2_FUNCTION_SET_LAYER_VISIBLE_REGION:  return (hwc2_function_pointer_t)hwc2_set_layer_noop_region;
        case HWC2_FUNCTION_SET_LAYER_Z_ORDER:         return (hwc2_function_pointer_t)hwc2_set_layer_noop_u32;
        case HWC2_FUNCTION_SET_OUTPUT_BUFFER:         return (hwc2_function_pointer_t)hwc2_set_output_buffer;
        case HWC2_FUNCTION_SET_POWER_MODE:            return (hwc2_function_pointer_t)hwc2_set_power_mode;
        case HWC2_FUNCTION_SET_VSYNC_ENABLED:         return (hwc2_function_pointer_t)hwc2_set_vsync_enabled;
        case HWC2_FUNCTION_VALIDATE_DISPLAY:          return (hwc2_function_pointer_t)hwc2_validate_display;
        case HWC2_FUNCTION_GET_RENDER_INTENTS:        return (hwc2_function_pointer_t)hwc2_get_render_intents;
        case HWC2_FUNCTION_SET_COLOR_MODE_WITH_RENDER_INTENT: return (hwc2_function_pointer_t)hwc2_set_color_mode_with_intent;
        case HWC2_FUNCTION_GET_DISPLAY_CAPABILITIES:  return (hwc2_function_pointer_t)hwc2_get_display_capabilities;
        case HWC2_FUNCTION_GET_DISPLAY_BRIGHTNESS_SUPPORT: return (hwc2_function_pointer_t)hwc2_get_display_brightness_support;
        case HWC2_FUNCTION_SET_DISPLAY_BRIGHTNESS:    return (hwc2_function_pointer_t)hwc2_set_display_brightness;
        case HWC2_FUNCTION_GET_DISPLAY_CONNECTION_TYPE: return (hwc2_function_pointer_t)hwc2_get_display_connection_type;
        case HWC2_FUNCTION_GET_DISPLAY_VSYNC_PERIOD:  return (hwc2_function_pointer_t)hwc2_get_display_vsync_period;
        case HWC2_FUNCTION_SET_ACTIVE_CONFIG_WITH_CONSTRAINTS: return (hwc2_function_pointer_t)hwc2_set_active_config_with_constraints;
        case HWC2_FUNCTION_SET_AUTO_LOW_LATENCY_MODE: return (hwc2_function_pointer_t)hwc2_set_auto_low_latency_mode;
        case HWC2_FUNCTION_GET_SUPPORTED_CONTENT_TYPES: return (hwc2_function_pointer_t)hwc2_get_supported_content_types;
        case HWC2_FUNCTION_SET_CONTENT_TYPE:          return (hwc2_function_pointer_t)hwc2_set_content_type;
        case HWC2_FUNCTION_GET_CLIENT_TARGET_PROPERTY:return (hwc2_function_pointer_t)hwc2_get_client_target_property;
        default:                                      return NULL;
    }
}

static int hwc2_close(hw_device_t *device) {
    drm_hwc2_ctx_t *ctx = (drm_hwc2_ctx_t *)device;
    if (!ctx) return 0;

    pthread_mutex_lock(&ctx->lock);
    ctx->vsync_thread_running = false;
    ctx->vsync_enabled = false;
    pthread_cond_signal(&ctx->vsync_cond);
    pthread_mutex_unlock(&ctx->lock);

    pthread_join(ctx->vsync_thread, NULL);

    if (ctx->client_acquire_fence >= 0)
        close(ctx->client_acquire_fence);
    fb_cache_clear(ctx);
    if (ctx->mode_blob_id)
        drmModeDestroyPropertyBlob(ctx->drm_fd, ctx->mode_blob_id);
    if (ctx->drm_fd >= 0)
        close(ctx->drm_fd);

    pthread_cond_destroy(&ctx->vsync_cond);
    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
    return 0;
}

static int hwc2_open(const hw_module_t *module, const char *id, hw_device_t **device) {
    if (!id || strcmp(id, HWC_HARDWARE_COMPOSER) != 0)
        return -EINVAL;

    drm_hwc2_ctx_t *ctx = (drm_hwc2_ctx_t *)calloc(1, sizeof(*ctx));
    if (!ctx) return -ENOMEM;

    ctx->drm_fd = -1;
    ctx->client_acquire_fence = -1;
    ctx->needs_modeset = true;
    ctx->power_mode = HWC2_POWER_MODE_ON;
    pthread_mutex_init(&ctx->lock, NULL);
    pthread_cond_init(&ctx->vsync_cond, NULL);

    init_backlight(ctx);
    int ret = drm_init_kms(ctx);
    if (ret != 0) {
        pthread_cond_destroy(&ctx->vsync_cond);
        pthread_mutex_destroy(&ctx->lock);
        free(ctx);
        return ret;
    }

    ctx->vsync_thread_running = true;
    pthread_create(&ctx->vsync_thread, NULL, vsync_thread_fn, ctx);

    ctx->dev.common.tag = HARDWARE_DEVICE_TAG;
    ctx->dev.common.version = HWC_DEVICE_API_VERSION_2_0;
    ctx->dev.common.module = (hw_module_t *)module;
    ctx->dev.common.close = hwc2_close;
    ctx->dev.getCapabilities = hwc2_get_capabilities;
    ctx->dev.getFunction = hwc2_get_function;

    *device = &ctx->dev.common;
    ALOGI("hwcomposer.drm.so opened successfully");
    return 0;
}

static hw_module_methods_t g_hwc_methods = {
    .open = hwc2_open,
};

__attribute__((visibility("default")))
hw_module_t HAL_MODULE_INFO_SYM = {
    .tag = HARDWARE_MODULE_TAG,
    .module_api_version = HWC_MODULE_API_VERSION_0_1,
    .hal_api_version = 0,
    .id = HWC_HARDWARE_MODULE_ID,
    .name = "SM-P613 Mainline DRM/KMS HWC2",
    .author = "p613_kernel_port",
    .methods = &g_hwc_methods,
};
