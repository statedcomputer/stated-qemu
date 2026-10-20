/*
 * Spacebox remote GPU: the pictures of the user's toggle (the task of
 * 2026-10-20, one GPU spec).
 *
 * With the toggle off (the sink's hotkey or its control file, SP_GPU) the
 * Firecracker device runs the guest on the renderer inside the guest, which is
 * this sink built for Linux, and asks it for pictures (SP_STREAM). That
 * renderer then reads its scanout back after each frame it shows (ui/sdl2-gl.c,
 * ui/spacebox-stream.h: at most 30 a second, by a token bucket that takes the
 * last frame of a burst), encodes it as JPEG (libjpeg-turbo, quality 80, 4:2:0)
 * on a thread of its own (at a lower priority than the guest's programs), the
 * guest's CPU, and sends it to the device on its
 * link (SP_PICTURE, not resent). The device passes each one to the user's
 * renderer, which decodes it and shows it in its window as a 2D surface; when
 * the toggle is on again the device's switch back sets the virgl scanout again.
 *
 * SPACEBOX_STREAM_SCALE=2 (the renderer inside the guest): the pictures at half
 * the scanout's size (a 2x2 box filter); the user's renderer decodes them at
 * the scanout's size (libjpeg's scaling).
 *
 * Every second with pictures, a line [SPACEBOX-STREAM] (the renderer inside
 * the guest) or [SPACEBOX-PICTURE] (the user's renderer) gives the counts.
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "ui/console.h"
#include "virtio-gpu-remote.h"

#include <setjmp.h>
#include <jpeglib.h>
#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

#define SP_PIC_QUALITY 80
/* the device drops a picture while its link to the user has more than 1 MiB
 * waiting; this end does the same with its own link to the device */
#define SP_PIC_BACKLOG (1u << 20)
#define SP_PIC_QUEUE 3
/* the encoder's niceness (Linux): it takes the CPU that the guest's own programs
 * leave, so that its work does not delay their input and drawing */
#define SP_PIC_NICE 10

/* ---- libjpeg's errors: back to the caller, not exit() --------------------- */

typedef struct SpJerr {
    struct jpeg_error_mgr pub;
    jmp_buf jb;
} SpJerr;

static void sp_jerr_exit(j_common_ptr c)
{
    longjmp(((SpJerr *)c->err)->jb, 1);
}

static void sp_jerr_quiet(j_common_ptr c)
{
}

/* ---- the renderer inside the guest: the stream ---------------------------- */

static struct {
    bool on;            /* the source asked for pictures (SP_STREAM) */
    int scale;          /* 1, or 2: half the scanout's size */
    QemuMutex lock;
    QemuCond cond;
    bool worker;
    QEMUBH *bh;
    /* the newest picture read back, waiting for the encoder (it replaces one
     * that waits: counted as coalesced) */
    uint8_t *px;
    int w, h, stride;
    bool bottom_up;
    /* the JPEGs made and not sent yet, oldest first: the main loop sends them all
     * when it runs (it is busy with the guest's drawing between); a JPEG that
     * finds the queue full replaces its oldest (counted as coalesced) */
    struct {
        uint8_t *data;
        unsigned long len;
        int w, h;
    } q[SP_PIC_QUEUE];
    int qn;
    int jpg_w, jpg_h, out_w, out_h;
    uint64_t n;         /* pictures sent since the start (arg2) */
    /* counts of the second */
    int64_t stats_at;
    uint64_t taken, coalesced, encoded, sent, dropped, bytes, enc_us;
} sps;

bool spacebox_stream_on(void)
{
    return sps.on;
}

/* 2x2 box filter of 4-byte pixels; w and h even */
static void sp_stream_half(const uint8_t *in, int w, int h, int stride,
                           uint8_t *out)
{
    int ow = w / 2;

    for (int y = 0; y < h / 2; y++) {
        const uint8_t *a = in + (size_t)(2 * y) * stride, *b = a + stride;
        uint8_t *o = out + (size_t)y * ow * 4;

        for (int x = 0; x < ow; x++, a += 8, b += 8, o += 4) {
            o[0] = (a[0] + a[4] + b[0] + b[4] + 2) >> 2;
            o[1] = (a[1] + a[5] + b[1] + b[5] + 2) >> 2;
            o[2] = (a[2] + a[6] + b[2] + b[6] + 2) >> 2;
            o[3] = 255;
        }
    }
}

/* One picture of RGBX pixels as JPEG; NULL if libjpeg failed. */
static uint8_t *sp_stream_encode(const uint8_t *px, int w, int h, int stride,
                                 bool bottom_up, unsigned long *len)
{
    struct jpeg_compress_struct c;
    SpJerr err;
    unsigned char *out = NULL;
    unsigned long size = 0;
    g_autofree JSAMPROW *rows = g_new(JSAMPROW, h);

    c.err = jpeg_std_error(&err.pub);
    err.pub.error_exit = sp_jerr_exit;
    err.pub.output_message = sp_jerr_quiet;
    if (setjmp(err.jb)) {
        /* (what libjpeg's memory destination had of the picture is lost; it
         * fails only on a wrong call) */
        jpeg_destroy_compress(&c);
        return NULL;
    }
    jpeg_create_compress(&c);
    jpeg_mem_dest(&c, &out, &size);
    c.image_width = w;
    c.image_height = h;
    c.input_components = 4;
    c.in_color_space = JCS_EXT_RGBX;
    jpeg_set_defaults(&c);   /* YCbCr 4:2:0 */
    jpeg_set_quality(&c, SP_PIC_QUALITY, TRUE);
    for (int y = 0; y < h; y++) {
        rows[y] = (JSAMPROW)(px + (size_t)(bottom_up ? h - 1 - y : y) * stride);
    }
    jpeg_start_compress(&c, TRUE);
    jpeg_write_scanlines(&c, rows, h);
    jpeg_finish_compress(&c);
    jpeg_destroy_compress(&c);
    *len = size;
    return out;
}

static void *sp_stream_worker(void *opaque)
{
#ifdef __linux__
    if (setpriority(PRIO_PROCESS, syscall(SYS_gettid), SP_PIC_NICE)) {
        fprintf(stderr, "[SPACEBOX-STREAM] the encoder's niceness not set: %s\n",
                strerror(errno));
    }
#endif
    for (;;) {
        uint8_t *px, *half = NULL, *jpg;
        int w, h, stride, ow, oh;
        unsigned long len = 0;
        bool bottom_up;
        int64_t t0;

        qemu_mutex_lock(&sps.lock);
        while (!sps.px) {
            qemu_cond_wait(&sps.cond, &sps.lock);
        }
        px = sps.px;
        sps.px = NULL;
        w = sps.w;
        h = sps.h;
        stride = sps.stride;
        bottom_up = sps.bottom_up;
        qemu_mutex_unlock(&sps.lock);

        t0 = g_get_monotonic_time();
        ow = w;
        oh = h;
        if (sps.scale == 2 && !(w & 1) && !(h & 1)) {
            ow = w / 2;
            oh = h / 2;
            half = g_malloc((size_t)ow * oh * 4);
            sp_stream_half(px, w, h, stride, half);
        }
        jpg = half ? sp_stream_encode(half, ow, oh, ow * 4, bottom_up, &len)
                   : sp_stream_encode(px, w, h, stride, bottom_up, &len);
        g_free(half);
        g_free(px);

        qemu_mutex_lock(&sps.lock);
        sps.enc_us += g_get_monotonic_time() - t0;
        if (jpg) {
            sps.encoded++;
            if (sps.qn == SP_PIC_QUEUE) {
                free(sps.q[0].data);
                memmove(&sps.q[0], &sps.q[1], sizeof(sps.q[0]) * (SP_PIC_QUEUE - 1));
                sps.qn--;
                sps.coalesced++;
            }
            sps.q[sps.qn].data = jpg;
            sps.q[sps.qn].len = len;
            sps.q[sps.qn].w = w;
            sps.q[sps.qn].h = h;
            sps.qn++;
            sps.jpg_w = w;
            sps.jpg_h = h;
            sps.out_w = ow;
            sps.out_h = oh;
        }
        qemu_mutex_unlock(&sps.lock);
        if (jpg) {
            qemu_bh_schedule(sps.bh);
        }
    }
    return NULL;
}

static void sp_stream_stats(void)
{
    int64_t now = g_get_monotonic_time();

    if (now - sps.stats_at < 1000000) {
        return;
    }
    if (sps.taken || sps.sent || sps.dropped) {
        fprintf(stderr, "[SPACEBOX-STREAM] us=%" PRId64 " secs=%.3f taken=%" PRIu64
                " coalesced=%" PRIu64 " encoded=%" PRIu64 " sent=%" PRIu64
                " dropped=%" PRIu64 " bytes=%" PRIu64 " enc_ms=%.1f size=%dx%d"
                " out=%dx%d\n", g_get_real_time(),
                (now - sps.stats_at) / 1e6, sps.taken, sps.coalesced,
                sps.encoded, sps.sent, sps.dropped, sps.bytes,
                sps.enc_us / 1000.0, sps.jpg_w, sps.jpg_h, sps.out_w, sps.out_h);
    }
    sps.stats_at = now;
    sps.taken = sps.coalesced = sps.encoded = sps.sent = sps.dropped = 0;
    sps.bytes = sps.enc_us = 0;
}

/* The main loop: send the JPEGs made to the source, in order. */
static void sp_stream_bh(void *opaque)
{
    for (;;) {
        uint8_t *jpg;
        unsigned long len;
        int w, h;

        qemu_mutex_lock(&sps.lock);
        if (!sps.qn) {
            sp_stream_stats();
            qemu_mutex_unlock(&sps.lock);
            return;
        }
        jpg = sps.q[0].data;
        len = sps.q[0].len;
        w = sps.q[0].w;
        h = sps.q[0].h;
        memmove(&sps.q[0], &sps.q[1], sizeof(sps.q[0]) * (SP_PIC_QUEUE - 1));
        sps.qn--;
        qemu_mutex_unlock(&sps.lock);
        if (sps.on && sp_sink_backlog() <= SP_PIC_BACKLOG) {
            sp_sink_send_once(SP_PICTURE, jpg, len, w, h, ++sps.n);
            qemu_mutex_lock(&sps.lock);
            sps.sent++;
            sps.bytes += len;
            qemu_mutex_unlock(&sps.lock);
        } else {
            qemu_mutex_lock(&sps.lock);
            sps.dropped++;
            qemu_mutex_unlock(&sps.lock);
        }
        free(jpg);
    }
}

/* SP_STREAM, and at every pairing (none then). */
void sp_picture_stream(bool on)
{
    if (!sps.bh) {
        const char *s = getenv("SPACEBOX_STREAM_SCALE");

        sps.scale = s && !strcmp(s, "2") ? 2 : 1;
        qemu_mutex_init(&sps.lock);
        qemu_cond_init(&sps.cond);
        sps.bh = qemu_bh_new(sp_stream_bh, NULL);
    }
    if (on != sps.on) {
        fprintf(stderr, "[SPACEBOX-STREAM] %s (scale %d)\n",
                on ? "the source asks for pictures" : "no pictures", sps.scale);
    }
    sps.on = on;
}

/* ui: a picture of the scanout read back (RGBX, stride bytes a row); taken
 * over, freed here. */
void spacebox_stream_take(uint8_t *px, int w, int h, int stride, bool bottom_up)
{
    if (!sps.on) {
        g_free(px);
        return;
    }
    qemu_mutex_lock(&sps.lock);
    if (!sps.worker) {
        QemuThread th;

        qemu_thread_create(&th, "sp-jpeg", sp_stream_worker, NULL,
                           QEMU_THREAD_DETACHED);
        sps.worker = true;
        sps.stats_at = g_get_monotonic_time();
    }
    if (sps.px) {
        g_free(sps.px);
        sps.coalesced++;
    }
    sps.px = px;
    sps.w = w;
    sps.h = h;
    sps.stride = stride;
    sps.bottom_up = bottom_up;
    sps.taken++;
    qemu_cond_signal(&sps.cond);
    qemu_mutex_unlock(&sps.lock);
}

/* ---- the user's renderer: the pictures shown ------------------------------ */

static struct {
    DisplaySurface *ds;  /* the console's surface while it is ours */
    uint64_t last_n;
    int64_t stats_at;
    uint64_t received, shown, late, lost, failed, bytes, dec_us;
} spp;

/* SP_PICTURE from the source: arg0, arg1 the scanout's size, arg2 the
 * renderer's count. Shown while the toggle is off. */
void sp_picture_show(VirtIOGPU *g, bool off, uint64_t sw, uint64_t sh,
                     uint64_t n, const uint8_t *a, size_t len)
{
    QemuConsole *con = g->parent_obj.scanout[0].con;
    struct jpeg_decompress_struct d;
    SpJerr err;
    DisplaySurface *volatile ds = NULL;
    volatile bool fresh = false;
    int64_t t0 = g_get_monotonic_time();
    unsigned num = 8;

    spp.received++;
    spp.bytes += len;
    if (spp.last_n && n > spp.last_n + 1) {
        spp.lost += n - spp.last_n - 1;
    }
    spp.last_n = n;
    if (!off || !con) {
        spp.late++;
        goto stats;
    }
    d.err = jpeg_std_error(&err.pub);
    err.pub.error_exit = sp_jerr_exit;
    err.pub.output_message = sp_jerr_quiet;
    if (setjmp(err.jb)) {
        jpeg_destroy_decompress(&d);
        if (fresh) {
            qemu_free_displaysurface(ds);
        }
        spp.failed++;
        goto stats;
    }
    jpeg_create_decompress(&d);
    jpeg_mem_src(&d, a, len);
    jpeg_read_header(&d, TRUE);
    /* a picture smaller than the scanout is decoded at the scanout's size */
    if (d.image_width && sw > d.image_width && sw * 8 % d.image_width == 0 &&
        sw * 8 / d.image_width <= 16) {
        num = sw * 8 / d.image_width;
    }
    d.scale_num = num;
    d.scale_denom = 8;
    d.out_color_space = JCS_EXT_BGRX;  /* PIXMAN_x8r8g8b8 in memory */
    jpeg_start_decompress(&d);
    if (spp.ds && qemu_console_surface(con) == spp.ds &&
        surface_width(spp.ds) == (int)d.output_width &&
        surface_height(spp.ds) == (int)d.output_height) {
        ds = spp.ds;
    } else {
        ds = qemu_create_displaysurface(d.output_width, d.output_height);
        fresh = true;
    }
    while (d.output_scanline < d.output_height) {
        JSAMPROW row = (JSAMPROW)((uint8_t *)surface_data(ds) +
                                  (size_t)d.output_scanline * surface_stride(ds));

        jpeg_read_scanlines(&d, &row, 1);
    }
    jpeg_finish_decompress(&d);
    jpeg_destroy_decompress(&d);
    if (fresh) {
        /* the console owns it from now on (a reset or a scanout replaces it) */
        dpy_gfx_replace_surface(con, ds);
        spp.ds = ds;
    }
    dpy_gfx_update_full(con);
    spp.shown++;
    spp.dec_us += g_get_monotonic_time() - t0;
stats:
    t0 = g_get_monotonic_time();
    if (!spp.stats_at) {
        spp.stats_at = t0;
    } else if (t0 - spp.stats_at >= 1000000) {
        fprintf(stderr, "[SPACEBOX-PICTURE] us=%" PRId64 " secs=%.3f received=%"
                PRIu64 " shown=%" PRIu64 " late=%" PRIu64 " lost=%" PRIu64
                " failed=%" PRIu64 " bytes=%" PRIu64 " decode_ms=%.1f size=%"
                PRIu64 "x%" PRIu64 " shown_size=%dx%d\n", g_get_real_time(),
                (t0 - spp.stats_at) / 1e6, spp.received, spp.shown, spp.late,
                spp.lost, spp.failed, spp.bytes, spp.dec_us / 1000.0, sw, sh,
                spp.ds ? surface_width(spp.ds) : 0,
                spp.ds ? surface_height(spp.ds) : 0);
        spp.stats_at = t0;
        spp.received = spp.shown = spp.late = spp.lost = spp.failed = 0;
        spp.bytes = spp.dec_us = 0;
    }
}

/* The toggle is on again, or a new pairing: the pictures end. The surface is
 * the console's; the switch back replaces it. */
void sp_picture_end(void)
{
    spp.ds = NULL;
    spp.last_n = 0;
    spp.stats_at = 0;
}
