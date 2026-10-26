/*
 * Spacebox remote GPU, sink: the record of video frames.
 *
 * SPACEBOX_VIDEO_RECORD=<file>   write the record into the file from the start;
 *                                the control file's "record-start FILE" and
 *                                "record-stop" start and stop it while the sink
 *                                runs. With no record nothing of it runs but a
 *                                check of spacebox_video_record_on here, and in
 *                                virglrenderer a ticket stored into the textures
 *                                of each decoded picture (below).
 *
 * JSON lines, one each (screen/README.md of the spacebox repository, "The
 * sink's record of video frames"):
 *   swap  each swap of the Mac's presenter (ui/spacebox-presenter.h): its
 *         return, the refresh it is shown at (from the display link's times),
 *         the frame code read in the picture swapped, the tickets of the video
 *         pictures it shows, the frames the presenter dropped since the last
 *         swap
 *   pic   each video picture: a decoded one, or an upload of a picture that
 *         the guest decoded itself into a texture of the video's size
 *         (video-size); its stage times: the source's time of the command
 *         that decoded or uploaded it (mapped to this clock as the lateness
 *         of the messages is), when this sink's reader read that command off
 *         the link and when its main loop ran it, the decoder's start and
 *         end, the hand-over into its textures, the first draw that read it,
 *         the first swap that showed it. Written at that swap, or 10 s after
 *         the picture with no swap.
 *   ctl   the record's start (with the sink's build, sp_rec_build()) and
 *         stop, the controls, the test pause
 * Every time is CLOCK_MONOTONIC in microseconds (this clock is
 * g_get_monotonic_time(), which on macOS does not count sleep: the
 * difference is taken at the record's start). On Windows CLOCK_MONOTONIC is
 * the QueryPerformanceCounter's (as winpthreads' clock_gettime, which
 * virglrenderer reads there), and each swap line has the counter's own
 * values at the swap's call and return (qpc_call, qpc_ret; qpc_hz in the
 * record's start), the clock of PresentMon's events.
 *
 * Tickets come from virglrenderer (virgl_renderer_video_record): a picture's
 * ticket follows it into what the draws that read it write, so the scanout
 * that a RESOURCE_FLUSH shows holds the tickets of its pictures; the
 * presenter carries them with the frame to its swap. The pictures get their
 * tickets also with no record (virglrenderer stores one into their textures),
 * so those that a player decoded before the record began (ahead, while it
 * was paused) have theirs when the record shows them; they have no pic line.
 *
 * The controls (the control file, virtio-gpu-remote-state.c):
 *   video-rect X Y W H | off   where the presenter reads the frame code, in
 *                              the scanout's pixels (the video's rectangle)
 *   video-size W H | off       the video's own size: uploads into textures of
 *                              that size are pictures
 *   pause-present MS           the presenter shows nothing for MS ms after its
 *                              next swap, once (a stall made on purpose)
 *   record-start FILE, record-stop
 *
 * The lines go to the file through a thread of their own, so neither the main
 * loop nor the presenter waits for the disk.
 */

#include "qemu/osdep.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "hw/virtio/virtio-gpu.h"
#include "ui/spacebox-video-record.h"
#include "virtio-gpu-remote.h"

#include <virglrenderer.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

/* a picture whose ticket no swap showed this long after it is written alone */
#define SP_REC_UNSHOWN_US (10 * G_USEC_PER_SEC)

bool spacebox_video_record_on;
uint64_t spacebox_frame_tickets[SP_REC_TICKETS];
unsigned spacebox_frame_ntickets;
int64_t spacebox_frame_flush_dev, spacebox_frame_flush_read, spacebox_frame_flush_rx;
int spacebox_present_pause_ms;

typedef struct SpRecPic {
    uint64_t ticket;
    uint32_t codec;         /* 0: an upload */
    const char *fmt;
    uint32_t w, h;
    int64_t t_dev, t_read, t_rx;    /* this clock (g_get_monotonic_time()), 0 = none */
    int64_t t_dec0, t_dec1, t_tex, t_draw;  /* CLOCK_MONOTONIC, 0 = none */
    int64_t t_draw_dev, t_draw_read;    /* the first draw's message: its source's
                                           time and its read (this clock) */
    int64_t made;           /* g_get_monotonic_time() of its ticket */
} SpRecPic;

static char sp_rec_stop_line;   /* to the writer: end */

static struct {
    QemuMutex lock;         /* pics, the counters, the rect */
    bool ready;
    char *path;
    FILE *f;
    GAsyncQueue *lines;
    QemuThread writer;
    int64_t off;            /* CLOCK_MONOTONIC - g_get_monotonic_time() */
    GHashTable *pics;       /* ticket -> SpRecPic */
    QEMUTimer *timer;
    int64_t cmd_dev, cmd_read, cmd_rx;  /* the command that runs */
    bool rect;
    int rx, ry, rw, rh;
    uint32_t vw, vh;
    /* what the record costs: the presenter's read of the code, the main
     * loop's time in the record */
    uint64_t swaps, n_pics, unshown, reads;
    int64_t read_us, read_max, main_us;
} rec;

#ifdef _WIN32
/* as winpthreads' clock_gettime(CLOCK_MONOTONIC), to the microsecond */
static int64_t sp_rec_mono(void)
{
    LARGE_INTEGER f, c;
    int64_t ns;

    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    ns = ((c.QuadPart % f.QuadPart) * 1000000000 + (f.QuadPart >> 1)) / f.QuadPart;
    return (int64_t)(c.QuadPart / f.QuadPart) * 1000000 + ns / 1000;
}
#else
static int64_t sp_rec_mono(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}
#endif

/* a time of this clock as CLOCK_MONOTONIC, or null */
static void sp_rec_time(GString *s, const char *key, int64_t g)
{
    if (g) {
        g_string_append_printf(s, ",\"%s\":%" PRId64, key, g + rec.off);
    } else {
        g_string_append_printf(s, ",\"%s\":null", key);
    }
}

static void sp_rec_mtime(GString *s, const char *key, int64_t m)
{
    if (m) {
        g_string_append_printf(s, ",\"%s\":%" PRId64, key, m);
    } else {
        g_string_append_printf(s, ",\"%s\":null", key);
    }
}

static void sp_rec_put(GString *s)
{
    g_string_append_c(s, '\n');
    g_async_queue_push(rec.lines, g_string_free(s, false));
}

static void *sp_rec_writer(void *opaque)
{
    FILE *f = opaque;

    for (;;) {
        char *line = g_async_queue_pop(rec.lines);

        if (line == &sp_rec_stop_line) {
            break;
        }
        fputs(line, f);
        g_free(line);
        if (!g_async_queue_length(rec.lines)) {
            fflush(f);
        }
    }
    fclose(f);
    return NULL;
}

/* under rec.lock */
static void sp_rec_pic_line(SpRecPic *p, int64_t t_swap)
{
    GString *s = g_string_new("{\"k\":\"pic\"");

    if (p->codec) {
        g_string_append_printf(s, ",\"codec\":%u", p->codec);
    } else {
        g_string_append(s, ",\"codec\":\"upload\"");
    }
    g_string_append_printf(s, ",\"ticket\":%" PRIu64 ",\"fmt\":\"%s\",\"w\":%u,\"h\":%u",
                           p->ticket, p->fmt, p->w, p->h);
    sp_rec_time(s, "t_dev", p->t_dev);
    sp_rec_time(s, "t_rx", p->t_rx);
    sp_rec_time(s, "t_read", p->t_read);
    sp_rec_mtime(s, "t_dec0", p->t_dec0);
    sp_rec_mtime(s, "t_dec1", p->t_dec1);
    sp_rec_mtime(s, "t_tex", p->t_tex);
    sp_rec_mtime(s, "t_draw", p->t_draw);
    sp_rec_time(s, "t_draw_dev", p->t_draw_dev);
    sp_rec_time(s, "t_draw_read", p->t_draw_read);
    sp_rec_time(s, "t_swap", t_swap);
    g_string_append_c(s, '}');
    sp_rec_put(s);
    rec.n_pics++;
    if (!t_swap) {
        rec.unshown++;
    }
}

static void sp_rec_ctl(const char *what, const char *more)
{
    GString *s = g_string_new("{\"k\":\"ctl\"");

    sp_rec_time(s, "t_us", g_get_monotonic_time());
    g_string_append_printf(s, ",\"what\":\"%s\"%s}", what, more ? more : "");
    sp_rec_put(s);
}

/* ---- virglrenderer's callbacks, on the main loop --------------------------- */

static void sp_rec_picture(void *opaque, uint64_t ticket, uint32_t codec,
                           const char *fmt)
{
    int64_t t0 = g_get_monotonic_time();
    SpRecPic *p = g_new0(SpRecPic, 1);

    p->ticket = ticket;
    p->codec = codec;
    p->fmt = fmt;
    p->t_dev = rec.cmd_dev;
    p->t_read = rec.cmd_read;
    p->t_rx = rec.cmd_rx;
    p->made = t0;
    qemu_mutex_lock(&rec.lock);
    g_hash_table_insert(rec.pics, &p->ticket, p);
    qemu_mutex_unlock(&rec.lock);
    rec.main_us += g_get_monotonic_time() - t0;
}

static void sp_rec_decoded(void *opaque, uint64_t ticket, uint32_t width,
                           uint32_t height, int64_t t_dec0, int64_t t_dec1,
                           int64_t t_tex)
{
    int64_t t0 = g_get_monotonic_time();
    SpRecPic *p;

    qemu_mutex_lock(&rec.lock);
    p = g_hash_table_lookup(rec.pics, &ticket);
    if (p) {
        p->w = width;
        p->h = height;
        p->t_dec0 = t_dec0;
        p->t_dec1 = t_dec1;
        p->t_tex = t_tex;
    }
    qemu_mutex_unlock(&rec.lock);
    rec.main_us += g_get_monotonic_time() - t0;
}

static void sp_rec_drawn(void *opaque, uint64_t ticket, int64_t t)
{
    int64_t t0 = g_get_monotonic_time();
    SpRecPic *p;

    qemu_mutex_lock(&rec.lock);
    p = g_hash_table_lookup(rec.pics, &ticket);
    if (p && !p->t_draw) {
        p->t_draw = t;
        p->t_draw_dev = rec.cmd_dev;
        p->t_draw_read = rec.cmd_read;
    }
    qemu_mutex_unlock(&rec.lock);
    rec.main_us += g_get_monotonic_time() - t0;
}

static const struct virgl_renderer_video_record sp_rec_cbs = {
    .picture = sp_rec_picture,
    .decoded = sp_rec_decoded,
    .drawn = sp_rec_drawn,
};

/* ---- the sink's hooks ----------------------------------------------------- */

void sp_record_cmd(struct virtio_gpu_ctrl_command *cmd)
{
    rec.cmd_dev = cmd->sp_t_dev;
    rec.cmd_read = cmd->sp_t_read;
    rec.cmd_rx = cmd->sp_t_rx;
}

void sp_record_flush(uint32_t resource_id, bool begin)
{
    if (!begin) {
        spacebox_frame_ntickets = 0;
        spacebox_frame_flush_dev = spacebox_frame_flush_read = spacebox_frame_flush_rx = 0;
    } else if (spacebox_video_record_on) {
        spacebox_frame_flush_dev = rec.cmd_dev;
        spacebox_frame_flush_read = rec.cmd_read;
        spacebox_frame_flush_rx = rec.cmd_rx;
        int64_t t0 = g_get_monotonic_time();

        spacebox_frame_ntickets = virgl_renderer_resource_video_tickets(resource_id,
                                    spacebox_frame_tickets, SP_REC_TICKETS);
        rec.main_us += g_get_monotonic_time() - t0;
    }
}

/* the pictures no swap showed, once they are old (all of them: at the stop) */
static void sp_rec_unshown(bool all)
{
    int64_t now = g_get_monotonic_time();
    GHashTableIter it;
    gpointer value;

    qemu_mutex_lock(&rec.lock);
    g_hash_table_iter_init(&it, rec.pics);
    while (g_hash_table_iter_next(&it, NULL, &value)) {
        SpRecPic *p = value;

        if (all || now - p->made > SP_REC_UNSHOWN_US) {
            sp_rec_pic_line(p, 0);
            g_hash_table_iter_remove(&it);
        }
    }
    qemu_mutex_unlock(&rec.lock);
}

static void sp_rec_tick(void *opaque)
{
    int64_t t0 = g_get_monotonic_time();

    sp_rec_unshown(false);
    rec.main_us += g_get_monotonic_time() - t0;
    timer_mod(rec.timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 1000);
}

/* the presenter's thread */
void spacebox_video_record_swap(uint64_t seq, int64_t t_swap, int code,
                                bool code_ok, const uint64_t *tickets,
                                unsigned n, uint64_t coalesced, int64_t read_us,
                                int64_t t_show, int64_t t_call, int64_t t_vsync,
                                int64_t t_out, int64_t t_due, int64_t t_flush_dev,
                                int64_t t_flush_read, int64_t qpc_call,
                                int64_t qpc_ret, int64_t t_flush_rx,
                                int64_t t_sub, int64_t t_ready)
{
    GString *s;

    qemu_mutex_lock(&rec.lock);
    if (!spacebox_video_record_on) {
        qemu_mutex_unlock(&rec.lock);
        return;
    }
    s = g_string_new("{\"k\":\"swap\"");
    g_string_append_printf(s, ",\"seq\":%" PRIu64, seq);
    sp_rec_time(s, "t_us", t_swap);
    if (code >= 0) {
        g_string_append_printf(s, ",\"code\":%d", code);
    } else {
        g_string_append(s, ",\"code\":null");
    }
    g_string_append_printf(s, ",\"code_ok\":%s,\"tickets\":[", code_ok ? "true" : "false");
    for (unsigned i = 0; i < n; i++) {
        g_string_append_printf(s, "%s%" PRIu64, i ? "," : "", tickets[i]);
    }
    g_string_append_printf(s, "],\"coalesced\":%" PRIu64, coalesced);
    sp_rec_time(s, "t_show_us", t_show);
    sp_rec_time(s, "t_call_us", t_call);
    sp_rec_time(s, "t_vsync_us", t_vsync);
    sp_rec_time(s, "t_out_us", t_out);
    sp_rec_time(s, "t_due_us", t_due);
    sp_rec_time(s, "t_flush_dev", t_flush_dev);
    sp_rec_time(s, "t_flush_read", t_flush_read);
    if (qpc_call || qpc_ret) {
        g_string_append_printf(s, ",\"qpc_call\":%" PRId64 ",\"qpc_ret\":%" PRId64,
                               qpc_call, qpc_ret);
    }
    if (t_sub) {
        sp_rec_time(s, "t_flush_rx", t_flush_rx);
        sp_rec_time(s, "t_sub_us", t_sub);
        sp_rec_time(s, "t_ready_us", t_ready);
    }
    g_string_append_c(s, '}');
    sp_rec_put(s);
    for (unsigned i = 0; i < n; i++) {
        SpRecPic *p = g_hash_table_lookup(rec.pics, &tickets[i]);

        if (p) {
            sp_rec_pic_line(p, t_swap);
            g_hash_table_remove(rec.pics, &tickets[i]);
        }
    }
    rec.swaps++;
    if (read_us >= 0) {
        rec.reads++;
        rec.read_us += read_us;
        rec.read_max = MAX(rec.read_max, read_us);
    }
    qemu_mutex_unlock(&rec.lock);
}

void spacebox_video_record_paused(int ms, int64_t t0, int64_t t1)
{
    g_autofree char *more = NULL;

    qemu_mutex_lock(&rec.lock);
    if (spacebox_video_record_on) {
        more = g_strdup_printf(",\"ms\":%d,\"t_end_us\":%" PRId64, ms, t1 + rec.off);
        /* its own start time, not the time of the line */
        GString *s = g_string_new("{\"k\":\"ctl\"");

        sp_rec_time(s, "t_us", t0);
        g_string_append_printf(s, ",\"what\":\"pause\"%s}", more);
        sp_rec_put(s);
    }
    qemu_mutex_unlock(&rec.lock);
}

bool spacebox_video_rect(int *x, int *y, int *w, int *h)
{
    bool on;

    qemu_mutex_lock(&rec.lock);
    on = rec.rect;
    *x = rec.rx;
    *y = rec.ry;
    *w = rec.rw;
    *h = rec.rh;
    qemu_mutex_unlock(&rec.lock);
    return on;
}

/* ---- start, stop, the controls -------------------------------------------- */

static char *sp_rec_rect_json(void)
{
    return rec.rect ? g_strdup_printf("[%d,%d,%d,%d]", rec.rx, rec.ry, rec.rw, rec.rh)
                    : g_strdup("null");
}

static char *sp_rec_size_json(void)
{
    return rec.vw ? g_strdup_printf("[%u,%u]", rec.vw, rec.vh) : g_strdup("null");
}

static void sp_rec_stop(void)
{
    g_autofree char *more = NULL;

    if (!spacebox_video_record_on) {
        fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] stop: no record\n");
        return;
    }
    virgl_renderer_video_record(NULL, rec.vw, rec.vh);
    sp_rec_unshown(true);
    qemu_mutex_lock(&rec.lock);
    spacebox_video_record_on = false;
    more = g_strdup_printf(",\"swaps\":%" PRIu64 ",\"pics\":%" PRIu64
                           ",\"pics_unshown\":%" PRIu64 ",\"code_reads\":%" PRIu64
                           ",\"code_read_us_mean\":%.1f,\"code_read_us_max\":%" PRId64
                           ",\"main_loop_us\":%" PRId64,
                           rec.swaps, rec.n_pics, rec.unshown, rec.reads,
                           rec.reads ? (double)rec.read_us / rec.reads : 0.0,
                           rec.read_max, rec.main_us);
    qemu_mutex_unlock(&rec.lock);
    sp_rec_ctl("record-stop", more);
    g_async_queue_push(rec.lines, &sp_rec_stop_line);
    qemu_thread_join(&rec.writer);
    timer_del(rec.timer);
    fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] stopped: %s (swaps=%" PRIu64
            " pictures=%" PRIu64 " not shown=%" PRIu64 ")\n", rec.path,
            rec.swaps, rec.n_pics, rec.unshown);
    g_free(rec.path);
    rec.path = NULL;
}

/*
 * The sink's build, which the record names so that a calibration of its
 * refresh times can be matched to it: SPACEBOX_SINK_BUILD, else the folder of
 * the build that holds this program (BUILD/qemu-build/qemu-system-aarch64, as
 * screen/mac/sink.sh starts it). NULL when neither is known.
 */
static char *sp_rec_build(void)
{
    const char *b = getenv("SPACEBOX_SINK_BUILD");
    g_autofree char *exe = NULL, *dir = NULL, *name = NULL, *real = NULL;

    if (b && *b) {
        return g_strdup(b);
    }
#ifdef __APPLE__
    {
        char buf[4096];
        uint32_t size = sizeof(buf);

        if (_NSGetExecutablePath(buf, &size) == 0) {
            exe = g_strdup(buf);
        }
    }
#elif defined(_WIN32)
    {
        wchar_t buf[4096];
        DWORD n = GetModuleFileNameW(NULL, buf, G_N_ELEMENTS(buf));

        if (n && n < G_N_ELEMENTS(buf)) {
            exe = g_utf16_to_utf8(buf, n, NULL, NULL, NULL);
        }
    }
#else
    exe = g_file_read_link("/proc/self/exe", NULL);
#endif
    if (!exe) {
        return NULL;
    }
#ifdef _WIN32
    real = g_canonicalize_filename(exe, NULL);
#else
    real = realpath(exe, NULL);
#endif
    dir = g_path_get_dirname(real ? real : exe);
    name = g_path_get_basename(dir);
    return strcmp(name, "qemu-build") ? NULL : g_path_get_dirname(dir);
}

static void sp_rec_start(const char *path)
{
    g_autofree char *more = NULL, *rect = NULL, *size = NULL, *qpath = NULL;
    g_autofree char *build = NULL, *b = sp_rec_build();
    FILE *f;

    if (spacebox_video_record_on) {
        sp_rec_stop();
    }
    f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] cannot write %s: %s\n", path,
                strerror(errno));
        return;
    }
    setvbuf(f, NULL, _IOFBF, 1 << 16);
    rec.path = g_strdup(path);
    rec.off = sp_rec_mono() - g_get_monotonic_time();
    rec.swaps = rec.n_pics = rec.unshown = rec.reads = 0;
    rec.read_us = rec.read_max = rec.main_us = 0;
    qemu_thread_create(&rec.writer, "sp-video-record", sp_rec_writer, f,
                       QEMU_THREAD_JOINABLE);
    rect = sp_rec_rect_json();
    size = sp_rec_size_json();
    qpath = g_strescape(path, NULL);
    if (b) {
        g_autofree char *qb = g_strescape(b, NULL);

        build = g_strdup_printf(",\"sink_build\":\"%s\"", qb);
    }
#ifdef _WIN32
    {
        LARGE_INTEGER hz;
        g_autofree char *b2 = build;

        QueryPerformanceFrequency(&hz);
        build = g_strdup_printf("%s,\"qpc_hz\":%" PRId64, b2 ? b2 : "",
                                (int64_t)hz.QuadPart);
    }
#endif
    more = g_strdup_printf(",\"path\":\"%s\",\"clock\":\"CLOCK_MONOTONIC\","
                           "\"video_rect\":%s,\"video_size\":%s,\"version\":1%s",
                           qpath, rect, size, build ? build : "");
    sp_rec_ctl("record-start", more);
    qemu_mutex_lock(&rec.lock);
    spacebox_video_record_on = true;
    qemu_mutex_unlock(&rec.lock);
    virgl_renderer_video_record(&sp_rec_cbs, rec.vw, rec.vh);
    timer_mod(rec.timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + 1000);
    fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] started: %s\n", path);
}

void sp_record_realize(void)
{
    const char *path = getenv("SPACEBOX_VIDEO_RECORD");

    qemu_mutex_init(&rec.lock);
    rec.lines = g_async_queue_new();
    rec.pics = g_hash_table_new_full(g_int64_hash, g_int64_equal, NULL, g_free);
    rec.timer = timer_new_ms(QEMU_CLOCK_REALTIME, sp_rec_tick, NULL);
    rec.ready = true;
    if (path && *path) {
        sp_rec_start(path);
    }
}

static bool sp_rec_ints(const char *text, int *v, int n)
{
    char *end;

    for (int i = 0; i < n; i++) {
        long x = strtol(text, &end, 10);

        if (end == text || x < 0 || x > 65535) {
            return false;
        }
        v[i] = x;
        text = end;
    }
    while (*text == ' ') {
        text++;
    }
    return !*text;
}

/* a line of the control file; false if it is not one of the record's */
bool sp_record_control(const char *text)
{
    g_autofree char *more = NULL, *json = NULL;
    int v[4];

    if (!rec.ready) {
        return false;
    }
    if (!strncmp(text, "video-rect ", 11)) {
        qemu_mutex_lock(&rec.lock);
        if (!strcmp(text + 11, "off")) {
            rec.rect = false;
        } else if (sp_rec_ints(text + 11, v, 4) && v[2] >= 20 && v[3] > 0) {
            rec.rect = true;
            rec.rx = v[0];
            rec.ry = v[1];
            rec.rw = v[2];
            rec.rh = v[3];
        } else {
            qemu_mutex_unlock(&rec.lock);
            fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] video-rect: X Y W H (W 20 "
                    "or more) or off\n");
            return true;
        }
        json = sp_rec_rect_json();
        qemu_mutex_unlock(&rec.lock);
        if (spacebox_video_record_on) {
            more = g_strdup_printf(",\"rect\":%s", json);
            sp_rec_ctl("video-rect", more);
        }
        fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] video-rect %s\n", json);
    } else if (!strncmp(text, "video-size ", 11)) {
        if (!strcmp(text + 11, "off")) {
            rec.vw = rec.vh = 0;
        } else if (sp_rec_ints(text + 11, v, 2) && v[0] > 0 && v[1] > 0) {
            rec.vw = v[0];
            rec.vh = v[1];
        } else {
            fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] video-size: W H or off\n");
            return true;
        }
        json = sp_rec_size_json();
        /* also with no record: the uploads before one starts get tickets */
        virgl_renderer_video_record(spacebox_video_record_on ? &sp_rec_cbs : NULL,
                                    rec.vw, rec.vh);
        if (spacebox_video_record_on) {
            more = g_strdup_printf(",\"size\":%s", json);
            sp_rec_ctl("video-size", more);
        }
        fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] video-size %s\n", json);
    } else if (!strncmp(text, "pause-present ", 14)) {
        if (!sp_rec_ints(text + 14, v, 1) || v[0] < 1 || v[0] > 10000) {
            fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] pause-present: MS, 1 to 10000\n");
            return true;
        }
        qatomic_set(&spacebox_present_pause_ms, v[0]);
        fprintf(stderr, "[SPACEBOX-VIDEO-RECORD] pause-present %d ms after the "
                "next swap\n", v[0]);
    } else if (!strncmp(text, "record-start ", 13)) {
        sp_rec_start(text + 13);
    } else if (!strcmp(text, "record-stop")) {
        sp_rec_stop();
    } else {
        return false;
    }
    return true;
}
