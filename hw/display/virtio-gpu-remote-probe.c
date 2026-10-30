/*
 * Spacebox remote GPU, sink: the UI evals' probe (screen/ui/README.md of the
 * spacebox repository, "The sink's probe"; include/ui/spacebox-ui-probe.h).
 *
 * The controls (the control file, virtio-gpu-remote-state.c):
 *   ui-start DIR SPEC     DIR/probe.jsonl, DIR/queue.jsonl and DIR/crops/,
 *                         for the strip and the regions of SPEC
 *                         (probe-spec.json)
 *   ui-stop               the files closed, with a last line of the counts
 *                         and of what the probe cost
 *   ui-mark NAME          a line in probe.jsonl
 *   ui-fault NAME ARGS    a fault of the harness's validation; ui-fault off
 *                         ends every one
 *
 * probe.jsonl has one line for each swap of the presenter: its times, and,
 * from the readback that the presenter made of the slot it showed
 * (ui/spacebox-ui-probe-gl.h), the strip's values and each region's checksum;
 * a region's pixels go once into crops/ for each new checksum. The readback
 * is handed over here and read by a thread of its own, so the presenter only
 * copies it. queue.jsonl has a line each queue_ms of the spec, on the main
 * loop: the reader's messages that wait for the main loop, the pictures that
 * a decoder has not handed over, the presenter's slots, the commands that
 * wait for a write in pieces, this sink's own send backlog; and once a second
 * the base by which the source's times are placed on this clock (the
 * lateness's, sp_rx_delay). The lines go to the files through a thread of
 * their own.
 *
 * The faults that act here: drop-key and delay-input (the sink's input and
 * the scroll channel's lines), stuck-rx (the reader's messages,
 * virtio-gpu-remote.c); those that act in the presenter
 * are set here and done there (ui/spacebox-ui-probe-gl.h), and freeze is the
 * record's test pause (pause-present).
 *
 * Every time is CLOCK_MONOTONIC in microseconds, as in the record of video
 * frames (virtio-gpu-remote-record.c): g_get_monotonic_time() with the
 * difference taken at the start; on Windows also the QueryPerformanceCounter
 * of the swap's call and return.
 */

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qobject/qdict.h"
#include "qobject/qjson.h"
#include "qobject/qlist.h"
#include "qobject/qnum.h"
#include "qobject/qstring.h"
#include "hw/virtio/virtio-gpu.h"
#include "ui/spacebox-ui-probe.h"
#include "ui/spacebox-video-record.h"
#include "virtio-gpu-remote.h"
#include "../../ui/spacebox-ui-probe-px.h"

#include <virglrenderer.h>
#include <zlib.h>

bool spacebox_ui_probe_on;
int spacebox_ui_fault_delay_ms, spacebox_ui_fault_late_ms, spacebox_ui_fault_hold;
int spacebox_ui_fault_corrupt_n, spacebox_ui_fault_corrupt_w, spacebox_ui_fault_corrupt_h,
    spacebox_ui_fault_corrupt_amp;
void (*spacebox_ui_probe_slots)(unsigned counts[SP_PROBE_SLOT_STATES],
                                int64_t *ready_oldest_us);

enum { SP_PR_PROBE, SP_PR_QUEUE };

typedef struct SpPrLine {
    int file;
    char *text;
} SpPrLine;

static SpPrLine sp_pr_stop_line;        /* to the writer: end */
static SpProbeShot sp_pr_stop_shot;     /* to the worker: end */

static struct {
    QemuMutex lock;     /* running, the spec, the presenter's counts */
    bool ready, running;
    SpProbeSpec spec;
    unsigned gen;
    int queue_ms;
    char *dir;
    int64_t off;        /* CLOCK_MONOTONIC - g_get_monotonic_time() */
    FILE *f[2];
    GAsyncQueue *lines, *shots;
    QemuThread writer, worker;
    QEMUTimer *timer;
    int64_t last_clock;
    GHashTable *seen;   /* "region crc" of the crops written (the worker's) */
    /* counts: the shots (under lock), the worker's, the main loop's */
    uint64_t n_shots, lost, crops, qlines, faults;
    uint64_t pres_calls;
    int64_t pres_us, pres_max;
    uint64_t works;
    int64_t work_us, work_max;
    /* the faults that act in the sink (the main loop) */
    int drop_n;         /* key-downs until the one dropped, 0 = none */
    int drop_up;        /* the dropped key's qcode, whose key-up goes too; -1 */
    uint64_t dropped;
    bool stuck_armed, stuck_holding;
    int delay_input_ms; /* the input goes to the source this much later, 0 = at once */
    int delay_input_every;      /* > 1: only every so many key-downs (the first, then each EVERY-th), with their
                                 * key-ups, so that one run holds both delayed and undelayed keys */
    int delay_input_n;          /* the key-downs since the fault was set */
    bool delay_input_held[256]; /* by qcode: whether its key-down was held (its key-up follows it) */
} pr = { .drop_up = -1 };

#ifdef _WIN32
/* as winpthreads' clock_gettime(CLOCK_MONOTONIC), to the microsecond */
static int64_t sp_pr_mono(void)
{
    LARGE_INTEGER f, c;
    int64_t ns;

    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    ns = ((c.QuadPart % f.QuadPart) * 1000000000 + (f.QuadPart >> 1)) / f.QuadPart;
    return (int64_t)(c.QuadPart / f.QuadPart) * 1000000 + ns / 1000;
}
#else
static int64_t sp_pr_mono(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}
#endif

/* a time of g_get_monotonic_time() as CLOCK_MONOTONIC, or null */
static void sp_pr_time(GString *s, const char *key, int64_t g)
{
    if (g) {
        g_string_append_printf(s, ",\"%s\":%" PRId64, key, g + pr.off);
    } else {
        g_string_append_printf(s, ",\"%s\":null", key);
    }
}

static void sp_pr_put(int file, GString *s)
{
    SpPrLine *l = g_new(SpPrLine, 1);

    g_string_append_c(s, '\n');
    l->file = file;
    l->text = g_string_free(s, false);
    g_async_queue_push(pr.lines, l);
}

static void *sp_pr_writer(void *opaque)
{
    for (;;) {
        SpPrLine *l = g_async_queue_pop(pr.lines);

        if (l == &sp_pr_stop_line) {
            break;
        }
        fputs(l->text, pr.f[l->file]);
        g_free(l->text);
        g_free(l);
        if (!g_async_queue_length(pr.lines)) {
            fflush(pr.f[SP_PR_PROBE]);
            fflush(pr.f[SP_PR_QUEUE]);
        }
    }
    fclose(pr.f[SP_PR_PROBE]);
    fclose(pr.f[SP_PR_QUEUE]);
    pr.f[0] = pr.f[1] = NULL;
    return NULL;
}

/* ---- the presenter's hand-over and the worker -------------------------- */

bool spacebox_ui_probe_spec(SpProbeSpec *spec)
{
    bool on;

    qemu_mutex_lock(&pr.lock);
    on = pr.running;
    if (on) {
        *spec = pr.spec;
    }
    qemu_mutex_unlock(&pr.lock);
    return on;
}

void spacebox_ui_probe_shot(SpProbeShot *shot)
{
    qemu_mutex_lock(&pr.lock);
    if (!pr.running || shot->gen != pr.spec.gen) {
        qemu_mutex_unlock(&pr.lock);
        g_free(shot->px);
        g_free(shot);
        return;
    }
    pr.n_shots++;
    pr.lost += !!shot->lost;
    g_async_queue_push(pr.shots, shot);
    qemu_mutex_unlock(&pr.lock);
}

void spacebox_ui_probe_cost(int64_t us)
{
    qemu_mutex_lock(&pr.lock);
    pr.pres_calls++;
    pr.pres_us += us;
    pr.pres_max = MAX(pr.pres_max, us);
    qemu_mutex_unlock(&pr.lock);
}

void spacebox_ui_probe_fault(const char *name, const char *more)
{
    int64_t now = g_get_monotonic_time();

    fprintf(stderr, "[SPACEBOX-UI-PROBE] fault %s%s\n", name, more ? more : "");
    qemu_mutex_lock(&pr.lock);
    if (pr.running) {
        GString *s = g_string_new("{\"k\":\"fault\"");

        sp_pr_time(s, "t_us", now);
        g_string_append_printf(s, ",\"name\":\"%s\"%s}", name, more ? more : "");
        sp_pr_put(SP_PR_PROBE, s);
        pr.faults++;
    }
    qemu_mutex_unlock(&pr.lock);
}

/* the crop of a region whose checksum is new: written once */
static void sp_pr_crop(const char *name, const char *crc, const uint8_t *rgb, int w, int h)
{
    g_autofree char *key = g_strdup_printf("%s %s", name, crc);
    g_autofree char *rel = NULL, *path = NULL;
    uLong n = (uLong)w * h * 3;
    uLongf zn = compressBound(n);
    g_autofree uint8_t *z = g_malloc(zn);
    GString *s;
    FILE *f;

    if (g_hash_table_contains(pr.seen, key)) {
        return;
    }
    g_hash_table_add(pr.seen, g_steal_pointer(&key));
    rel = g_strdup_printf("crops/%s-%s.rgb.z", name, crc);
    path = g_build_filename(pr.dir, rel, NULL);
    if (compress2(z, &zn, rgb, n, 1) != Z_OK || !(f = fopen(path, "wb"))) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] cannot write %s\n", path);
        return;
    }
    fwrite(z, 1, zn, f);
    fclose(f);
    s = g_string_new("{\"k\":\"crop\"");
    g_string_append_printf(s, ",\"region\":\"%s\",\"crc\":\"%s\",\"w\":%d,\"h\":%d,"
                           "\"file\":\"%s\"}", name, crc, w, h, rel);
    sp_pr_put(SP_PR_PROBE, s);
    pr.crops++;
}

static void sp_pr_shot_line(SpProbeShot *shot)
{
    GString *s = g_string_new("{\"k\":\"probe\"");
    SpProbeSpec *spec = &pr.spec;   /* the worker's copy is the start's (gen) */

    g_string_append_printf(s, ",\"seq\":%" PRIu64, shot->seq);
    sp_pr_time(s, "t_call_us", shot->t_call);
    sp_pr_time(s, "t_ret_us", shot->t_ret);
    g_string_append_printf(s, ",\"slot\":%d", shot->slot);
#ifdef _WIN32
    g_string_append_printf(s, ",\"qpc_call\":%" PRId64 ",\"qpc_ret\":%" PRId64,
                           shot->qpc_call, shot->qpc_ret);
#endif
    if (shot->sx != 1 || shot->sy != 1) {
        g_string_append_printf(s, ",\"scale\":[%g,%g]", shot->sx, shot->sy);
    }
    if (shot->px) {
        const uint8_t *px = shot->px;
        const SpProbeRect *r = &shot->rects[0];
        bool ok[SP_PROBE_ROWS];
        size_t most = 0;

        g_string_append(s, ",\"strip\":{");
        for (int i = 0; i < spec->nrows; i++) {
            int v = spacebox_ui_probe_strip_row(px, r->w, r->h, (double)r->h / spec->nrows,
                                                i, &ok[i]);

            if (v >= 0) {
                g_string_append_printf(s, "%s\"%s\":%d", i ? "," : "", spec->rows[i], v);
            } else {
                g_string_append_printf(s, "%s\"%s\":null", i ? "," : "", spec->rows[i]);
            }
        }
        g_string_append(s, ",\"ok\":[");
        for (int i = 0; i < spec->nrows; i++) {
            g_string_append_printf(s, "%s%s", i ? "," : "", ok[i] ? "true" : "false");
        }
        g_string_append(s, "]},\"regions\":{");
        px += (size_t)r->w * r->h * 4;
        for (int i = 1; i < shot->nrects; i++) {
            most = MAX(most, (size_t)shot->rects[i].w * shot->rects[i].h * 3);
        }
        g_autofree uint8_t *rgb = g_malloc(MAX(most, 1));
        for (int i = 1; i < shot->nrects; i++) {
            const SpProbeRect *q = &shot->rects[i];
            char crc[9];

            snprintf(crc, sizeof(crc), "%08x", spacebox_ui_probe_rgb(px, q->w, q->h, rgb));
            sp_pr_crop(q->name, crc, rgb, q->w, q->h);
            g_string_append_printf(s, "%s\"%s\":\"%s\"", i > 1 ? "," : "", q->name, crc);
            px += (size_t)q->w * q->h * 4;
        }
        g_string_append_c(s, '}');
        sp_pr_time(s, "t_read_us", shot->t_read);
        g_string_append(s, ",\"lost\":false");
    } else {
        g_string_append_printf(s, ",\"lost\":true,\"why\":\"%s\"",
                               shot->lost ? shot->lost : "none");
    }
    sp_pr_time(s, "t_flush_dev", shot->flush_dev);
    sp_pr_time(s, "t_flush_read", shot->flush_read);
    g_string_append_c(s, '}');
    sp_pr_put(SP_PR_PROBE, s);
}

static void *sp_pr_worker(void *opaque)
{
    for (;;) {
        SpProbeShot *shot = g_async_queue_pop(pr.shots);
        int64_t t0 = g_get_monotonic_time(), d;

        if (shot == &sp_pr_stop_shot) {
            break;
        }
        sp_pr_shot_line(shot);
        g_free(shot->px);
        g_free(shot);
        d = g_get_monotonic_time() - t0;
        pr.works++;
        pr.work_us += d;
        pr.work_max = MAX(pr.work_max, d);
    }
    return NULL;
}

/* ---- the queue line, on the main loop ----------------------------------- */

static void sp_pr_tick(void *opaque)
{
    int64_t now = g_get_monotonic_time(), oldest = 0;
    unsigned c[SP_PROBE_SLOT_STATES] = { 0 };
    SpProbeQueue q;
    GString *s;

    if (!pr.running) {
        return;
    }
    sp_probe_sink_queue(&q);
    if (spacebox_ui_probe_slots) {
        spacebox_ui_probe_slots(c, &oldest);
    }
    s = g_string_new("{\"k\":\"q\"");
    sp_pr_time(s, "t_us", now);
    g_string_append_printf(s, ",\"rx_bytes\":%zu,\"rx_msgs\":%u,\"rx_oldest_us\":%" PRId64
                           ",\"decode_pending\":%u,\"slots\":{\"free\":%u,\"writing\":%u,"
                           "\"ready\":%u,\"reading\":%u,\"displayed\":%u,\"held\":%u},"
                           "\"ready_oldest_us\":%" PRId64 ",\"waiting_cmds\":%u,"
                           "\"writes_incomplete\":%u,\"tx_bytes\":%zu}",
                           q.rx_bytes, q.rx_msgs, q.rx_oldest ? now - q.rx_oldest : 0,
                           virgl_renderer_video_pending(), c[0], c[1], c[2], c[3], c[4], c[5],
                           oldest, q.waiting, q.incomplete, q.tx_backlog);
    sp_pr_put(SP_PR_QUEUE, s);
    pr.qlines++;
    if (now - pr.last_clock >= G_USEC_PER_SEC) {
        pr.last_clock = now;
        s = g_string_new("{\"k\":\"clock\"");
        sp_pr_time(s, "t_us", now);
        if (q.base != INT64_MAX) {
            g_string_append_printf(s, ",\"src_to_client_us\":%" PRId64 "}", q.base + pr.off);
        } else {
            g_string_append(s, ",\"src_to_client_us\":null}");
        }
        sp_pr_put(SP_PR_QUEUE, s);
    }
    timer_mod(pr.timer, qemu_clock_get_us(QEMU_CLOCK_REALTIME) + (int64_t)pr.queue_ms * 1000);
}

/* ---- the controls -------------------------------------------------------- */

static bool sp_pr_int(QDict *d, const char *key, int lo, int hi, int *v)
{
    QNum *n = qobject_to(QNum, qdict_get(d, key));
    int64_t x;

    if (!n || !qnum_get_try_int(n, &x) || x < lo || x > hi) {
        return false;
    }
    *v = (int)x;
    return true;
}

static bool sp_pr_rect(QDict *d, const char *name, SpProbeRect *r)
{
    if (!d || strlen(name) >= sizeof(r->name)) {
        return false;
    }
    pstrcpy(r->name, sizeof(r->name), name);
    return sp_pr_int(d, "x", 0, 16384, &r->x) && sp_pr_int(d, "y", 0, 16384, &r->y);
}

/* probe-spec.json; why it is not one */
static const char *sp_pr_parse(const char *text, SpProbeSpec *spec, int *queue_ms)
{
    QObject *o = qobject_from_json(text, NULL);
    g_autoptr(QDict) d = qobject_to(QDict, o);
    QDict *st;
    QList *rows, *regions;
    QListEntry *e;

    memset(spec, 0, sizeof(*spec));
    *queue_ms = 10;
    if (!d) {
        qobject_unref(o);
        return "not a JSON object";
    }
    st = qdict_get_qdict(d, "strip");
    if (!sp_pr_rect(st, "strip", &spec->strip) || !sp_pr_int(st, "cell", 4, 256, &spec->cell)) {
        return "strip: x, y, cell";
    }
    rows = qdict_get_qlist(st, "rows");
    if (!rows) {
        return "strip: rows";
    }
    QLIST_FOREACH_ENTRY(rows, e) {
        QString *n = qobject_to(QString, qlist_entry_obj(e));

        if (!n || spec->nrows == SP_PROBE_ROWS ||
            strlen(qstring_get_str(n)) >= sizeof(spec->rows[0])) {
            return "strip: rows, at most 4 names of at most 15 characters";
        }
        pstrcpy(spec->rows[spec->nrows++], sizeof(spec->rows[0]), qstring_get_str(n));
    }
    if (!spec->nrows) {
        return "strip: rows";
    }
    spec->strip.w = SP_CODE_CELLS * spec->cell;
    spec->strip.h = spec->nrows * spec->cell;
    regions = qdict_get_qlist(d, "regions");
    for (e = regions ? QTAILQ_FIRST(&regions->head) : NULL; e; e = QTAILQ_NEXT(e, next)) {
        QDict *r = qobject_to(QDict, qlist_entry_obj(e));
        SpProbeRect *q = &spec->regions[spec->nregions];
        const char *name = r ? qdict_get_try_str(r, "name") : NULL;

        if (spec->nregions == SP_PROBE_REGIONS || !name || !sp_pr_rect(r, name, q) ||
            !sp_pr_int(r, "w", 1, 16384, &q->w) || !sp_pr_int(r, "h", 1, 16384, &q->h)) {
            return "regions: at most 4, each name (at most 15 characters), x, y, w, h";
        }
        spec->nregions++;
    }
    if (qdict_haskey(d, "queue_ms") && !sp_pr_int(d, "queue_ms", 1, 1000, queue_ms)) {
        return "queue_ms: 1 to 1000";
    }
    return NULL;
}

static void sp_pr_faults_off(void)
{
    qatomic_set(&spacebox_ui_fault_delay_ms, 0);
    qatomic_set(&spacebox_ui_fault_late_ms, 0);
    qatomic_set(&spacebox_ui_fault_hold, 0);
    qatomic_set(&spacebox_ui_fault_corrupt_n, 0);
    pr.drop_n = 0;
    pr.drop_up = -1;
    pr.delay_input_ms = 0;
    pr.delay_input_every = 0;
    pr.stuck_armed = false;
    if (pr.stuck_holding) {
        pr.stuck_holding = false;
        spacebox_ui_probe_fault("stuck-rx", ",\"released\":true");
        sp_rx_kick();
    }
}

static void sp_pr_stop(void)
{
    GString *s;

    if (!pr.running) {
        return;
    }
    timer_del(pr.timer);
    sp_pr_faults_off();
    qemu_mutex_lock(&pr.lock);
    pr.running = false;
    qatomic_set(&spacebox_ui_probe_on, false);
    qemu_mutex_unlock(&pr.lock);
    g_async_queue_push(pr.shots, &sp_pr_stop_shot);
    qemu_thread_join(&pr.worker);
    s = g_string_new("{\"k\":\"stop\"");
    sp_pr_time(s, "t_us", g_get_monotonic_time());
    g_string_append_printf(s, ",\"swaps\":%" PRIu64 ",\"lost\":%" PRIu64 ",\"crops\":%" PRIu64
                           ",\"queue_lines\":%" PRIu64 ",\"faults\":%" PRIu64
                           ",\"keys_dropped\":%" PRIu64 ",\"present_calls\":%" PRIu64
                           ",\"present_us_mean\":%.1f,\"present_us_max\":%" PRId64
                           ",\"worker_us_mean\":%.1f,\"worker_us_max\":%" PRId64 "}",
                           pr.n_shots, pr.lost, pr.crops, pr.qlines, pr.faults, pr.dropped,
                           pr.pres_calls, pr.pres_calls ? (double)pr.pres_us / pr.pres_calls : 0,
                           pr.pres_max, pr.works ? (double)pr.work_us / pr.works : 0,
                           pr.work_max);
    fprintf(stderr, "[SPACEBOX-UI-PROBE] stopped: %s", s->str + 1);
    sp_pr_put(SP_PR_PROBE, s);
    g_async_queue_push(pr.lines, &sp_pr_stop_line);
    qemu_thread_join(&pr.writer);
    g_hash_table_destroy(pr.seen);
    pr.seen = NULL;
    g_free(pr.dir);
    pr.dir = NULL;
    fprintf(stderr, "\n");
}

static void sp_pr_start(const char *args)
{
    const char *sp = strrchr(args, ' ');
    g_autofree char *dir = NULL, *text = NULL, *crops = NULL, *pp = NULL, *qp = NULL;
    g_autofree char *qdir = NULL, *qspec = NULL;
    g_autoptr(GError) err = NULL;
    const char *why;
    SpProbeSpec spec;
    int queue_ms;
    GString *s;
    FILE *fp, *fq;

    if (!sp || sp == args) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] ui-start DIR SPEC\n");
        return;
    }
    dir = g_strndup(args, sp - args);
    if (!g_file_get_contents(sp + 1, &text, NULL, &err)) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] cannot read %s: %s\n", sp + 1, err->message);
        return;
    }
    why = sp_pr_parse(text, &spec, &queue_ms);
    if (why) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] %s: %s\n", sp + 1, why);
        return;
    }
    sp_pr_stop();
    crops = g_build_filename(dir, "crops", NULL);
    pp = g_build_filename(dir, "probe.jsonl", NULL);
    qp = g_build_filename(dir, "queue.jsonl", NULL);
    if (g_mkdir_with_parents(crops, 0755) || !(fp = fopen(pp, "w"))) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] cannot write in %s: %s\n", dir, strerror(errno));
        return;
    }
    if (!(fq = fopen(qp, "w"))) {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] cannot write %s: %s\n", qp, strerror(errno));
        fclose(fp);
        return;
    }
    setvbuf(fp, NULL, _IOFBF, 1 << 16);
    setvbuf(fq, NULL, _IOFBF, 1 << 16);
    pr.f[SP_PR_PROBE] = fp;
    pr.f[SP_PR_QUEUE] = fq;
    pr.dir = g_steal_pointer(&dir);
    pr.off = sp_pr_mono() - g_get_monotonic_time();
    pr.queue_ms = queue_ms;
    pr.seen = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, NULL);
    pr.n_shots = pr.lost = pr.crops = pr.qlines = pr.faults = pr.dropped = 0;
    pr.pres_calls = pr.works = 0;
    pr.pres_us = pr.pres_max = pr.work_us = pr.work_max = 0;
    pr.last_clock = 0;
    qdir = g_strescape(pr.dir, NULL);
    qspec = g_strescape(sp + 1, NULL);
    for (int file = 0; file < 2; file++) {
        s = g_string_new("{\"k\":\"hdr\",\"tool\":\"spacebox-ui-probe\",\"version\":1");
        g_string_append_printf(s, ",\"file\":\"%s\",\"clock\":\"CLOCK_MONOTONIC\"",
                               file == SP_PR_PROBE ? "probe" : "queue");
        sp_pr_time(s, "t_us", g_get_monotonic_time());
        g_string_append_printf(s, ",\"dir\":\"%s\",\"spec_path\":\"%s\",\"queue_ms\":%d",
                               qdir, qspec, queue_ms);
        g_string_append_printf(s, ",\"strip\":[%d,%d,%d,%d],\"cell\":%d,\"rows\":[",
                               spec.strip.x, spec.strip.y, spec.strip.w, spec.strip.h,
                               spec.cell);
        for (int i = 0; i < spec.nrows; i++) {
            g_string_append_printf(s, "%s\"%s\"", i ? "," : "", spec.rows[i]);
        }
        g_string_append(s, "],\"regions\":[");
        for (int i = 0; i < spec.nregions; i++) {
            const SpProbeRect *q = &spec.regions[i];

            g_string_append_printf(s, "%s{\"name\":\"%s\",\"x\":%d,\"y\":%d,\"w\":%d,\"h\":%d}",
                                   i ? "," : "", q->name, q->x, q->y, q->w, q->h);
        }
        g_string_append_c(s, ']');
#ifdef _WIN32
        {
            LARGE_INTEGER hz;

            QueryPerformanceFrequency(&hz);
            g_string_append_printf(s, ",\"qpc_freq\":%" PRId64, (int64_t)hz.QuadPart);
        }
#endif
        g_string_append_c(s, '}');
        sp_pr_put(file, s);
    }
    qemu_thread_create(&pr.writer, "sp-ui-probe-w", sp_pr_writer, NULL, QEMU_THREAD_JOINABLE);
    qemu_mutex_lock(&pr.lock);
    spec.gen = ++pr.gen;
    pr.spec = spec;
    pr.running = true;
    qatomic_set(&spacebox_ui_probe_on, true);
    qemu_mutex_unlock(&pr.lock);
    qemu_thread_create(&pr.worker, "sp-ui-probe", sp_pr_worker, NULL, QEMU_THREAD_JOINABLE);
    timer_mod(pr.timer, qemu_clock_get_us(QEMU_CLOCK_REALTIME) + (int64_t)queue_ms * 1000);
    fprintf(stderr, "[SPACEBOX-UI-PROBE] started: %s (%d regions, queue line each %d ms)\n",
            pr.dir, spec.nregions, queue_ms);
}

/* a number of the control's arguments, lo to hi each */
static bool sp_pr_ints(const char *text, int *v, int n, int lo, int hi)
{
    char *end;

    for (int i = 0; i < n; i++) {
        long x = strtol(text, &end, 10);

        if (end == text || x < lo || x > hi) {
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

static void sp_pr_fault(const char *text)
{
    g_autofree char *q = g_strescape(text, NULL);
    g_autofree char *more = g_strdup_printf(",\"set\":\"%s\"", q);
    int v[4];

    if (!strcmp(text, "off")) {
        sp_pr_faults_off();
    } else if (!strncmp(text, "delay ", 6) && sp_pr_ints(text + 6, v, 1, 0, 1000)) {
        qatomic_set(&spacebox_ui_fault_delay_ms, v[0]);
    } else if (!strncmp(text, "freeze ", 7) && sp_pr_ints(text + 7, v, 1, 1, 10000)) {
        qatomic_set(&spacebox_present_pause_ms, v[0]);
    } else if (!strncmp(text, "drop-key ", 9) && sp_pr_ints(text + 9, v, 1, 1, 1000)) {
        pr.drop_n = v[0];
        pr.drop_up = -1;
    } else if (!strncmp(text, "corrupt-after-read ", 19) &&
               sp_pr_ints(text + 19, v, 4, 1, 10000) && v[0] <= 256 && v[1] <= 256 &&
               v[2] <= 255) {
        qatomic_set(&spacebox_ui_fault_corrupt_w, v[0]);
        qatomic_set(&spacebox_ui_fault_corrupt_h, v[1]);
        qatomic_set(&spacebox_ui_fault_corrupt_amp, v[2]);
        qatomic_set(&spacebox_ui_fault_corrupt_n, v[3]);
    } else if (!strcmp(text, "hold-slot")) {
        qatomic_set(&spacebox_ui_fault_hold, 1);
    } else if (!strcmp(text, "stuck-rx")) {
        pr.stuck_armed = true;
    } else if (!strncmp(text, "late-enqueue ", 13) && sp_pr_ints(text + 13, v, 1, 1, 2000)) {
        qatomic_set(&spacebox_ui_fault_late_ms, v[0]);
    } else if (!strncmp(text, "delay-input ", 12) && (sp_pr_ints(text + 12, v, 1, 0, 1000) ||
                                                       sp_pr_ints(text + 12, v, 2, 0, 1000))) {
        /* delay-input MS [EVERY]: a comparison of two runs carries each run's own phase of the guest's frames
         * against the client's refresh (up to a refresh); with EVERY the delayed and the undelayed keys are in one
         * run, at random input times, and their difference is the hold alone */
        pr.delay_input_ms = v[0];
        pr.delay_input_every = sp_pr_ints(text + 12, v, 2, 0, 1000) ? v[1] : 0;
        pr.delay_input_n = 0;
        memset(pr.delay_input_held, 0, sizeof(pr.delay_input_held));
    } else {
        fprintf(stderr, "[SPACEBOX-UI-PROBE] ui-fault: off, delay MS, freeze MS, "
                "drop-key N, corrupt-after-read W H AMP N, hold-slot, stuck-rx, "
                "late-enqueue MS, delay-input MS [EVERY]\n");
        return;
    }
    spacebox_ui_probe_fault("set", more);
}

/* a line of the control file; false if it is not one of the probe's */
bool sp_probe_control(const char *text)
{
    if (!pr.ready) {
        return false;
    }
    if (!strncmp(text, "ui-start ", 9)) {
        sp_pr_start(text + 9);
    } else if (!strcmp(text, "ui-stop")) {
        sp_pr_stop();
    } else if (!strncmp(text, "ui-mark ", 8)) {
        g_autofree char *q = g_strescape(text + 8, NULL);

        qemu_mutex_lock(&pr.lock);
        if (pr.running) {
            GString *s = g_string_new("{\"k\":\"mark\"");

            sp_pr_time(s, "t_us", g_get_monotonic_time());
            g_string_append_printf(s, ",\"name\":\"%s\"}", q);
            sp_pr_put(SP_PR_PROBE, s);
        }
        qemu_mutex_unlock(&pr.lock);
    } else if (!strncmp(text, "ui-fault ", 9)) {
        sp_pr_fault(text + 9);
    } else {
        return false;
    }
    return true;
}

/* ---- the faults that act in the sink ----------------------------------- */

bool sp_probe_drop_key(int qcode, bool down)
{
    g_autofree char *more = NULL;

    if (!down && qcode == pr.drop_up) {
        pr.drop_up = -1;
        more = g_strdup_printf(",\"qcode\":%d,\"down\":false", qcode);
        spacebox_ui_probe_fault("drop-key", more);
        return true;
    }
    if (down && pr.drop_n > 0 && --pr.drop_n == 0) {
        pr.drop_up = qcode;
        pr.dropped++;
        more = g_strdup_printf(",\"qcode\":%d,\"down\":true", qcode);
        spacebox_ui_probe_fault("drop-key", more);
        return true;
    }
    return false;
}

int sp_probe_input_every(void)
{
    return pr.delay_input_ms ? pr.delay_input_every : 0;
}

int sp_probe_input_delay_ms(const char *kind, int a, int b)
{
    int ms = pr.delay_input_ms;
    bool held;

    if (!ms || pr.delay_input_every <= 1 || !kind || strcmp(kind, "key")) {
        return ms;
    }
    if (!b) {           /* a key-up follows its key-down */
        return a >= 0 && a < 256 && pr.delay_input_held[a] ? ms : 0;
    }
    held = pr.delay_input_n++ % pr.delay_input_every == 0;
    if (a >= 0 && a < 256) {
        pr.delay_input_held[a] = held;
    }
    return held ? ms : 0;
}

void sp_probe_input_held(const char *kind, int a, int b, int64_t hold_us)
{
    g_autofree char *more = g_strdup_printf(",\"kind\":\"%s\",\"a\":%d,\"b\":%d,"
                                            "\"hold_us\":%" PRId64, kind, a, b, hold_us);

    spacebox_ui_probe_fault("delay-input", more);
}

bool sp_probe_rx_stuck(void)
{
    return pr.stuck_holding;
}

bool sp_probe_rx_hold(bool is_flush)
{
    if (!pr.stuck_armed || !is_flush) {
        return false;
    }
    pr.stuck_armed = false;
    pr.stuck_holding = true;
    spacebox_ui_probe_fault("stuck-rx", ",\"held\":true");
    return true;
}

void sp_probe_realize(void)
{
    qemu_mutex_init(&pr.lock);
    pr.lines = g_async_queue_new();
    pr.shots = g_async_queue_new();
    pr.timer = timer_new_us(QEMU_CLOCK_REALTIME, sp_pr_tick, NULL);
    pr.ready = true;
}
