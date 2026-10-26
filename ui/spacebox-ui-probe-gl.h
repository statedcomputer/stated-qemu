/*
 * Spacebox: the UI evals' probe in the presenter (include/ui/spacebox-ui-probe.h;
 * screen/ui/README.md of the spacebox repository, "The sink's probe").
 * Included by ui/spacebox-presenter.h after its slots; it runs on the
 * presenter's thread, in its GL context.
 *
 * After each swap returns, the strip and the regions of the probe's spec are
 * read back from the slot just shown, at the slot's size (on Windows the
 * guest's picture at its own size; on the Mac the window's drawable, which is
 * the guest's size at Retina; the shot says the scale otherwise), into a pixel
 * buffer of a ring, with a fence. A buffer whose fence is done is copied out
 * and handed to the sink's worker (hw/display/virtio-gpu-remote-probe.c),
 * which reads the strip and the checksums, after a later swap or while the
 * presenter waits for the display. Nothing of it runs between a frame's pick
 * and its swap, and the presentation never waits for it: a swap that finds
 * the ring full is lost, with its times kept.
 *
 * The faults of the harness's validation that act here (ui-fault): delay
 * (each picked frame waits more before its swap), hold-slot (one frame kept
 * in its slot, never shown), late-enqueue (a frame submitted is not shown
 * before its time is up), corrupt-after-read (a block of the picture swapped
 * is changed after the slot it comes from, which the probe reads), and the
 * freeze, which is the test pause of the record (pause-present).
 */

#include "ui/spacebox-ui-probe.h"

#define SP_PROBE_RING 8

typedef struct SpProbeRead {
    GLuint pbo;
    size_t size;            /* the buffer's */
    size_t bytes;           /* of the readback in it */
    GLsync fence;
    SpProbeShot *shot;      /* waiting for its fence, or NULL */
} SpProbeRead;

static SpProbeRead sp_probe_ring[SP_PROBE_RING];
static SpPresenter *sp_probe_presenter;
static GLuint sp_probe_cfb, sp_probe_ctex;  /* corrupt-after-read's block */
static int sp_probe_delay_logged, sp_probe_late_logged;
static int sp_probe_n_held;     /* frames hold-slot kept (atomic) */

/* the slots by state, for the sink's queue line (on the main loop) */
static void sp_probe_slots_of(unsigned c[SP_PROBE_SLOT_STATES], int64_t *oldest)
{
    SpPresenter *p = sp_probe_presenter;
    int64_t now = g_get_monotonic_time(), o = 0;

    memset(c, 0, sizeof(unsigned) * SP_PROBE_SLOT_STATES);
    *oldest = 0;
    if (!p) {
        return;
    }
    qemu_mutex_lock(&p->lock);
    for (int i = 0; i < SP_SLOTS; i++) {
        SpSlot *s = &p->slots[i];

        c[s->state]++;
        if (s->state == SP_READY) {
            c[5] += s->probe_held;
            if (!o || s->submitted < o) {
                o = s->submitted;
            }
        }
    }
    qemu_mutex_unlock(&p->lock);
    *oldest = o ? now - o : 0;
}

/* the readbacks whose fence is done (all: waited for) are handed over */
static void sp_probe_harvest(bool all)
{
    for (int i = 0; i < SP_PROBE_RING; i++) {
        SpProbeRead *r = &sp_probe_ring[i];
        SpProbeShot *shot = r->shot;
        int64_t t0;

        if (!shot) {
            continue;
        }
        if (all) {
            glClientWaitSync(r->fence, GL_SYNC_FLUSH_COMMANDS_BIT, 100000000);
        } else if (!sp_fence_done(r->fence)) {
            continue;
        }
        t0 = g_get_monotonic_time();
        glBindBuffer(GL_PIXEL_PACK_BUFFER, r->pbo);
        const uint8_t *m = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, (GLsizeiptr)r->bytes,
                                            GL_MAP_READ_BIT);
        if (m) {
            shot->px = g_malloc(r->bytes);
            memcpy(shot->px, m, r->bytes);
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            shot->t_read = g_get_monotonic_time();
        } else {
            shot->lost = "map";
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glDeleteSync(r->fence);
        r->fence = NULL;
        r->shot = NULL;
        spacebox_ui_probe_shot(shot);
        spacebox_ui_probe_cost(g_get_monotonic_time() - t0);
    }
}

/* a rectangle of the scanout in the slot's pixels; false when it is not inside */
static bool sp_probe_map(const SpProbeRect *in, double sx, double sy, const SpSlot *s,
                         SpProbeRect *out)
{
    *out = *in;
    out->x = (int)(in->x * sx + 0.5);
    out->y = (int)(in->y * sy + 0.5);
    out->w = (int)(in->w * sx + 0.5);
    out->h = (int)(in->h * sy + 0.5);
    return out->w > 0 && out->h > 0 && out->x >= 0 && out->y >= 0 &&
           out->x + out->w <= s->fb.width && out->y + out->h <= s->fb.height;
}

/* after the swap of the slot s (index pick, the swap seq): its readback */
static void sp_probe_after_swap(SpSlot *s, int pick, GLuint readfb, uint64_t seq,
                                int64_t t_call, int64_t t_ret, int64_t qpc_call,
                                int64_t qpc_ret)
{
    SpProbeSpec spec;
    SpProbeRead *r = NULL;
    SpProbeShot *shot;
    size_t bytes = 0, off = 0;
    int64_t t0 = g_get_monotonic_time();

    if (!qatomic_read(&spacebox_ui_probe_on) || !spacebox_ui_probe_spec(&spec)) {
        return;
    }
    sp_probe_harvest(false);
    shot = g_new0(SpProbeShot, 1);
    shot->gen = spec.gen;
    shot->seq = seq;
    shot->slot = pick;
    shot->t_call = t_call;
    shot->t_ret = t_ret;
    shot->qpc_call = qpc_call;
    shot->qpc_ret = qpc_ret;
    shot->flush_dev = s->flush_dev;
    shot->flush_read = s->flush_read;
    shot->sx = s->src_w ? (double)s->fb.width / s->src_w : 1;
    shot->sy = s->src_h ? (double)s->fb.height / s->src_h : 1;
    shot->nrects = 1 + spec.nregions;
    for (int i = 0; i < shot->nrects; i++) {
        const SpProbeRect *in = i ? &spec.regions[i - 1] : &spec.strip;

        if (!sp_probe_map(in, shot->sx, shot->sy, s, &shot->rects[i])) {
            shot->lost = "outside";
        }
        bytes += (size_t)shot->rects[i].w * shot->rects[i].h * 4;
    }
    for (int i = 0; !shot->lost && i < SP_PROBE_RING; i++) {
        if (!sp_probe_ring[i].shot) {
            r = &sp_probe_ring[i];
        }
    }
    if (!shot->lost && !r) {
        shot->lost = "ring";
    }
    if (shot->lost) {
        spacebox_ui_probe_shot(shot);
        spacebox_ui_probe_cost(g_get_monotonic_time() - t0);
        return;
    }
    if (!r->pbo) {
        glGenBuffers(1, &r->pbo);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, r->pbo);
    if (r->size < bytes) {
        glBufferData(GL_PIXEL_PACK_BUFFER, (GLsizeiptr)bytes, NULL, GL_STREAM_READ);
        r->size = bytes;
    }
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           s->fb.texture, 0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    for (int i = 0; i < shot->nrects; i++) {
        const SpProbeRect *q = &shot->rects[i];

        /* GL's rows count from the bottom */
        glReadPixels(q->x, s->fb.height - q->y - q->h, q->w, q->h, GL_RGBA,
                     GL_UNSIGNED_BYTE, (void *)(uintptr_t)off);
        off += (size_t)q->w * q->h * 4;
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    r->fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    r->bytes = bytes;
    r->shot = shot;
    spacebox_ui_probe_cost(g_get_monotonic_time() - t0);
}

/*
 * corrupt-after-read: a block in the type region of the picture about to be
 * swapped (in the window's buffer, dst_w x dst_h) changed by amp levels in
 * each colour; the slot it comes from, which the probe reads back, stays as
 * it is. Its readback of the block waits for the GPU between the pick and
 * the swap: a cost of the fault only.
 */
static void sp_probe_corrupt(SpSlot *s, GLuint readfb, int dst_w, int dst_h)
{
    SpProbeSpec spec;
    const SpProbeRect *t = NULL;
    SpProbeRect b, q;
    int left = qatomic_read(&spacebox_ui_fault_corrupt_n), amp = spacebox_ui_fault_corrupt_amp;

    if (left <= 0 || !spacebox_ui_probe_spec(&spec)) {
        return;
    }
    for (int i = 0; i < spec.nregions; i++) {
        if (!strcmp(spec.regions[i].name, "type")) {
            t = &spec.regions[i];
        }
    }
    if (!t) {
        return;
    }
    b = *t;
    b.x += 8;
    b.y += 8;
    b.w = spacebox_ui_fault_corrupt_w;
    b.h = spacebox_ui_fault_corrupt_h;
    double sx = s->src_w ? (double)s->fb.width / s->src_w : 1;
    double sy = s->src_h ? (double)s->fb.height / s->src_h : 1;
    if (!sp_probe_map(&b, sx, sy, s, &q)) {
        return;
    }
    g_autofree uint8_t *px = g_malloc((size_t)q.w * q.h * 4);

    glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           s->fb.texture, 0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glReadPixels(q.x, s->fb.height - q.y - q.h, q.w, q.h, GL_RGBA, GL_UNSIGNED_BYTE, px);
    for (size_t i = 0; i < (size_t)q.w * q.h * 4; i++) {
        if (i % 4 != 3) {
            px[i] = px[i] + amp <= 255 ? px[i] + amp : px[i] - amp;
        }
    }
    if (!sp_probe_ctex) {
        glGenTextures(1, &sp_probe_ctex);
        glGenFramebuffers(1, &sp_probe_cfb);
    }
    glBindTexture(GL_TEXTURE_2D, sp_probe_ctex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, q.w, q.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, sp_probe_cfb);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                           sp_probe_ctex, 0);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    double dx = (double)dst_w / s->fb.width, dy = (double)dst_h / s->fb.height;
    int y0 = s->fb.height - q.y - q.h;

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, q.w, q.h, (int)(q.x * dx), (int)(y0 * dy),
                      (int)((q.x + q.w) * dx), (int)((y0 + q.h) * dy),
                      GL_COLOR_BUFFER_BIT, dx == 1 && dy == 1 ? GL_NEAREST : GL_LINEAR);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, readfb);
    left = qatomic_fetch_dec(&spacebox_ui_fault_corrupt_n) - 1;
    {
        g_autofree char *more = g_strdup_printf(",\"rect\":[%d,%d,%d,%d],\"amp\":%d,\"left\":%d",
                                                b.x, b.y, b.w, b.h, amp, MAX(left, 0));

        spacebox_ui_probe_fault("corrupt-after-read", more);
    }
}

/*
 * A frame that a fault keeps from being shown now: one kept in its slot
 * (hold-slot), or one whose late-enqueue time is not up (*next_due then).
 * Under the presenter's lock.
 */
static bool sp_probe_keeps(SpSlot *s, int64_t now, int64_t *next_due)
{
    if (s->probe_held) {
        return true;
    }
    if (s->probe_late > now) {
        if (!*next_due || s->probe_late < *next_due) {
            *next_due = s->probe_late;
        }
        return true;
    }
    return false;
}

/* hold-slot let go (off): the kept frame's slot is free again */
static void sp_probe_release(SpPresenter *p)
{
    bool freed = false;

    if (!qatomic_read(&sp_probe_n_held) || qatomic_read(&spacebox_ui_fault_hold)) {
        return;
    }
    qatomic_set(&sp_probe_n_held, 0);
    qemu_mutex_lock(&p->lock);
    for (int i = 0; i < SP_SLOTS; i++) {
        if (p->slots[i].probe_held) {
            p->slots[i].probe_held = false;
            if (p->slots[i].state == SP_READY) {
                p->slots[i].state = SP_FREE;
            }
            freed = true;
        }
    }
    qemu_mutex_unlock(&p->lock);
    if (freed) {
        spacebox_ui_probe_fault("hold-slot", ",\"released\":true");
    }
}

/* at a submit (under the presenter's lock): the faults that keep this frame;
 * true when hold-slot keeps it */
static bool sp_probe_submitted(SpSlot *s)
{
    int late = qatomic_read(&spacebox_ui_fault_late_ms);

    s->probe_held = qatomic_cmpxchg(&spacebox_ui_fault_hold, 1, 2) == 1;
    if (s->probe_held) {
        qatomic_inc(&sp_probe_n_held);
    }
    s->probe_late = late > 0 ? s->submitted + (int64_t)late * 1000 : 0;
    return s->probe_held;
}

/* after the submit of frame seq, outside the lock: the lines of the faults */
static void sp_probe_submit_lines(bool held, uint64_t seq)
{
    int late = qatomic_read(&spacebox_ui_fault_late_ms);

    if (held) {
        g_autofree char *more = g_strdup_printf(",\"seq\":%" PRIu64, seq);

        spacebox_ui_probe_fault("hold-slot", more);
    }
    if (late != sp_probe_late_logged) {
        g_autofree char *more = g_strdup_printf(",\"ms\":%d", late);

        sp_probe_late_logged = late;
        spacebox_ui_probe_fault("late-enqueue", more);
    }
}

/* delay: the picked frame waits before its swap (between the pick and the swap) */
static void sp_probe_delay(void)
{
    int ms = qatomic_read(&spacebox_ui_fault_delay_ms);

    if (ms != sp_probe_delay_logged) {
        g_autofree char *more = g_strdup_printf(",\"ms\":%d", ms);

        sp_probe_delay_logged = ms;
        spacebox_ui_probe_fault("delay", more);
    }
    if (ms > 0) {
        g_usleep((gulong)ms * 1000);
    }
}

static void sp_probe_end(void)
{
    sp_probe_harvest(true);
    for (int i = 0; i < SP_PROBE_RING; i++) {
        if (sp_probe_ring[i].pbo) {
            glDeleteBuffers(1, &sp_probe_ring[i].pbo);
            sp_probe_ring[i].pbo = 0;
            sp_probe_ring[i].size = 0;
        }
    }
    if (sp_probe_ctex) {
        glDeleteTextures(1, &sp_probe_ctex);
        glDeleteFramebuffers(1, &sp_probe_cfb);
        sp_probe_ctex = sp_probe_cfb = 0;
    }
}
