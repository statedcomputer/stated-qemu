/*
 * The sink's sound (ui/spacebox-sound.h): a model of the sink's buffer, the backend and the sound device, with the
 * sink's rules of what is handed over (sp_sink_audio_cb) and of when it plays faster (sp_sink_audio). It runs with the
 * rule of today and with the rule that read the surplus from the lead, which must show what the sink's log showed on
 * 2026-10-29: 1% faster for good, and about one request a second that found nothing.
 */

#include "qemu/osdep.h"
#include "../../ui/spacebox-sound.h"

#define FREQ 48000
#define FRAME 4
#define BPS (FREQ * FRAME)
#define PACKET_US 10000
#define PACKET_FRAMES 480
#define REQUEST_FRAMES 512
#define BACKEND_FRAMES 4096
#define TICK_US 5000
#define LEAD_OLD_US 20000
#define TARGET_US INT64_C(80000)

typedef struct {
    bool old_rule;      /* the surplus read from the lead */
    int64_t lead_us;
    int seconds;
    int hold_at_s;      /* a hold-up of the sound at this second, 0 = none */
    int hold_ms;
} Run;

typedef struct {
    double ring, backend, frac;     /* frames */
    uint64_t fed;
    int64_t t_start, held_avg;
    bool playing, fast;
    uint64_t fast_us, starved_while_playing, in, out, played;     /* played: frames that the device took */
    uint64_t played_start;
    int last_fast_s;
} Sim;

static void hand_over(Sim *s, const Run *r, int64_t now)
{
    int64_t allowed;

    if (!s->playing) {
        return;
    }
    /* the old rule's clock, or what the device played */
    allowed = r->old_rule ? (now - s->t_start + r->lead_us) * FREQ / 1000000 - (int64_t)(s->fed / FRAME)
                          : (int64_t)(s->played - s->played_start) + r->lead_us * FREQ / 1000000 - (int64_t)(s->fed / FRAME);
    while (allowed > 0 && s->backend < BACKEND_FRAMES && s->ring >= 1) {
        s->ring -= s->fast ? 1.01 : 1.0;
        s->ring = MAX(s->ring, 0);
        s->backend++;
        s->fed += FRAME;
        s->out++;
        allowed--;
        s->fast_us += s->fast ? 1000000 / FREQ : 0;
    }
}

static void arrive(Sim *s, const Run *r, int frames, int64_t now)
{
    int64_t held, ring_us, target_us = TARGET_US;

    s->ring += frames;
    s->in += frames;
    if (!s->playing && s->ring >= TARGET_US * FREQ / 1000000) {
        s->playing = true;
        s->t_start = now;
        s->played_start = s->played;
        s->fed = 0;
    }
    ring_us = (int64_t)s->ring * 1000000 / FREQ;
    if (r->old_rule) {
        held = ring_us + r->lead_us - LEAD_OLD_US;
    } else {
        int64_t played_us = (s->played - s->played_start) * 1000000 / FREQ;

        held = ring_us + (s->playing ? spacebox_sound_queued_us(s->fed, BPS, played_us) : 0);
        held = MAX(held - LEAD_OLD_US, 1);
    }
    s->held_avg = s->held_avg ? s->held_avg + (held - s->held_avg) / 64 : held;
    s->fast = spacebox_sound_fast(s->fast, s->held_avg, target_us);
}

static Sim run(const Run *r)
{
    Sim s = { 0 };
    int64_t end = r->seconds * G_USEC_PER_SEC, next_packet = 0, next_request = 0, next_tick = 0;
    int64_t request_us = REQUEST_FRAMES * G_USEC_PER_SEC / FREQ, hold_from = r->hold_at_s * G_USEC_PER_SEC;
    int64_t hold_to = hold_from + r->hold_ms * 1000;

    for (int64_t now = 0; now < end; now += 100) {
        if (now >= next_packet && (!r->hold_at_s || now < hold_from || now >= hold_to)) {
            /* what the hold-up kept back arrives at once after it */
            do {
                arrive(&s, r, PACKET_FRAMES, now);
                next_packet += PACKET_US;
            } while (next_packet <= now);
        }
        if (now >= next_tick) {
            hand_over(&s, r, now);
            next_tick += TICK_US;
        }
        if (now >= next_request) {
            if (s.backend >= REQUEST_FRAMES) {
                s.backend -= REQUEST_FRAMES;
                s.played += REQUEST_FRAMES;
            } else if (s.playing && now - s.t_start > 500000) {
                s.starved_while_playing++;
            }
            next_request += request_us;
        }
        if (s.fast) {
            s.last_fast_s = now / G_USEC_PER_SEC;
        }
    }
    return s;
}

/*
 * The rule that read the surplus from the lead, at the lead of the log: one hold-up turns the fast mode on, and it
 * stays on with nothing left to work off. What the log showed, for the 50 s after the hold-up was worked off.
 */
static void test_old_rule_shows_the_log(void)
{
    Run r = { .old_rule = true, .lead_us = 100000, .seconds = 60, .hold_at_s = 2, .hold_ms = 300 };
    Sim s = run(&r);

    g_assert_true(s.fast);
    g_assert_cmpint(s.last_fast_s, ==, 59);
    g_assert_cmpuint(s.starved_while_playing, >=, 35);
    g_assert_cmpfloat((double)s.out / s.in, <, 0.995);
}

/* steady sound at the same lead: nothing played faster, no request found nothing */
static void test_no_surplus_at_the_largest_lead(void)
{
    Run r = { .lead_us = 100000, .seconds = 60 };
    Sim s = run(&r);

    g_assert_false(s.fast);
    g_assert_cmpuint(s.fast_us, ==, 0);
    g_assert_cmpuint(s.starved_while_playing, ==, 0);
}

/* at the default lead too */
static void test_no_surplus_at_the_default_lead(void)
{
    Run r = { .lead_us = 65000, .seconds = 60 };
    Sim s = run(&r);

    g_assert_cmpuint(s.fast_us, ==, 0);
    g_assert_cmpuint(s.starved_while_playing, ==, 0);
}

/* a hold-up of 300 ms: the sound behind it arrives at once, is played faster until it is worked off, and then not */
static void test_a_real_surplus_is_worked_off(void)
{
    Run r = { .lead_us = 100000, .seconds = 60, .hold_at_s = 10, .hold_ms = 300 };
    Sim s = run(&r);

    g_assert_false(s.fast);
    g_assert_cmpuint(s.fast_us, >, 1 * G_USEC_PER_SEC);
    g_assert_cmpint(s.last_fast_s, <, 55);
    /* the hold-up itself leaves the device short for its own length, once */
    g_assert_cmpuint(s.starved_while_playing, <=, 30);
}

/* a ramp through the ring, read at another rate over requests that wrap around it: the same ramp, with no step */
static void test_the_device_at_another_rate(void)
{
    enum { RING = 4096, CH = 2, REQ = 512 };
    static float ring[RING * CH], out[REQ * CH];
    double step = 44100.0 / 48000, phase = 0;
    size_t start = 3000, total = 0;
    float last = -1;

    /* each frame is its own number in the ramp, written again as the reading goes round */
    for (int request = 0; request < 2000; request++) {
        size_t used;

        for (size_t i = 0; i < spacebox_sound_need(REQ, step); i++) {
            size_t at = (start + i) % RING;

            ring[at * CH] = (float)(total + i);
            ring[at * CH + 1] = -(float)(total + i);
        }
        used = spacebox_sound_resample(ring, RING, start, CH, out, REQ, step, &phase);
        g_assert_cmpuint(used, <=, spacebox_sound_need(REQ, step) - 1);
        for (int i = 0; i < REQ; i++) {
            if (last >= 0) {
                /* (a float holds these numbers to about 0.06) */
                g_assert_cmpfloat_with_epsilon(out[i * CH] - last, (float)step, 0.2);
            }
            g_assert_cmpfloat(out[i * CH + 1], ==, -out[i * CH]);
            last = out[i * CH];
        }
        start = (start + used) % RING;
        total += used;
    }
    /* in the time of 2000 requests the device takes the frames of the voice's rate */
    g_assert_cmpfloat_with_epsilon((double)total, 2000.0 * REQ * step, 2.0);
}

/* 44.1 kHz from a voice of 48 kHz as well */
static void test_the_device_slower_than_the_voice(void)
{
    enum { RING = 4096, REQ = 512 };
    static float ring[RING], out[REQ];
    double step = 48000.0 / 44100, phase = 0;
    size_t used;

    for (int i = 0; i < RING; i++) {
        ring[i] = i;
    }
    used = spacebox_sound_resample(ring, RING, 100, 1, out, REQ, step, &phase);
    g_assert_cmpuint(used, ==, (size_t)(REQ * step));
    g_assert_cmpfloat(phase, <, 1.0);
    g_assert_cmpfloat_with_epsilon(out[REQ - 1], 100 + (REQ - 1) * step, 0.01);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/spacebox-sound/old-rule-shows-the-log", test_old_rule_shows_the_log);
    g_test_add_func("/spacebox-sound/no-surplus-at-the-largest-lead", test_no_surplus_at_the_largest_lead);
    g_test_add_func("/spacebox-sound/no-surplus-at-the-default-lead", test_no_surplus_at_the_default_lead);
    g_test_add_func("/spacebox-sound/a-real-surplus-is-worked-off", test_a_real_surplus_is_worked_off);
    g_test_add_func("/spacebox-sound/the-device-at-another-rate", test_the_device_at_another_rate);
    g_test_add_func("/spacebox-sound/the-device-slower-than-the-voice", test_the_device_slower_than_the_voice);
    return g_test_run();
}
