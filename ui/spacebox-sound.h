/*
 * The sink's sound (hw/display/virtio-gpu-remote.c): how much waits to be heard, and when it is played faster to
 * work off a surplus. Kept apart so that tests/unit/test-spacebox-sound.c runs them with no sound device.
 */
#ifndef UI_SPACEBOX_SOUND_H
#define UI_SPACEBOX_SOUND_H

#include <stdbool.h>
#include <stdint.h>

/*
 * What the backend was given and has not played yet, us, by the clock: the time of what was fed, less the time since
 * playing began. The lead is only how much the backend may be given; when the sound that arrives is less, this is
 * less than the lead, and a surplus must not be read from the lead.
 */
static inline int64_t spacebox_sound_queued_us(uint64_t fed_bytes, uint64_t bytes_per_s, int64_t played_us)
{
    int64_t fed_us = (int64_t)(fed_bytes * 1000000 / bytes_per_s);

    return fed_us > played_us ? fed_us - played_us : 0;
}

/* One step of the choice to play 1% faster: on well above the target, off near it, as it was between the two. */
static inline bool spacebox_sound_fast(bool fast, int64_t held_avg_us, int64_t target_us)
{
    if (held_avg_us > target_us * 5 / 4) {
        return true;
    }
    if (held_avg_us < target_us * 21 / 20) {
        return false;
    }
    return fast;
}

/*
 * The device's rate can differ from the rate of the sound (another program or a new display set it): out_frames frames
 * for the device from a ring of interleaved float frames, read from `start` with a step of step input frames for each
 * output frame, by a straight line between two frames. *phase keeps the position between two input frames for the next
 * request. Returns the input frames that are done with; it reads one frame beyond them, so the ring must hold
 * spacebox_sound_need() of them.
 */
static inline size_t spacebox_sound_need(size_t out_frames, double step)
{
    return (size_t)(out_frames * step) + 2;
}

static inline size_t spacebox_sound_resample(const float *ring, size_t ring_frames, size_t start, int channels,
                                             float *out, size_t out_frames, double step, double *phase)
{
    double pos = *phase;
    size_t used;

    for (size_t i = 0; i < out_frames; i++) {
        size_t whole = (size_t)pos;
        size_t a = (start + whole) % ring_frames, b = (a + 1) % ring_frames;
        float f = (float)(pos - whole);

        for (int c = 0; c < channels; c++) {
            out[i * channels + c] = ring[a * channels + c] * (1 - f) + ring[b * channels + c] * f;
        }
        pos += step;
    }
    used = (size_t)pos;
    *phase = pos - used;
    return used;
}

#endif
