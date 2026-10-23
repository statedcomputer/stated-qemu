/*
 * Spacebox remote GPU: what the connection (virtio-gpu-remote.c) and the
 * sink's record of its state (virtio-gpu-remote-state.c) share. Internal to
 * these two files.
 */

#ifndef HW_DISPLAY_VIRTIO_GPU_REMOTE_H
#define HW_DISPLAY_VIRTIO_GPU_REMOTE_H

#include "hw/virtio/virtio-gpu.h"

/*
 * The messages that build a renderer's state again (sent by the Firecracker
 * device; the QEMU source never sends them). See the protocol of the screen
 * track; the header and the other types are in virtio-gpu-remote.c.
 */
enum {
    SP_WMARK = 16,       /* source -> sink. a: u32[] resources the next SP_CTRL
                            may write (on the GPU or by an upload) */
    SP_STATE_RESET = 17, /* source -> sink. arg0 = N: drop every context and
                            every resource but those that existed at message id
                            N and were not written after it (N = 0: all) */
    SP_STATE_KEPT = 18,  /* sink -> source. arg0 = N, a: u32[] the kept resources */
    SP_IMPORT = 19,      /* source -> sink. a: SpXfer, b: the bytes */
    SP_EXPORT = 22,      /* source -> sink. arg0 the request id (not 0: an
                            export without one is refused); a: u32[]
                            resources, in that order */
    SP_EXPORT_DATA = 23, /* sink -> source. arg2 the request id; a: SpXfer,
                            b: at most SP_XFER_MAX bytes */
    SP_EXPORT_DONE = 24, /* sink -> source. arg0 SP_EXPORT_DATA messages sent,
                            arg1 their bytes, arg2 the request id; a: u32[]
                            asked but not exported */
    SP_DETACH = 25,      /* sink -> source: this sink ends on purpose */
    SP_BYE = 26,         /* source -> sink: done with this sink, which exits */
    SP_SPEC = 27,        /* source -> sink, not resent: the first message of a
                            new pairing of a source whose HELLO has SP_F_SPEC.
                            a: the guest's GPU spec (the capset profile format) */
    SP_SPEC_REPLY = 28,  /* sink -> source, not resent. arg0 1: this renderer
                            can do the spec, 0: it cannot (the source closes);
                            a: text, what it cannot do */
    SP_GPU = 29,         /* sink -> source, the user's toggle. arg0 1: the local
                            GPU renders (on), 0: the renderer inside the guest
                            renders and this sink shows its pictures (off) */
    SP_STREAM = 30,      /* source -> sink (the renderer inside the guest).
                            arg0 1: send pictures of the scanout, 0: stop */
    SP_PICTURE = 31,     /* either direction, not resent. a: a JPEG of the
                            scanout; arg0, arg1 the scanout's size; arg2 the
                            renderer's count of its pictures */
    SP_GPU_STATE = 32,   /* source -> sink: what the toggle made. arg0 1 on, 0
                            off; arg1 1: it could not be turned off */
    SP_HOLD = 33,        /* source -> sink: the notice of the next command, what
                            it touches; with pieces (arg0 1) the command and its
                            memory writes come as a write in SP_PIECEs. See
                            "Large writes in pieces" in virtio-gpu-remote.c */
    SP_PIECE = 34,       /* source -> sink: a piece of such a write; arg0 its
                            notice's id, arg1 the offset of a in it */
    SP_RXB = 35,         /* sink -> source, not resent: arg0 the bytes of the
                            messages read on this connection after the source's
                            HELLO (the source paces its pieces by it) */
};

#define SP_XFER_MAX (16u << 20)

/*
 * The contents of one part of a resource. kind 0: the bytes of box at level in
 * the guest layout of the resource's format (stride and layer_stride, 0 =
 * packed), as virgl_renderer_transfer_write_iov() takes them with context 0.
 * Other kinds are made and read by sinks only (virtio-gpu-remote-state.c).
 */
typedef struct SpXfer {
    uint32_t res;
    uint32_t kind;
    uint32_t level;
    uint32_t stride;
    uint32_t layer_stride;
    uint32_t sample;
    uint32_t box[6];    /* x, y, z, w, h, d */
} QEMU_PACKED SpXfer;

/* virtio-gpu-remote.c */
void sp_sink_send(uint32_t type, const void *a, size_t a_len,
                  const void *b, size_t b_len,
                  uint64_t arg0, uint64_t arg1, uint64_t arg2);
void sp_sink_detach(void);
void sp_sink_send_once(uint32_t type, const void *a, size_t a_len,
                       uint64_t arg0, uint64_t arg1, uint64_t arg2);
size_t sp_sink_backlog(void);
void sp_sink_gpu(bool on);
/* the sink's log: the video pictures that a decoder's thread has not yet
 * handed over, at a transition (at: what happens) */
void sp_video_pending_log(const char *at);

/* virtio-gpu-remote-record.c: the record of video frames */
void sp_record_realize(void);
bool sp_record_control(const char *text);
void sp_record_cmd(struct virtio_gpu_ctrl_command *cmd);

/* called by ui/sdl2.c, which declares them itself */
void spacebox_gpu_toggle(void);
const char *spacebox_gpu_caption(unsigned *gen);

/* virtio-gpu-remote-picture.c; the first two are called by ui/spacebox-stream.h */
bool spacebox_stream_on(void);
void spacebox_stream_take(uint8_t *px, int w, int h, int stride, bool bottom_up,
                          int64_t read_us);
void sp_picture_stream(bool on);
void sp_picture_show(VirtIOGPU *g, bool off, uint64_t sw, uint64_t sh,
                     uint64_t n, const uint8_t *a, size_t len);
void sp_picture_end(void);

/* virtio-gpu-remote-state.c */
void sp_state_realize(VirtIOGPU *g);
void sp_state_forget(void);
void sp_state_wmark(uint64_t id, const void *a, size_t len);
void sp_state_attach(struct virtio_gpu_ctrl_command *cmd, uint64_t id);
void sp_state_queue(VirtIOGPU *g, uint32_t type, uint64_t id, uint64_t arg0,
                    const void *a, size_t a_len, const void *b, size_t b_len);
void sp_state_ctrl_seen(void);

/* virtio-gpu-remote.c: a command stream of the source, run before QEMU's virgl
 * code takes its command (which then submits nothing of it); true if the
 * renderer stopped it before a command that touches what waiting commands
 * hold: its rest waits, and the command is done with no answer */
bool sp_sink_submit_held(struct virtio_gpu_ctrl_command *cmd);
/* the control's "hold-log on|off": a line for each write in pieces when it runs */
void sp_sink_hold_log(bool on);

#endif
