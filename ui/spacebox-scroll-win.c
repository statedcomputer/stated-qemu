/*
 * The scroll channel on Windows: the counterpart of spacebox-scroll.m. The
 * Mac's takes AppKit's scroll events; this one takes SDL's wheel events
 * (ui/sdl2.c, handle_mousewheel), with the precise deltas that Windows gives
 * (a precision touchpad's are not whole notches of WHEEL_DELTA), and sends the
 * guest's scroll agent the same lines on the same channel: "sequence host_us dx
 * dy precise phase momentum", in points (Chromium's 40 points for a notch),
 * through SP_SCROLL in the remote GPU's stream or the socket
 * SPACEBOX_SCROLL_SOCKET (a Unix socket's path, or HOST:PORT for TCP), after
 * the agent has answered a PING. Windows gives no phase and no momentum: 0.
 * The fixture SPACEBOX_SCROLL_TEST takes the Mac's six fields "x y precise
 * phase momentum [inverted]" (as AppKit's scrollingDelta, or delta when not
 * precise) and goes the same way; SPACEBOX_SCROLL_TRACE as on the Mac.
 */
#include "qemu/osdep.h"
#include "qemu/sockets.h"
#include <SDL.h>
#include <math.h>
#include "ui/spacebox-hook.h"

extern int64_t spacebox_last_input_us;
/* hw/display/virtio-gpu-remote.c: the scroll channel inside the remote GPU's
 * stream, when the source asked for it (main thread, like this file). */
extern int64_t spacebox_scroll_answer_us;
bool spacebox_remote_scroll_stream(uint64_t *conn);
bool spacebox_remote_scroll_send(const void *buf, size_t len);
/* hw/display/virtio-gpu-remote.c: the link's sockets */
bool sp_addr_is_tcp(const char *a);
int sp_sock_connect(const char *a);
void sp_sock_tune(int fd);
void spacebox_scroll_tick(SDL_Window *window);
bool spacebox_scroll_wheel(const SDL_MouseWheelEvent *wev);

/* A scroll event as AppKit gives one (its signs): scrollingDelta in points,
 * delta in notches. */
typedef struct SpScroll {
    double sx, sy, dx, dy;
    bool precise, inverted;
    unsigned long phase, momentum;
} SpScroll;

static FILE *sp_trace;
static uint64_t sp_event_id;
static void sp_log(const SpScroll *e, const char *origin)
{
    const char *path = getenv("SPACEBOX_SCROLL_TRACE");
    if (!path) return;
    if (!sp_trace) { sp_trace = fopen(path, "a"); if (sp_trace) setvbuf(sp_trace,NULL,_IOLBF,0); }
    if (sp_trace) fprintf(sp_trace,"{\"stage\":\"native\",\"id\":%llu,\"host_us\":%lld,\"origin\":\"%s\",\"x\":%.9g,\"y\":%.9g,\"delta_x\":%.9g,\"delta_y\":%.9g,\"precise\":%d,\"phase\":%lu,\"momentum\":%lu,\"inverted\":%d}\n",(unsigned long long)++sp_event_id,(long long)g_get_monotonic_time(),origin,e->sx,e->sy,e->dx,e->dy,e->precise,e->phase,e->momentum,e->inverted);
}
static int sp_fd=-1;
static bool sp_ready;
static char sp_pending[65536];
static size_t sp_pending_len;
static int64_t sp_retry, sp_ping, sp_last_ready;
static uint64_t sp_sequence;
static bool sp_on_stream;
static uint64_t sp_conn;
static int64_t sp_answer_seen;
static bool sp_connecting;

/*
 * The connect to the socket, without a wait: on Windows a TCP connect to a
 * port where nobody listens takes two seconds to fail, which would stop the
 * main loop each retry (the Mac's connect, to a Unix socket, is non-blocking
 * too). -1, or a socket that sp_connected() then follows.
 */
static int sp_connect_start(const char *path)
{
    g_autofree char *host = NULL;
    const char *port = strrchr(path, ':');
    struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM }, *ai = NULL;
    int fd;

    if (!sp_addr_is_tcp(path)) {
        fd = sp_sock_connect(path);   /* a Unix socket answers at once */
        sp_connecting = false;
        return fd;
    }
    host = g_strndup(path, port - path);
    if (getaddrinfo(host, port + 1, &hints, &ai) || !ai) {
        return -1;
    }
    fd = qemu_socket(ai->ai_family, SOCK_STREAM, 0);
    if (fd >= 0) {
        qemu_socket_set_nonblock(fd);
        if (connect(fd, ai->ai_addr, ai->ai_addrlen) < 0 && errno != EINPROGRESS &&
            errno != EWOULDBLOCK && errno != EAGAIN) {
            close(fd);
            fd = -1;
        }
    }
    freeaddrinfo(ai);
    sp_connecting = fd >= 0;
    return fd;
}

/* 1: connected, 0: not yet, -1: refused */
static int sp_connected(int fd)
{
    SOCKET s = (SOCKET)_get_osfhandle(fd);
    fd_set w, e;
    struct timeval tv = { 0, 0 };

    FD_ZERO(&w);
    FD_ZERO(&e);
    FD_SET(s, &w);
    FD_SET(s, &e);
    if (select(0, NULL, &w, &e, &tv) == SOCKET_ERROR || FD_ISSET(s, &e)) {
        return -1;
    }
    return FD_ISSET(s, &w) ? 1 : 0;
}
/* The source carries the channel in the stream: PING on each new connection
 * and every second; any SP_SCROLL back is the agent's answer. */
static void sp_stream_poll(uint64_t conn)
{
    int64_t now=g_get_monotonic_time();
    if(conn && (conn!=sp_conn || now-sp_ping>1000000)) { spacebox_remote_scroll_send("PING\n",5);sp_ping=now; }
    sp_conn=conn;
    if(spacebox_scroll_answer_us!=sp_answer_seen) {
        sp_answer_seen=spacebox_scroll_answer_us;
        if(!sp_ready) fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=1 path=stream\n");
        sp_ready=true;sp_last_ready=sp_answer_seen;
    }
    if(sp_ready && now-sp_last_ready>3000000) { sp_ready=false;fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=0 timeout=1 path=stream\n"); }
}
static void sp_bridge_poll(void)
{
    uint64_t conn;
    bool stream=spacebox_remote_scroll_stream(&conn);
    if(stream!=sp_on_stream) {
        /* Another kind of source: the other path starts not ready. */
        sp_on_stream=stream;sp_ready=false;sp_pending_len=0;sp_conn=0;
        if(sp_fd>=0) { close(sp_fd);sp_fd=-1;sp_connecting=false; }
    }
    if(stream) { sp_stream_poll(conn);return; }
    const char *path=getenv("SPACEBOX_SCROLL_SOCKET");
    if (!path) return;
    if (sp_fd<0 && g_get_monotonic_time()>=sp_retry) {
        sp_retry=g_get_monotonic_time()+250000;
        sp_fd=sp_connect_start(path);
        if(sp_fd<0)return;
        if (!sp_connecting) {
            sp_sock_tune(sp_fd);
            qemu_socket_set_nonblock(sp_fd);
            send(sp_fd,"PING\n",5,0);
        }
    }
    if(sp_fd<0)return;
    if (sp_connecting) {
        int c = sp_connected(sp_fd);

        if (c < 0) { close(sp_fd);sp_fd=-1;sp_connecting=false;return; }
        if (c == 0) {
            if (g_get_monotonic_time()>=sp_retry) { close(sp_fd);sp_fd=-1;sp_connecting=false; }
            return;
        }
        sp_connecting=false;
        sp_sock_tune(sp_fd);
        send(sp_fd,"PING\n",5,0);
    }
    char reply[128];ssize_t n=recv(sp_fd,reply,sizeof(reply),0);
    if(n>0) {
        if(!sp_ready) fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=1 path=socket\n");
        sp_ready=true;sp_last_ready=g_get_monotonic_time();
    }
    else if(n==0 || (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK)) { close(sp_fd);sp_fd=-1;sp_ready=false;sp_pending_len=0;return; }
    int64_t now=g_get_monotonic_time();
    if(now-sp_ping>1000000) { send(sp_fd,"PING\n",5,0);sp_ping=now; }
    if(sp_ready && now-sp_last_ready>3000000) { sp_ready=false;fprintf(stderr,"[SPACEBOX-SCROLL] bridge_ready=0 timeout=1 path=socket\n"); }
    if(sp_pending_len) {
        ssize_t wrote=send(sp_fd,sp_pending,sp_pending_len,0);
        if(wrote>0) { memmove(sp_pending,sp_pending+wrote,sp_pending_len-wrote);sp_pending_len-=wrote; }
        else if(wrote<0 && errno!=EAGAIN && errno!=EWOULDBLOCK) { close(sp_fd);sp_fd=-1;sp_ready=false;sp_pending_len=0; }
    }
}
static bool sp_route(const SpScroll *e, const char *origin)
{
    sp_log(e,origin);
    const char *bypass=getenv("SPACEBOX_SCROLL_BYPASS");
    if(bypass && !access(bypass,F_OK))return false;
    sp_bridge_poll();
    if(!sp_on_stream && !getenv("SPACEBOX_SCROLL_SOCKET"))return false;
    if(!sp_ready)return false;
    /* As the Mac's: point deltas, and Chromium's 40 points for a notch. */
    double x=e->precise ? -e->sx : -40.0*e->dx;
    double y=e->precise ? -e->sy : -40.0*e->dy;
    spacebox_last_input_us=g_get_monotonic_time(); /* hw/display/virtio-gpu-remote.c */
    char line[192];int n=snprintf(line,sizeof(line),"%llu %lld %.9g %.9g %d %lu %lu\n",(unsigned long long)++sp_sequence,(long long)g_get_monotonic_time(),x,y,e->precise,e->phase,e->momentum);
    if(sp_on_stream) { if(n>0 && n<sizeof(line)) spacebox_remote_scroll_send(line,n); }
    else if(n>0 && n<sizeof(line) && sp_pending_len+n<=sizeof(sp_pending)) {
        memcpy(sp_pending+sp_pending_len,line,n);sp_pending_len+=n;sp_bridge_poll();
    } else fprintf(stderr,"[SPACEBOX-SCROLL-ERROR] bounded queue overflow\n");
    return true; /* Exactly one path; the caller drops SDL's quantized wheel. */
}

/* the fixture's own wheel event, sent to SDL when it did not go to the agent */
#define SP_FIXTURE_MOUSEID 0x5350

/* precise: not whole notches of WHEEL_DELTA (a precision touchpad's) */
static bool sp_precise(float v)
{
    double u = round(v * 120.0);

    return fabs(v * 120.0 - u) > 1e-3 || fmod(fabs(u), 120.0) != 0;
}

/*
 * ui/sdl2.c: a wheel event of the window; true when it went to the agent (the
 * caller then sends no wheel buttons). SDL's precise deltas are in notches
 * (WHEEL_DELTA, 120, is 1.0), y up and x right positive; AppKit's delta is y
 * up and x left positive, and scrollingDelta is in points.
 */
bool spacebox_scroll_wheel(const SDL_MouseWheelEvent *wev)
{
    double flip = wev->direction == SDL_MOUSEWHEEL_FLIPPED ? -1 : 1;
    SpScroll e = { 0 };

    if (wev->which == SP_FIXTURE_MOUSEID || wev->which == SDL_TOUCH_MOUSEID) {
        /* the fixture's, routed already; SDL's mouse made of touches */
        return false;
    }
    e.dx = -wev->preciseX * flip;
    e.dy = wev->preciseY * flip;
    e.sx = 40.0 * e.dx;
    e.sy = 40.0 * e.dy;
    e.precise = sp_precise(wev->preciseX) || sp_precise(wev->preciseY);
    e.inverted = wev->direction == SDL_MOUSEWHEEL_FLIPPED;
    return sp_route(&e, "device");
}

void spacebox_scroll_tick(SDL_Window *window)
{
    static bool said;
    if (!window) return;
    if (!said) {
        said = true;
        fprintf(stderr,"[SPACEBOX-SCROLL] sdl_wheel=1 no_system_event_tap=1\n");
    }
    sp_bridge_poll();
    const char *path=getenv("SPACEBOX_SCROLL_TEST");
    if (!path) return;
    g_autofree char *text = NULL;
    if (!spacebox_hook_take(path, &text)) return;
    double x=0,y=0; int precise=1,phase=4,momentum=0,inverted=1;
    int n=sscanf(text,"%lf %lf %d %d %d %d",&x,&y,&precise,&phase,&momentum,&inverted);
    if(n<5)return;
    /* the Mac's fixture: scrollingDelta x y, and delta a tenth of it when precise */
    SpScroll e = { .sx = x, .sy = y, .dx = precise ? x / 10.0 : x, .dy = precise ? y / 10.0 : y,
                   .precise = precise, .inverted = inverted, .phase = phase, .momentum = momentum };
    if (!sp_route(&e,"fixture")) {
        /* as the Mac's goes to SDL's own listener: a wheel event of the window */
        SDL_Event ev = { 0 };
        ev.type = SDL_MOUSEWHEEL;
        ev.wheel.windowID = SDL_GetWindowID(window);
        ev.wheel.which = SP_FIXTURE_MOUSEID;
        ev.wheel.preciseX = -e.dx;
        ev.wheel.preciseY = e.dy;
        ev.wheel.x = (Sint32)ev.wheel.preciseX;
        ev.wheel.y = (Sint32)ev.wheel.preciseY;
        ev.wheel.direction = SDL_MOUSEWHEEL_NORMAL;
        SDL_PushEvent(&ev);
    }
}
