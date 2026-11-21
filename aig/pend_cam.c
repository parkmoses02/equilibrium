/*
 * pend_cam.c  --  inverted-pendulum marker tracker for the TOPST AI-G board
 *
 *   red marker  : pivot on the cart
 *   blue markers: rod middle + rod tip
 *
 * Captures UYVY frames from a V4L2 device (default /dev/video2, 632x464), finds the
 * markers directly in the YUV data (no RGB conversion), computes the rod angle and
 * logs everything to a CSV that is written when the run ends.
 *
 * Build on the board (if gcc exists):   gcc -O2 -o pend_cam pend_cam.c -lm
 * Cross-build on the PC (static):       aarch64-linux-gnu-gcc -O2 -static -o pend_cam pend_cam.c -lm
 *
 * Run (after ~/cam.sh 90):              ./pend_cam --vflip --frames 900 --out /dev/shm/track.csv
 * Offline test with a raw UYVY file:    ./pend_cam --file rec.raw --vflip --out t.csv
 * Fallback if direct capture fails:     v4l2-ctl -d /dev/video2 --stream-mmap --stream-count=300 --stream-to=- \
 *                                         | ./pend_cam --file - --vflip --out t.csv
 * Send every frame to the ESP32:        ./pend_cam --vflip --uart /dev/ttyAMA1 --baud 460800 --frames 100000
 *   (header pin 8 = TX, pin 10 = RX on /dev/ttyAMA1; needs root or dialout membership)
 *
 * UART packet, 14 bytes, little-endian, 8N1, no flow control:
 *   [0] 0xAA  [1] 0x55  [2..3] seq u16 (V4L2 frame sequence, wraps)
 *   [4..5] angle i16 in 0.01 deg, range -18000..18000, 0x8000 = invalid
 *   [6..7] pivot_x u16 in 0.1 px   [8..9] pivot_y u16 in 0.1 px   (0xFFFF = invalid)
 *   [10] flags: b0 angle valid, b1 tip used, b2 mid used, b3 pivot valid,
 *               b4 rod-radius calibration still running, b5 'age' valid
 *   [11..12] age u16 in 0.1 ms = time from the driver frame timestamp to the moment of sending
 *   [13] CRC-8 (poly 0x07, init 0) over bytes [2..12]
 *
 * Angle: 0 deg = rod straight up (after --vflip), + = leaning to the right of the image.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define MAXB 64
#define MAXIGN 8
#define CAL_MAX 200

typedef struct { double x, y; int n; } Blob;

typedef struct {
    int W, H, MW, stride;
    uint8_t *rm, *bm, *dil, *tmp;
    int32_t *lab, *par, *cnt;
    double *sx, *sy;
    /* color thresholds (YUV, 0..255) */
    int r_vmin, r_umax, r_diff;
    int b_umin, b_vmax, b_diff;
    int min_blue, min_red;          /* in macropixels (2 px wide) */
    int vflip;
    double tol, jump;
    int ign_n; double ign[MAXIGN][3];
    int cal_n;
    double fixLt, fixLm;
} Ctx;

typedef struct {
    int calibrated, nclean;
    double dt[CAL_MAX], dm[CAL_MAX];
    double Lt, Lm;
    double last_tip, last_mid, last_best, unw;
    int has_tip, has_mid, has_best;
    int since_tip, since_mid;
} State;

typedef struct {
    int frame; unsigned seq; double t;
    double px, py, tx, ty, mx, my, at, am, ang, unw;
    int tn, mn, flags;
    double proc_ms, lat_ms;
} Rec;

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

static double now_s(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double wrap180(double a) {
    a = fmod(a + 180.0, 360.0); if (a < 0) a += 360.0; return a - 180.0;
}

/* ---------- connected components on a (dilated) mask ---------- */
static int32_t find(int32_t *p, int32_t a) {
    while (p[a] != a) { p[a] = p[p[a]]; a = p[a]; }
    return a;
}

static int label_blobs(Ctx *c, const uint8_t *m, int min_area, Blob *out, int maxb) {
    int MW = c->MW, MH = c->H, N = MW * MH;
    /* dilate 3x3 : horizontal then vertical */
    for (int y = 0; y < MH; y++) {
        const uint8_t *s = m + y * MW; uint8_t *d = c->tmp + y * MW;
        for (int x = 0; x < MW; x++) {
            uint8_t v = s[x];
            if (x > 0) v |= s[x - 1];
            if (x < MW - 1) v |= s[x + 1];
            d[x] = v;
        }
    }
    for (int y = 0; y < MH; y++) {
        uint8_t *d = c->dil + y * MW;
        for (int x = 0; x < MW; x++) {
            uint8_t v = c->tmp[y * MW + x];
            if (y > 0) v |= c->tmp[(y - 1) * MW + x];
            if (y < MH - 1) v |= c->tmp[(y + 1) * MW + x];
            d[x] = v;
        }
    }
    memset(c->lab, 0, sizeof(int32_t) * N);
    int32_t next = 1; c->par[0] = 0;
    for (int y = 0; y < MH; y++) {
        for (int x = 0; x < MW; x++) {
            int i = y * MW + x;
            if (!c->dil[i]) continue;
            int32_t l = x > 0 ? c->lab[i - 1] : 0, u = y > 0 ? c->lab[i - MW] : 0;
            if (!l && !u) { c->lab[i] = next; c->par[next] = next; next++; }
            else if (l && !u) c->lab[i] = l;
            else if (!l && u) c->lab[i] = u;
            else {
                c->lab[i] = l;
                int32_t a = find(c->par, l), b = find(c->par, u);
                if (a != b) c->par[a < b ? b : a] = a < b ? a : b;
            }
        }
    }
    for (int k = 0; k < next; k++) { c->cnt[k] = 0; c->sx[k] = c->sy[k] = 0; }
    for (int y = 0; y < MH; y++) {
        for (int x = 0; x < MW; x++) {
            int i = y * MW + x;
            if (!m[i] || !c->lab[i]) continue;
            int32_t r = find(c->par, c->lab[i]);
            c->cnt[r]++; c->sx[r] += x; c->sy[r] += y;
        }
    }
    int nb = 0;
    for (int k = 1; k < next && nb < maxb; k++) {
        if (c->par[k] != k || c->cnt[k] < min_area) continue;
        out[nb].n = c->cnt[k] * 2;                      /* full-res pixels */
        out[nb].x = 2.0 * (c->sx[k] / c->cnt[k]) + 0.5; /* pair covers px 2mx, 2mx+1 */
        out[nb].y = c->sy[k] / c->cnt[k];
        nb++;
    }
    return nb;
}

/* ---------- per-frame detection ---------- */
static void classify(Ctx *c, const uint8_t *img) {
    int MW = c->MW;
    for (int y = 0; y < c->H; y++) {
        const uint8_t *p = img + (size_t)y * c->stride;
        uint8_t *r = c->rm + y * MW, *b = c->bm + y * MW;
        for (int x = 0; x < MW; x++) {
            int U = p[4 * x], Y = (p[4 * x + 1] + p[4 * x + 3]) >> 1, V = p[4 * x + 2];
            r[x] = (V >= c->r_vmin && U <= c->r_umax && V - U >= c->r_diff);
            b[x] = (U >= c->b_umin && V <= c->b_vmax && U - V >= c->b_diff && Y >= 15);
        }
    }
}

static int detect(Ctx *c, const uint8_t *img, Blob *piv, Blob *bl, int maxb) {
    classify(c, img);
    Blob rb[MAXB];
    int nr = label_blobs(c, c->rm, c->min_red, rb, MAXB);
    int havep = 0; int best = -1;
    for (int i = 0; i < nr; i++) if (best < 0 || rb[i].n > rb[best].n) best = i;
    if (best >= 0) { *piv = rb[best]; havep = 1; if (c->vflip) piv->y = (c->H - 1) - piv->y; }
    Blob bb[MAXB];
    int nbb = label_blobs(c, c->bm, c->min_blue, bb, MAXB);
    int nb = 0;
    for (int i = 0; i < nbb && nb < maxb; i++) {
        if (c->vflip) bb[i].y = (c->H - 1) - bb[i].y;
        int skip = 0;
        for (int k = 0; k < c->ign_n; k++)
            if (hypot(bb[i].x - c->ign[k][0], bb[i].y - c->ign[k][1]) < c->ign[k][2]) skip = 1;
        if (!skip) bl[nb++] = bb[i];
    }
    return havep ? nb : -1;   /* -1 = no pivot */
}

/* ---------- tracking logic (same idea as pend_track.py, but causal) ---------- */
static int cmpd(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y);
}
static double median(double *v, int n) { qsort(v, n, sizeof(double), cmpd); return v[n / 2]; }

static double ang_of(const Blob *p, const Blob *b) { return atan2(b->x - p->x, p->y - b->y) * 180.0 / M_PI; }

static void track(Ctx *c, State *s, const Blob *piv, const Blob *bl, int nb, Rec *r) {
    const Blob *tip = NULL, *mid = NULL;
    r->flags = 0;
    double d[MAXB];
    for (int i = 0; i < nb; i++) d[i] = hypot(bl[i].x - piv->x, bl[i].y - piv->y);

    if (!s->calibrated) {
        if (c->fixLt > 0 && c->fixLm > 0) { s->Lt = c->fixLt; s->Lm = c->fixLm; s->calibrated = 1; }
        else if (nb == 2) {                   /* clean frame: farthest = tip */
            int ti = d[0] > d[1] ? 0 : 1, mi = 1 - ti;
            if (d[mi] > 0.3 * d[ti]) {
                s->dt[s->nclean] = d[ti]; s->dm[s->nclean] = d[mi]; s->nclean++;
                tip = &bl[ti]; mid = &bl[mi]; r->flags |= 1;       /* pre-calibration */
                if (s->nclean >= c->cal_n) {
                    s->Lt = median(s->dt, s->nclean); s->Lm = median(s->dm, s->nclean);
                    s->calibrated = 1;
                    fprintf(stderr, "calibrated rod radius: tip %.1f px, middle %.1f px\n", s->Lt, s->Lm);
                }
            }
        }
    }
    if (s->calibrated && !(r->flags & 1)) {
        double bt = 1e9, bm = 1e9;
        for (int i = 0; i < nb; i++) {
            double et = fabs(d[i] - s->Lt) / s->Lt, em = fabs(d[i] - s->Lm) / s->Lm;
            if (et < c->tol && et < bt && et <= em) { tip = &bl[i]; bt = et; }
            if (em < c->tol && em < bm && em < et) { mid = &bl[i]; bm = em; }
        }
        if (tip && mid) {            /* must lie on the same ray from the pivot */
            double df = fabs(wrap180(ang_of(piv, tip) - ang_of(piv, mid)));
            if (df > 25) { if (bt <= bm) mid = NULL; else tip = NULL; }
        }
    }

    double at = NAN, am = NAN;
    if (tip) at = ang_of(piv, tip);
    if (mid) am = ang_of(piv, mid);
    /* causal outlier rejection */
    if (!isnan(at) && s->has_tip && s->since_tip < 10) {
        double lim = c->jump * (1 + s->since_tip); if (lim > 180) lim = 180;
        if (fabs(wrap180(at - s->last_tip)) > lim) { at = NAN; tip = NULL; }
    }
    if (!isnan(am) && s->has_mid && s->since_mid < 10) {
        double lim = c->jump * (1 + s->since_mid); if (lim > 180) lim = 180;
        if (fabs(wrap180(am - s->last_mid)) > lim) { am = NAN; mid = NULL; }
    }
    if (!isnan(at)) { s->last_tip = at; s->has_tip = 1; s->since_tip = 0; } else s->since_tip++;
    if (!isnan(am)) { s->last_mid = am; s->has_mid = 1; s->since_mid = 0; } else s->since_mid++;

    double best = !isnan(at) ? at : am;
    if (!isnan(best)) {
        if (s->has_best) s->unw += wrap180(best - s->last_best); else s->unw = best;
        s->last_best = best; s->has_best = 1;
    }
    r->px = piv->x; r->py = piv->y;
    r->tx = tip ? tip->x : NAN; r->ty = tip ? tip->y : NAN; r->tn = tip ? tip->n : 0;
    r->mx = mid ? mid->x : NAN; r->my = mid ? mid->y : NAN; r->mn = mid ? mid->n : 0;
    r->at = at; r->am = am; r->ang = best; r->unw = isnan(best) ? NAN : s->unw;
    if (tip) r->flags |= 4;
    if (mid) r->flags |= 8;
}

/* ---------- V4L2 capture ---------- */
typedef struct {
    int fd, mplane, nbuf, W, H, stride;
    void *mem[8]; size_t len[8];
    uint32_t type;
} Cam;

static int xioctl(int fd, unsigned long req, void *arg) {
    int r; do r = ioctl(fd, req, arg); while (r == -1 && errno == EINTR); return r;
}

static int cam_open(Cam *cm, const char *dev, int W, int H) {
    memset(cm, 0, sizeof(*cm));
    cm->fd = open(dev, O_RDWR | O_NONBLOCK);
    if (cm->fd < 0) { perror(dev); return -1; }
    struct v4l2_capability cap;
    if (xioctl(cm->fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("QUERYCAP"); return -1; }
    uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    cm->mplane = (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) && !(caps & V4L2_CAP_VIDEO_CAPTURE);
    cm->type = cm->mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fprintf(stderr, "%s: %s, %s\n", dev, cap.card, cm->mplane ? "multi-planar" : "single-planar");

    struct v4l2_format f; memset(&f, 0, sizeof(f)); f.type = cm->type;
    if (cm->mplane) {
        f.fmt.pix_mp.width = W; f.fmt.pix_mp.height = H;
        f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_UYVY; f.fmt.pix_mp.field = V4L2_FIELD_NONE;
    } else {
        f.fmt.pix.width = W; f.fmt.pix.height = H;
        f.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY; f.fmt.pix.field = V4L2_FIELD_NONE;
    }
    if (xioctl(cm->fd, VIDIOC_S_FMT, &f) < 0) { perror("S_FMT"); return -1; }
    uint32_t pf, bpl;
    if (cm->mplane) { cm->W = f.fmt.pix_mp.width; cm->H = f.fmt.pix_mp.height; pf = f.fmt.pix_mp.pixelformat;
                      bpl = f.fmt.pix_mp.plane_fmt[0].bytesperline; }
    else            { cm->W = f.fmt.pix.width; cm->H = f.fmt.pix.height; pf = f.fmt.pix.pixelformat;
                      bpl = f.fmt.pix.bytesperline; }
    cm->stride = bpl >= (uint32_t)cm->W * 2 ? (int)bpl : cm->W * 2;
    fprintf(stderr, "format: %dx%d fourcc=%.4s stride=%d\n", cm->W, cm->H, (char *)&pf, cm->stride);
    if (pf != V4L2_PIX_FMT_UYVY) { fprintf(stderr, "device did not accept UYVY\n"); return -1; }

    struct v4l2_requestbuffers rq; memset(&rq, 0, sizeof(rq));
    rq.count = 4; rq.type = cm->type; rq.memory = V4L2_MEMORY_MMAP;
    if (xioctl(cm->fd, VIDIOC_REQBUFS, &rq) < 0) { perror("REQBUFS"); return -1; }
    cm->nbuf = rq.count > 8 ? 8 : rq.count;
    for (int i = 0; i < cm->nbuf; i++) {
        struct v4l2_buffer b; struct v4l2_plane pl[VIDEO_MAX_PLANES];
        memset(&b, 0, sizeof(b)); memset(pl, 0, sizeof(pl));
        b.type = cm->type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (cm->mplane) { b.m.planes = pl; b.length = VIDEO_MAX_PLANES; }
        if (xioctl(cm->fd, VIDIOC_QUERYBUF, &b) < 0) { perror("QUERYBUF"); return -1; }
        size_t len = cm->mplane ? pl[0].length : b.length;
        off_t off = cm->mplane ? pl[0].m.mem_offset : b.m.offset;
        cm->len[i] = len;
        cm->mem[i] = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, cm->fd, off);
        if (cm->mem[i] == MAP_FAILED) { perror("mmap"); return -1; }
        if (len < (size_t)cm->stride * cm->H)
            fprintf(stderr, "warning: buffer %zu bytes < %d expected\n", len, cm->stride * cm->H);
    }
    return 0;
}

static int cam_qbuf(Cam *cm, int i) {
    struct v4l2_buffer b; struct v4l2_plane pl[VIDEO_MAX_PLANES];
    memset(&b, 0, sizeof(b)); memset(pl, 0, sizeof(pl));
    b.type = cm->type; b.memory = V4L2_MEMORY_MMAP; b.index = i;
    if (cm->mplane) { b.m.planes = pl; b.length = VIDEO_MAX_PLANES; }
    return xioctl(cm->fd, VIDIOC_QBUF, &b);
}

static int cam_start(Cam *cm) {
    for (int i = 0; i < cm->nbuf; i++) if (cam_qbuf(cm, i) < 0) { perror("QBUF"); return -1; }
    int t = cm->type;
    if (xioctl(cm->fd, VIDIOC_STREAMON, &t) < 0) { perror("STREAMON"); return -1; }
    return 0;
}

/* returns buffer index, fills seq/timestamp; -1 on timeout/error */
static int cam_dq(Cam *cm, unsigned *seq, double *ts_s, int *monotonic) {
    struct pollfd pfd = { cm->fd, POLLIN, 0 };
    int pr = poll(&pfd, 1, 2000);
    if (pr <= 0) { if (pr == 0) fprintf(stderr, "timeout waiting for a frame\n"); return -1; }
    struct v4l2_buffer b; struct v4l2_plane pl[VIDEO_MAX_PLANES];
    memset(&b, 0, sizeof(b)); memset(pl, 0, sizeof(pl));
    b.type = cm->type; b.memory = V4L2_MEMORY_MMAP;
    if (cm->mplane) { b.m.planes = pl; b.length = VIDEO_MAX_PLANES; }
    if (xioctl(cm->fd, VIDIOC_DQBUF, &b) < 0) { if (errno == EAGAIN) return -2; perror("DQBUF"); return -1; }
    *seq = b.sequence;
    *ts_s = b.timestamp.tv_sec + b.timestamp.tv_usec * 1e-6;
    *monotonic = (b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    return b.index;
}

/* ---------- UART output to the ESP32 ---------- */
#define PKT_LEN 14

static uint8_t crc8(const uint8_t *p, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static int uart_open(const char *dev, int baud) {
    speed_t sp;
    switch (baud) {
        case 115200: sp = B115200; break;
        case 230400: sp = B230400; break;
        case 460800: sp = B460800; break;
        case 921600: sp = B921600; break;
        case 1000000: sp = B1000000; break;
        case 1500000: sp = B1500000; break;
        case 2000000: sp = B2000000; break;
        default: fprintf(stderr, "unsupported baud %d\n", baud); return -1;
    }
    int fd = open(dev, O_WRONLY | O_NOCTTY);
    if (fd < 0) { perror(dev); return -1; }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) { perror("tcgetattr"); close(fd); return -1; }
    cfmakeraw(&t);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    cfsetospeed(&t, sp); cfsetispeed(&t, sp);
    if (tcsetattr(fd, TCSANOW, &t) < 0) { perror("tcsetattr"); close(fd); return -1; }
    return fd;
}

static int send_pkt(int ufd, const Rec *r, double age_ms, int age_ok) {
    uint8_t b[PKT_LEN];
    int16_t ang = INT16_MIN;
    uint16_t px = 0xFFFF, py = 0xFFFF, age = 0;
    int fl = 0;
    if (!isnan(r->ang)) {
        long v = lround(r->ang * 100.0);
        if (v > 18000) v = 18000;
        if (v < -18000) v = -18000;
        ang = (int16_t)v; fl |= 1;
    }
    if (r->flags & 4) fl |= 2;
    if (r->flags & 8) fl |= 4;
    if (!isnan(r->px)) {
        long x = lround(r->px * 10.0), y = lround(r->py * 10.0);
        px = (uint16_t)(x < 0 ? 0 : x > 65534 ? 65534 : x);
        py = (uint16_t)(y < 0 ? 0 : y > 65534 ? 65534 : y);
        fl |= 8;
    }
    if (r->flags & 1) fl |= 16;
    if (age_ok) { long a = lround(age_ms * 10.0); age = (uint16_t)(a < 0 ? 0 : a > 65535 ? 65535 : a); fl |= 32; }
    b[0] = 0xAA; b[1] = 0x55;
    b[2] = r->seq & 0xFF; b[3] = (r->seq >> 8) & 0xFF;
    b[4] = (uint16_t)ang & 0xFF; b[5] = ((uint16_t)ang >> 8) & 0xFF;
    b[6] = px & 0xFF; b[7] = px >> 8; b[8] = py & 0xFF; b[9] = py >> 8;
    b[10] = (uint8_t)fl;
    b[11] = age & 0xFF; b[12] = age >> 8;
    b[13] = crc8(b + 2, 11);
    return write(ufd, b, PKT_LEN) == PKT_LEN ? 0 : -1;
}

/* ---------- main ---------- */
static void nanstr(FILE *f, double v, const char *fmt) { if (isnan(v)) fputc(',', f); else { fputc(',', f); fprintf(f, fmt, v); } }

static int cmpd2(const void *a, const void *b) { return cmpd(a, b); }

int main(int argc, char **argv) {
    const char *dev = "/dev/video2", *out = "/dev/shm/track.csv", *file = NULL, *dump = NULL;
    int W = 632, H = 464, frames = 900, dump_n = 3; double fps = 90;
    const char *uartdev = NULL; int baud = 460800, pace = 0, ufd = -1;
    unsigned long uart_pkts = 0, uart_err = 0;
    Ctx c; memset(&c, 0, sizeof(c));
    c.r_vmin = 148; c.r_umax = 130; c.r_diff = 30;
    c.b_umin = 150; c.b_vmax = 120; c.b_diff = 50;
    c.min_blue = 30; c.min_red = 10; c.tol = 0.30; c.jump = 45; c.cal_n = 30;

    for (int i = 1; i < argc; i++) {
        #define ARG(s) (!strcmp(argv[i], s))
        #define NEXT() (++i < argc ? argv[i] : (fprintf(stderr, "missing value for %s\n", argv[i-1]), exit(2), (char*)0))
        if (ARG("--dev")) dev = NEXT();
        else if (ARG("--width")) W = atoi(NEXT());
        else if (ARG("--height")) H = atoi(NEXT());
        else if (ARG("--frames")) frames = atoi(NEXT());
        else if (ARG("--out")) out = NEXT();
        else if (ARG("--file")) file = NEXT();
        else if (ARG("--fps")) fps = atof(NEXT());
        else if (ARG("--vflip")) c.vflip = 1;
        else if (ARG("--dump")) dump = NEXT();
        else if (ARG("--dump-count")) dump_n = atoi(NEXT());
        else if (ARG("--uart")) uartdev = NEXT();
        else if (ARG("--baud")) baud = atoi(NEXT());
        else if (ARG("--pace")) pace = 1;   /* file mode: replay at --fps instead of as fast as possible */
        else if (ARG("--red")) sscanf(NEXT(), "%d,%d,%d", &c.r_vmin, &c.r_umax, &c.r_diff);
        else if (ARG("--blue")) sscanf(NEXT(), "%d,%d,%d", &c.b_umin, &c.b_vmax, &c.b_diff);
        else if (ARG("--min-blue")) c.min_blue = atoi(NEXT());
        else if (ARG("--min-red")) c.min_red = atoi(NEXT());
        else if (ARG("--tol")) c.tol = atof(NEXT());
        else if (ARG("--jump")) c.jump = atof(NEXT());
        else if (ARG("--cal")) { c.cal_n = atoi(NEXT()); if (c.cal_n > CAL_MAX) c.cal_n = CAL_MAX; if (c.cal_n < 5) c.cal_n = 5; }
        else if (ARG("--Lt")) c.fixLt = atof(NEXT());
        else if (ARG("--Lm")) c.fixLm = atof(NEXT());
        else if (ARG("--ignore")) {
            if (c.ign_n < MAXIGN) { double *g = c.ign[c.ign_n]; if (sscanf(NEXT(), "%lf,%lf,%lf", &g[0], &g[1], &g[2]) == 3) c.ign_n++; }
        }
        else { fprintf(stderr, "unknown option %s (see the header of pend_cam.c)\n", argv[i]); return 2; }
    }

    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    Cam cam; memset(&cam, 0, sizeof(cam)); cam.fd = -1;
    FILE *fin = NULL;
    if (file) {
        fin = strcmp(file, "-") ? fopen(file, "rb") : stdin;
        if (!fin) { perror(file); return 1; }
        c.stride = W * 2;
    } else {
        if (cam_open(&cam, dev, W, H) < 0) return 1;
        W = cam.W; H = cam.H; c.stride = cam.stride;
    }
    c.W = W; c.H = H; c.MW = W / 2;
    int N = c.MW * H;
    c.rm = malloc(N); c.bm = malloc(N); c.dil = malloc(N); c.tmp = malloc(N);
    c.lab = malloc(sizeof(int32_t) * N); c.par = malloc(sizeof(int32_t) * (N / 2 + 8));
    c.cnt = malloc(sizeof(int32_t) * (N / 2 + 8));
    c.sx = malloc(sizeof(double) * (N / 2 + 8)); c.sy = malloc(sizeof(double) * (N / 2 + 8));
    uint8_t *fbuf = file ? malloc((size_t)c.stride * H) : NULL;
    Rec *rec = calloc(frames, sizeof(Rec));
    State st; memset(&st, 0, sizeof(st));
    FILE *fd = dump ? fopen(dump, "wb") : NULL;

    if (!file && cam_start(&cam) < 0) return 1;
    if (uartdev) {
        ufd = uart_open(uartdev, baud);
        if (ufd < 0) return 1;
        fprintf(stderr, "UART output: %s at %d baud, %d-byte packets (%.2f ms each)\n",
                uartdev, baud, PKT_LEN, PKT_LEN * 10.0 * 1000.0 / baud);
    }

    int n = 0, dropped = 0, nodet = 0; unsigned prev_seq = 0; int have_prev = 0;
    double t0 = -1, wall0 = now_s();
    while (n < frames && !g_stop) {
        const uint8_t *img; int bi = -1; unsigned seq = n; double ts = n / fps; int mono = 0;
        if (file && pace) { double d = wall0 + n / fps - now_s(); if (d > 0) usleep((useconds_t)(d * 1e6)); }
        if (file) {
            if (fread(fbuf, 1, (size_t)c.stride * H, fin) != (size_t)c.stride * H) break;
            img = fbuf;
        } else {
            bi = cam_dq(&cam, &seq, &ts, &mono);
            if (bi == -2) continue;
            if (bi < 0) break;
            img = cam.mem[bi];
        }
        double ta = now_s();
        if (fd && n < dump_n) fwrite(img, 1, (size_t)c.stride * H, fd);
        if (t0 < 0) t0 = ts;
        Blob piv = {0, 0, 0}, bl[MAXB];
        int nb = detect(&c, img, &piv, bl, MAXB);
        Rec *r = &rec[n]; memset(r, 0, sizeof(*r));
        r->frame = n; r->seq = seq; r->t = ts - t0;
        if (nb < 0) {
            r->flags = 2; r->px = r->py = r->tx = r->ty = r->mx = r->my = r->at = r->am = r->ang = r->unw = NAN;
            nodet++;
        } else track(&c, &st, &piv, bl, nb, r);
        double tb = now_s();
        r->proc_ms = (tb - ta) * 1e3;
        r->lat_ms = mono ? (ta - ts) * 1e3 : NAN;
        if (ufd >= 0) {
            if (send_pkt(ufd, r, mono ? (now_s() - ts) * 1e3 : 0, mono) == 0) uart_pkts++; else uart_err++;
        }
        if (!file) {
            if (have_prev && seq != prev_seq + 1) dropped += (int)(seq - prev_seq - 1);
            prev_seq = seq; have_prev = 1;
            cam_qbuf(&cam, bi);
        }
        n++;
    }
    double wall = now_s() - wall0;
    if (fd) fclose(fd);
    if (!file && cam.fd >= 0) { int t = cam.type; xioctl(cam.fd, VIDIOC_STREAMOFF, &t); }

    FILE *fo = fopen(out, "w");
    if (!fo) { perror(out); return 1; }
    fprintf(fo, "frame,seq,t_s,pivot_x,pivot_y,tip_x,tip_y,tip_n,mid_x,mid_y,mid_n,angle_tip_deg,angle_mid_deg,angle_deg,angle_unwrapped_deg,proc_ms,lat_ms,flags\n");
    for (int i = 0; i < n; i++) {
        Rec *r = &rec[i];
        fprintf(fo, "%d,%u,%.5f", r->frame, r->seq, r->t);
        nanstr(fo, r->px, "%.2f"); nanstr(fo, r->py, "%.2f");
        nanstr(fo, r->tx, "%.2f"); nanstr(fo, r->ty, "%.2f"); fprintf(fo, ",%d", r->tn);
        nanstr(fo, r->mx, "%.2f"); nanstr(fo, r->my, "%.2f"); fprintf(fo, ",%d", r->mn);
        nanstr(fo, r->at, "%.2f"); nanstr(fo, r->am, "%.2f"); nanstr(fo, r->ang, "%.2f"); nanstr(fo, r->unw, "%.2f");
        nanstr(fo, r->proc_ms, "%.3f"); nanstr(fo, r->lat_ms, "%.2f"); fprintf(fo, ",%d\n", r->flags);
    }
    fclose(fo);

    /* summary */
    int ntip = 0, nmid = 0, nnone = 0, nboth = 0;
    double *pm = malloc(sizeof(double) * (n + 1)), *dd = malloc(sizeof(double) * (n + 1));
    double psum = 0, pmax = 0, lsum = 0; int nl = 0;
    for (int i = 0; i < n; i++) {
        Rec *r = &rec[i];
        if (!isnan(r->at)) ntip++;
        if (!isnan(r->am)) nmid++;
        if (isnan(r->ang)) nnone++;
        if (!isnan(r->at) && !isnan(r->am)) dd[nboth++] = fabs(wrap180(r->at - r->am));
        pm[i] = r->proc_ms; psum += r->proc_ms; if (r->proc_ms > pmax) pmax = r->proc_ms;
        if (!isnan(r->lat_ms)) { lsum += r->lat_ms; nl++; }
    }
    qsort(pm, n ? n : 1, sizeof(double), cmpd2);
    fprintf(stderr, "\n== summary ==\nframes: %d in %.2f s (%.1f fps)%s\n", n, wall, n / (wall > 0 ? wall : 1),
            file ? " [file mode: not real time]" : "");
    if (!file) fprintf(stderr, "dropped frames (sequence gaps): %d\n", dropped);
    fprintf(stderr, "tip found %d/%d, middle found %d/%d, no angle %d, no pivot %d\n", ntip, n, nmid, n, nnone, nodet);
    if (nboth) { qsort(dd, nboth, sizeof(double), cmpd2);
        fprintf(stderr, "tip-vs-middle angle difference: median %.2f deg, p95 %.2f deg (n=%d)\n", dd[nboth / 2], dd[(int)(nboth * 0.95)], nboth); }
    if (n) fprintf(stderr, "processing time per frame: mean %.2f ms, median %.2f ms, max %.2f ms  (budget at %.0f fps = %.1f ms)\n",
                   psum / n, pm[n / 2], pmax, fps, 1000.0 / fps);
    if (nl) fprintf(stderr, "capture-to-processing latency (driver timestamp -> start of processing): mean %.2f ms\n", lsum / nl);
    if (ufd >= 0) fprintf(stderr, "UART packets sent: %lu, write errors: %lu\n", uart_pkts, uart_err);
    fprintf(stderr, "wrote %s\n", out);
    return 0;
}
