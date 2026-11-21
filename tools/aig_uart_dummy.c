// AI-G side UART link test: sends dummy camera packets (14 bytes) at 90 Hz and
// checks the 5-byte echo packets [0xAA][0x56][seq u16][crc8] coming from the ESP32.
//
// Build on the AI-G:  gcc -O2 -o aig_uart_dummy aig_uart_dummy.c -lm
// Run:                ./aig_uart_dummy [/dev/ttyAMA1] [baud] [seconds]
// Never open /dev/ttyAMA2 (kernel console).

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static speed_t baudConst(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

static double nowSec(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/ttyAMA1";
  int baud = argc > 2 ? atoi(argv[2]) : 460800;
  int seconds = argc > 3 ? atoi(argv[3]) : 60;
  if (strstr(dev, "ttyAMA2")) { fprintf(stderr, "ttyAMA2 is the kernel console\n"); return 1; }
  speed_t sp = baudConst(baud);
  if (!sp) { fprintf(stderr, "unsupported baud %d\n", baud); return 1; }

  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror("open"); return 1; }
  struct termios tio;
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &tio);

  const double period = 1.0 / 90.0;
  double t0 = nowSec(), next = t0, lastRep = t0;
  uint16_t frame = 0;
  uint32_t sent = 0;

  // echo parser state
  uint8_t rb[5];
  size_t ri = 0;
  uint32_t echoOk = 0, echoCrc = 0, echoGaps = 0;
  uint16_t lastSeq = 0;
  int haveSeq = 0;

  while (nowSec() - t0 < seconds) {
    double now = nowSec();
    if (now >= next) {
      next += period;
      double t = now - t0;
      int16_t angle = (int16_t)lrint(sin(t) * 18000.0);  // +-180 deg sweep, 0.01 deg
      uint16_t x = (uint16_t)(3160 + 1000 * sin(t * 0.5)); // 0.1 px
      uint8_t p[14] = {0xAA, 0x55};
      p[2] = frame & 0xFF; p[3] = frame >> 8;
      p[4] = (uint16_t)angle & 0xFF; p[5] = (uint16_t)angle >> 8;
      p[6] = x & 0xFF; p[7] = x >> 8;
      p[8] = 2320 & 0xFF; p[9] = 2320 >> 8;
      p[10] = 0x2D;  // angle valid, mid marker, cart valid, age valid
      p[11] = 75 & 0xFF; p[12] = 0;  // 7.5 ms
      p[13] = crc8(p + 2, 11);
      ssize_t w = write(fd, p, sizeof p);
      if (w == (ssize_t)sizeof p) sent++;
      else if (w < 0 && errno != EAGAIN) perror("write");
      frame++;
    }

    uint8_t b;
    while (read(fd, &b, 1) == 1) {
      rb[ri++] = b;
      if (ri >= 1 && rb[0] != 0xAA) { ri = 0; continue; }
      if (ri >= 2 && rb[1] != 0x56) { ri = (b == 0xAA) ? (rb[0] = 0xAA, 1) : 0; continue; }
      if (ri == 5) {
        if (crc8(rb + 2, 2) == rb[4]) {
          uint16_t seq = rb[2] | (rb[3] << 8);
          if (haveSeq && (uint16_t)(seq - lastSeq) != 1) echoGaps++;
          lastSeq = seq; haveSeq = 1; echoOk++;
        } else echoCrc++;
        ri = 0;
      }
    }

    if (now - lastRep >= 1.0) {
      lastRep += 1.0;
      printf("t=%.0fs sent=%u | echo ok=%u crc_err=%u gaps=%u\n", now - t0, sent, echoOk, echoCrc, echoGaps);
      fflush(stdout);
    }
    usleep(500);
  }
  close(fd);
  return 0;
}
