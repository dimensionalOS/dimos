// The vendor toolchain, to check src/dock against: docker run -it ubuntu:20.04 (arm64), apt-get install g++ libpcl-dev
// (g++ 9.4.0-1ubuntu1~20.04.2, PCL 1.10, glibc 2.31: reflective_column_node's own).
// gcc -O2 -fno-builtin glibc_libm_fixtures.c -lm && ./a.out: tests/fixtures/glibc231_{atan2f,sinf}.bin (gzip them).
// glibc 2.31 (Ubuntu 20.04 arm64) atan2f / sinf on deterministic inputs: little-endian u32 triples / pairs
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
static uint64_t st = 0x9e3779b97f4a7c15ull;
static uint32_t rnd(void) { st ^= st << 13; st ^= st >> 7; st ^= st << 17; return (uint32_t)(st >> 32); }
static float uni(float lo, float hi) { return lo + (hi - lo) * (rnd() / 4294967296.0f); }
static uint32_t b(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
int main(void) {
  FILE *a = fopen("atan2f.bin", "wb"), *s = fopen("sinf.bin", "wb");
  for (int i = 0; i < 6000; i++) {
    float y, x;
    if (i % 4 == 0) { y = uni(0.3f, 4.0f); x = uni(-2.0f, 2.0f); }          // CalculateCenter: atan2f(x, y)
    else if (i % 4 == 1) { y = uni(-0.5f, 0.5f); x = uni(-0.5f, 0.5f); }    // FindTarget's centroid differences
    else { uint32_t u = rnd(), v = rnd(); memcpy(&y, &u, 4); memcpy(&x, &v, 4); if (!isfinite(y) || !isfinite(x)) { y = 1; x = 2; } }
    float r = atan2f(y, x);
    uint32_t t[3] = {b(y), b(x), b(r)};
    fwrite(t, 4, 3, a);
  }
  for (int i = 0; i < 9000; i++) {
    float v = i % 2 ? uni(0.0f, 3.1416f) : uni(-119.0f, 119.0f);
    if (i % 7 == 0) v = uni(-0.8f, 0.8f);
    float r = sinf(v);
    uint32_t t[2] = {b(v), b(r)};
    fwrite(t, 4, 2, s);
  }
  return 0;
}
