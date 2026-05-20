#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#include <sobel.h>
#ifdef __OR1300__
#include <cache.h>
#endif

/* ---------- Profile CI (ID 0xC) helpers ---------- */
static inline void profile_reset(void)
{
    asm volatile ("l.nios_rrr r0,r0,%[v],0xC" :: [v]"r"(7));
}
static inline void profile_read(uint32_t *cycles, uint32_t *stall, uint32_t *idle)
{
    asm volatile ("l.nios_rrr %[c],r0,%[v],0xC"
                  : [c]"=r"(*cycles) : [v]"r"((1 << 8) | (7 << 4)));
    asm volatile ("l.nios_rrr %[s],%[o],%[v],0xC"
                  : [s]"=r"(*stall)  : [o]"r"(1), [v]"r"(1 << 9));
    asm volatile ("l.nios_rrr %[i],%[o],%[v],0xC"
                  : [i]"=r"(*idle)   : [o]"r"(2), [v]"r"(1 << 10));
}
static inline void print_stage(const char *name, uint32_t c, uint32_t s, uint32_t i)
{
    printf("[%-10s] cycles=%u  stall=%u  idle=%u  work=%u\n",
           name, c, s, i, c - s);
}

/* ---------- Integer sqrt ---------- */
static uint32_t isqrt(uint32_t x)
{
    uint32_t r = 0;
    uint32_t b = 1U << 30;
    while (b > x) b >>= 2;
    while (b > 0) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return r;
}

/* ---------- Frame buffers ---------- */
volatile uint16_t rgb565[640 * 480];
volatile uint8_t  grayscale[640 * 480];
volatile uint8_t  sobelOut[640 * 480];
uint8_t           visited[640 * 480];

#define SOBEL_THRESHOLD    128

/* ---------- Detection workspace ---------- */
#define MAX_STACK         8000
#define MAX_BLOB_PIXELS   8000
#define MIN_BLOB_SIZE       40
#define MAX_BLOB_SIZE    20000

/* Threshold on (sigma/mu)*1000. Tune after measuring real circle + square. */
#define CIRCLE_THRESHOLD_MILLI 130

/* Reject blobs whose bbox is too elongated to be a square or circle */
#define MIN_ASPECT_RATIO_10X 5      /* 0.5 -> very oblong = reject */
#define MAX_ASPECT_RATIO_10X 20     /* 2.0 */

/* Reject blobs whose ratio is wildly outside expected range */
#define MAX_VALID_RATIO 400         /* nothing real produces this */
#define MIN_VALID_MU 20             /* too small/degenerate */

static int32_t stack_buf[MAX_STACK];
static int16_t blob_x[MAX_BLOB_PIXELS];
static int16_t blob_y[MAX_BLOB_PIXELS];

static int stack_top;
static inline void stk_push(int32_t v) { if (stack_top < MAX_STACK) stack_buf[stack_top++] = v; }
static inline int32_t stk_pop(void)    { return stack_buf[--stack_top]; }

static const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
static const int dy8[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

static int floodfill(int sx, int sy, int W, int H)
{
    int count = 0;
    stack_top = 0;
    stk_push(sy * W + sx);
    visited[sy * W + sx] = 1;

    while (stack_top > 0) {
        int32_t idx = stk_pop();
        int x = idx % W;
        int y = idx / W;

        if (count < MAX_BLOB_PIXELS) {
            blob_x[count] = (int16_t)x;
            blob_y[count] = (int16_t)y;
        }
        count++;

        for (int k = 0; k < 8; k++) {
            int nx = x + dx8[k];
            int ny = y + dy8[k];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            int nidx = ny * W + nx;
            if (visited[nidx]) continue;
            if (sobelOut[nidx] == 0) continue;
            visited[nidx] = 1;
            stk_push(nidx);
        }
    }
    return count;
}

static void classify_blob(int n, int blob_id)
{
    if (n > MAX_BLOB_PIXELS) n = MAX_BLOB_PIXELS;

    /* Bbox */
    int xmin = 32767, ymin = 32767, xmax = -1, ymax = -1;
    for (int i = 0; i < n; i++) {
        if (blob_x[i] < xmin) xmin = blob_x[i];
        if (blob_x[i] > xmax) xmax = blob_x[i];
        if (blob_y[i] < ymin) ymin = blob_y[i];
        if (blob_y[i] > ymax) ymax = blob_y[i];
    }
    int bbox_w = xmax - xmin + 1;
    int bbox_h = ymax - ymin + 1;

    /* Aspect ratio check (avoid float: compare 10*w/h against bounds) */
    int aspect10 = (bbox_w * 10) / bbox_h;
    if (aspect10 < MIN_ASPECT_RATIO_10X || aspect10 > MAX_ASPECT_RATIO_10X) {
        printf("blob %d rejected (aspect=%d/10)\n", blob_id, aspect10);
        return;
    }

    int32_t cx = (xmin + xmax) / 2;
    int32_t cy = (ymin + ymax) / 2;

    uint32_t sum_d  = 0;
    uint32_t sum_d2 = 0;
    for (int i = 0; i < n; i++) {
        int32_t dx = blob_x[i] - cx;
        int32_t dy = blob_y[i] - cy;
        uint32_t d2 = (uint32_t)(dx * dx + dy * dy);
        uint32_t d  = isqrt(d2);
        sum_d  += d;
        sum_d2 += d2;
    }

    uint32_t mu = sum_d / (uint32_t)n;
    if (mu < MIN_VALID_MU) {
        printf("blob %d rejected (mu=%u too small)\n", blob_id, mu);
        return;
    }
    uint32_t mean_d2 = sum_d2 / (uint32_t)n;
    uint32_t mu_sq   = mu * mu;
    uint32_t var     = (mean_d2 > mu_sq) ? (mean_d2 - mu_sq) : 0;
    uint32_t sigma   = isqrt(var);
    uint32_t ratio_milli = (sigma * 1000U) / mu;

    if (ratio_milli > MAX_VALID_RATIO) {
        printf("blob %d rejected (ratio=%u unreasonable)\n", blob_id, ratio_milli);
        return;
    }

    printf("blob %d\n", blob_id);
    printf("  n=%d\n", n);
    printf("  bbox x=%d..%d  y=%d..%d\n", xmin, xmax, ymin, ymax);
    printf("  mu=%u sigma=%u ratio=%u\n", mu, sigma, ratio_milli);
    if (ratio_milli < CIRCLE_THRESHOLD_MILLI) printf("  -> CIRCLE\n");
    else                                      printf("  -> SQUARE\n");
}

static void detect_shapes(int W, int H)
{
    for (int i = 0; i < W * H; i++) visited[i] = 0;

    int blob_id = 0;
    int total = 0;
    for (int y = 1; y < H - 1; y++) {
        for (int x = 1; x < W - 1; x++) {
            int idx = y * W + x;
            if (visited[idx]) continue;
            if (sobelOut[idx] == 0) continue;

            int n = floodfill(x, y, W, H);
            if (n < MIN_BLOB_SIZE) continue;
            if (n > MAX_BLOB_SIZE) continue;

            classify_blob(n, blob_id);
            blob_id++;
            total++;
            if (total >= 5) return;
        }
    }
    if (total == 0) printf("  (no shapes)\n");
}

int main(void)
{
    volatile unsigned int *vga = (unsigned int *) 0x50000020;
    camParameters camParams;
    uint32_t cycles, stall, idle, result;

#ifdef __OR1300__
    icache_write_cfg(CACHE_SIZE_8K | CACHE_FOUR_WAY | CACHE_REPLACE_LRU);
    dcache_write_cfg(CACHE_SIZE_8K | CACHE_FOUR_WAY | CACHE_WRITE_BACK | CACHE_REPLACE_PLRU);
    icache_enable(1);
    dcache_enable(1);
#endif
    vga_clear();

    printf("Initialising camera...\n");
    camParams = initOv7670(VGA);
    printf("Done! %d x %d @ %d FPS\n",
           camParams.nrOfPixelsPerLine,
           camParams.nrOfLinesPerImage,
           camParams.framesPerSecond);

    result = (camParams.nrOfPixelsPerLine <= 320)
             ? camParams.nrOfPixelsPerLine | 0x80000000 : camParams.nrOfPixelsPerLine;
    vga[0] = swap_u32(result);
    result = (camParams.nrOfLinesPerImage <= 240)
             ? camParams.nrOfLinesPerImage | 0x80000000 : camParams.nrOfLinesPerImage;
    vga[1] = swap_u32(result);

    vga[2] = swap_u32(2);
    vga[3] = swap_u32((uint32_t) &sobelOut[0]);

    const int W = camParams.nrOfPixelsPerLine;
    const int H = camParams.nrOfLinesPerImage;

    while (1) {
        profile_reset();
        takeSingleImageBlocking((uint32_t) &rgb565[0]);
        profile_read(&cycles, &stall, &idle);
        print_stage("capture", cycles, stall, idle);

        profile_reset();
        for (int line = 0; line < H; line++) {
            for (int pixel = 0; pixel < W; pixel++) {
                uint16_t rgb = swap_u16(rgb565[line * W + pixel]);
                uint32_t r = ((rgb >> 11) & 0x1F) << 3;
                uint32_t g = ((rgb >> 5)  & 0x3F) << 2;
                uint32_t b = ( rgb        & 0x1F) << 3;
                uint32_t gray = ((r * 54 + g * 183 + b * 19) >> 8) & 0xFF;
                grayscale[line * W + pixel] = (uint8_t)gray;
            }
        }
        profile_read(&cycles, &stall, &idle);
        print_stage("grayscale", cycles, stall, idle);

        profile_reset();
        edgeDetection(grayscale, sobelOut, W, H, SOBEL_THRESHOLD);
        profile_read(&cycles, &stall, &idle);
        print_stage("sobel", cycles, stall, idle);

        profile_reset();
        detect_shapes(W, H);
        profile_read(&cycles, &stall, &idle);
        print_stage("detect", cycles, stall, idle);

        printf("---\n");
    }
}