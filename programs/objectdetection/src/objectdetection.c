/* streaming_sobel.c
 *
 * Camera -> HW grayscale (autonomous) -> HW sobel (CI 11) -> SW detection.
 */

#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#ifdef __OR1300__
#include <cache.h>
#endif

/* ================ Sobel CI ================ */
#define SOB_OP_FEED      0U
#define SOB_OP_READ_OUT  1U
#define SOB_OP_STATUS    2U
#define SOB_OP_RESET     3U
#define SOB_OP_SET_THR   4U
#define SOB_OP_SET_WIDTH 5U
#define SOB_OP_ACK_ROW   6U

#define SOBEL_THRESHOLD  128U
#define FRAME_WIDTH      640U
#define FRAME_HEIGHT     480U
#define WORDS_PER_ROW    (FRAME_WIDTH / 4U)

static inline void sob_feed(uint32_t four_pixels) {
    asm volatile ("l.nios_rrr r0,%[a],%[b],11"
                  :: [a]"r"(SOB_OP_FEED), [b]"r"(four_pixels));
}
static inline uint32_t sob_read_out(uint32_t word_idx) {
    uint32_t v;
    asm volatile ("l.nios_rrr %[v],%[a],%[b],11"
                  : [v]"=r"(v) : [a]"r"(SOB_OP_READ_OUT), [b]"r"(word_idx));
    return v;
}
static inline void sob_reset(void)       { asm volatile ("l.nios_rrr r0,%[a],r0,11" :: [a]"r"(SOB_OP_RESET)); }
static inline void sob_set_thr(uint32_t t)  { asm volatile ("l.nios_rrr r0,%[a],%[b],11" :: [a]"r"(SOB_OP_SET_THR), [b]"r"(t)); }
static inline void sob_set_width(uint32_t w){ asm volatile ("l.nios_rrr r0,%[a],%[b],11" :: [a]"r"(SOB_OP_SET_WIDTH), [b]"r"(w)); }
static inline void sob_ack_row(void)     { asm volatile ("l.nios_rrr r0,%[a],r0,11" :: [a]"r"(SOB_OP_ACK_ROW)); }

/* ================ Profile CI ================ */
static inline void profile_reset(void) {
    asm volatile ("l.nios_rrr r0,r0,%[v],0xC" :: [v]"r"(7));
}
static inline void profile_read(uint32_t *cyc, uint32_t *st, uint32_t *id) {
    asm volatile ("l.nios_rrr %[c],r0,%[v],0xC"  : [c]"=r"(*cyc) : [v]"r"((1<<8)|(7<<4)));
    asm volatile ("l.nios_rrr %[s],%[o],%[v],0xC": [s]"=r"(*st)  : [o]"r"(1), [v]"r"(1<<9));
    asm volatile ("l.nios_rrr %[i],%[o],%[v],0xC": [i]"=r"(*id)  : [o]"r"(2), [v]"r"(1<<10));
}
static inline void print_stage(const char *n, uint32_t c, uint32_t s, uint32_t i) {
    printf("[%-10s] cycles=%u stall=%u idle=%u work=%u\n", n, c, s, i, c - s);
}

/* ================ Integer sqrt ================ */
static uint32_t isqrt(uint32_t x) {
    uint32_t r = 0, b = 1U << 30;
    while (b > x) b >>= 2;
    while (b > 0) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; }
        else            { r >>= 1; }
        b >>= 2;
    }
    return r;
}

/* ================ Frame buffers ================ */
volatile uint16_t rgb565[640 * 480];
volatile uint8_t  grayscale[640 * 480];
volatile uint8_t  sobelOut[640 * 480];
uint8_t           visited[640 * 480];

/* ================ Detection params ================ */
#define MAX_STACK              8000
#define MAX_BLOB_PIXELS        8000
#define MIN_BLOB_SIZE            40
#define MAX_BLOB_SIZE         20000
#define CIRCLE_THRESHOLD_MILLI  130
#define MIN_ASPECT_RATIO_10X      5
#define MAX_ASPECT_RATIO_10X     20
#define MAX_VALID_RATIO         400
#define MIN_VALID_MU             20

static int32_t stack_buf[MAX_STACK];
static int16_t blob_x[MAX_BLOB_PIXELS];
static int16_t blob_y[MAX_BLOB_PIXELS];

static int stack_top;
static inline void stk_push(int32_t v) { if (stack_top < MAX_STACK) stack_buf[stack_top++] = v; }
static inline int32_t stk_pop(void)    { return stack_buf[--stack_top]; }

static const int dx8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
static const int dy8[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

static int floodfill(int sx, int sy, int W, int H) {
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

static void classify_blob(int n, int blob_id) {
    if (n > MAX_BLOB_PIXELS) n = MAX_BLOB_PIXELS;

    int xmin = 32767, ymin = 32767, xmax = -1, ymax = -1;
    for (int i = 0; i < n; i++) {
        if (blob_x[i] < xmin) xmin = blob_x[i];
        if (blob_x[i] > xmax) xmax = blob_x[i];
        if (blob_y[i] < ymin) ymin = blob_y[i];
        if (blob_y[i] > ymax) ymax = blob_y[i];
    }
    int bbox_w = xmax - xmin + 1;
    int bbox_h = ymax - ymin + 1;

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

static void detect_shapes(int W, int H) {
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

/* ================ Sobel pass ================ */
static void run_sobel(void) {
    sob_reset();
    sob_set_width(FRAME_WIDTH);
    sob_set_thr(SOBEL_THRESHOLD);

    volatile uint32_t *gray_w = (volatile uint32_t *) &grayscale[0];

    for (uint32_t y = 0; y < FRAME_HEIGHT; y++) {
        for (uint32_t wi = 0; wi < WORDS_PER_ROW; wi++) {
            sob_feed(gray_w[y * WORDS_PER_ROW + wi]);
        }
        if (y >= 1) {
            uint32_t out_y = y - 1;
            /* Drain row into a small local buffer, then horizontally
             * dilate and write to sobelOut. Horizontal dilation closes
             * the every-other-pixel gaps coming from grayscale aliasing.
             */
            static uint8_t row_buf[FRAME_WIDTH];
            for (uint32_t wi = 0; wi < WORDS_PER_ROW; wi++) {
                uint32_t w = sob_read_out(wi);
                row_buf[wi*4 + 0] = (uint8_t)( w        & 0xFF);
                row_buf[wi*4 + 1] = (uint8_t)((w >> 8)  & 0xFF);
                row_buf[wi*4 + 2] = (uint8_t)((w >> 16) & 0xFF);
                row_buf[wi*4 + 3] = (uint8_t)((w >> 24) & 0xFF);
            }
            /* 1D dilation: out[x] = max(row[x-1], row[x], row[x+1]) */
            sobelOut[out_y * FRAME_WIDTH + 0] = row_buf[0] | row_buf[1];
            for (uint32_t x = 1; x < FRAME_WIDTH - 1; x++) {
                uint8_t r = row_buf[x-1] | row_buf[x] | row_buf[x+1];
                sobelOut[out_y * FRAME_WIDTH + x] = r;
            }
            sobelOut[out_y * FRAME_WIDTH + FRAME_WIDTH - 1] =
                row_buf[FRAME_WIDTH - 2] | row_buf[FRAME_WIDTH - 1];
            sob_ack_row();
        }
    }
}

/* ================ Main ================ */
int main(void) {
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

    printf("Init camera...\n");
    camParams = initOv7670(VGA);
    printf("Done: %d x %d @ %d FPS\n",
           camParams.nrOfPixelsPerLine,
           camParams.nrOfLinesPerImage,
           camParams.framesPerSecond);

    result = (camParams.nrOfPixelsPerLine <= 320)
             ? camParams.nrOfPixelsPerLine | 0x80000000U
             : camParams.nrOfPixelsPerLine;
    vga[0] = swap_u32(result);
    result = (camParams.nrOfLinesPerImage <= 240)
             ? camParams.nrOfLinesPerImage | 0x80000000U
             : camParams.nrOfLinesPerImage;
    vga[1] = swap_u32(result);

    vga[2] = swap_u32(2);
    vga[3] = swap_u32((uint32_t) &sobelOut[0]);

    enableContinues((uint32_t) &grayscale[0]);

    while (1) {
        profile_reset();
        run_sobel();
        profile_read(&cycles, &stall, &idle);
        print_stage("sobel", cycles, stall, idle);

        profile_reset();
        detect_shapes(FRAME_WIDTH, FRAME_HEIGHT);
        profile_read(&cycles, &stall, &idle);
        print_stage("detect", cycles, stall, idle);

        printf("---\n");
    }
}