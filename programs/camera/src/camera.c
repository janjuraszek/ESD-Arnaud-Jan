#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#include <sobel.h>
#ifdef __OR1300__
#include <cache.h>
#endif

/* ============================================================
 * DISPLAY_MODE:
 *   0 = raw RGB565 camera feed
 *   1 = grayscale
 *   2 = Sobel edge detection
 * ============================================================ */
#ifndef DISPLAY_MODE
#define DISPLAY_MODE 0
#endif

#define SOBEL_THRESHOLD 128

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

/* ---------- Frame buffers (SDRAM) ---------- */
volatile uint16_t rgb565[640 * 480];
volatile uint8_t  grayscale[640 * 480];
volatile uint8_t  sobelOut[640 * 480];

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

    printf("Initialising camera (up to 3 seconds)...\n");
    camParams = initOv7670(VGA);
    printf("Done! %d x %d @ %d FPS  | DISPLAY_MODE=%d\n",
           camParams.nrOfPixelsPerLine,
           camParams.nrOfLinesPerImage,
           camParams.framesPerSecond,
           DISPLAY_MODE);

    const int W = camParams.nrOfPixelsPerLine;
    const int H = camParams.nrOfLinesPerImage;

    /* VGA resolution */
    result = (W <= 320) ? (uint32_t)W | 0x80000000U : (uint32_t)W;
    vga[0] = swap_u32(result);
    result = (H <= 240) ? (uint32_t)H | 0x80000000U : (uint32_t)H;
    vga[1] = swap_u32(result);

    /* VGA mode + buffer chosen once, based on DISPLAY_MODE */
#if DISPLAY_MODE == 0
    vga[2] = swap_u32(1);                              /* RGB565 mode    */
    vga[3] = swap_u32((uint32_t) &rgb565[0]);
#elif DISPLAY_MODE == 1
    vga[2] = swap_u32(2);                              /* grayscale mode */
    vga[3] = swap_u32((uint32_t) &grayscale[0]);
#else
    vga[2] = swap_u32(2);                              /* grayscale mode */
    vga[3] = swap_u32((uint32_t) &sobelOut[0]);
#endif

    while (1) {
        takeSingleImageBlocking((uint32_t) &rgb565[0]);

#if DISPLAY_MODE >= 1
        /* RGB565 -> grayscale */
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
#endif

#if DISPLAY_MODE == 2
        /* Sobel */
        profile_reset();
        edgeDetection(grayscale, sobelOut, W, H, SOBEL_THRESHOLD);
        profile_read(&cycles, &stall, &idle);
        print_stage("sobel", cycles, stall, idle);
#endif
    }
}