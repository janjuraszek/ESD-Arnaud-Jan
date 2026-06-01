/*
 * SOFTWARE-ONLY baseline.
 *
 * capture -> grayscale -> sobel -> detect
 *
 * Detection: for each row, find leftmost and rightmost edge pixel.
 *   - Compute average row width across the bbox.
 *   - A circle's average width is much smaller than its bbox width (~78%).
 *   - A square's average width equals (or nearly equals) its bbox width.
 *   - Threshold: if (average_width * 100) / bbox_width >= cutoff -> SQUARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>
#include <sobel.h>
#ifdef __OR1300__
#include <cache.h>
#endif

// To show specific values for bbox, shape, and profiling
#define DEBUG


// Profile CI Functions

static inline void profile_reset(void)
{
    asm volatile ("l.nios_rrr r0,r0,%[v],0xC" :: [v]"r"(7));
}
static inline void profile_read(uint32_t *c, uint32_t *s, uint32_t *i)
{
    asm volatile ("l.nios_rrr %[c],r0,%[v],0xC"  : [c]"=r"(*c) : [v]"r"((1<<8)|(7<<4)));
    asm volatile ("l.nios_rrr %[s],%[o],%[v],0xC": [s]"=r"(*s) : [o]"r"(1), [v]"r"(1<<9));
    asm volatile ("l.nios_rrr %[i],%[o],%[v],0xC": [i]"=r"(*i) : [o]"r"(2), [v]"r"(1<<10));
}
static inline void print_stage(const char *n, uint32_t c, uint32_t s, uint32_t i)
{
    printf("[%-10s] cycles=%u  stall=%u  idle=%u \n", n, c, s, i);
}

// Buffers 

volatile uint16_t rgb565[640 * 480];
volatile uint8_t  grayscale[640 * 480];
volatile uint8_t  sobelOut[640 * 480];

#define SOBEL_THRESHOLD 128 

// Detection 
#define MIN_EDGE_PIXELS  60
#define MAX_EDGE_PIXELS  10000
#define SQUARE_PERCENT   85

// Row extremities
static int16_t leftX[480];
static int16_t rightX[480];

// Bounding Box 
static int  bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax;
static int  bbox_valid;

static void detect_shapes(int W, int H)
{
    bbox_valid = 0;
    for (int y = 0; y < H; y++) { leftX[y] = -1; rightX[y] = -1; } // Reset extremities, -1 = empty

    int xmin = W, xmax = -1, ymin = H, ymax = -1; // set infinite boundaries
    int32_t total_edges = 0;
    int total_width = 0;
    int widths_count = 0;

    // Scan Sobel image
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            if (sobelOut[y * W + x] != 0) {
                total_edges++;
                // Set bbox
                if (x < xmin) xmin = x;
                if (x > xmax) xmax = x;
                if (y < ymin) ymin = y;
                if (y > ymax) ymax = y;
                // Set extremities
                if (leftX[y] < 0) leftX[y] = (int16_t)x;
                rightX[y] = (int16_t)x;
            }
        }
        // Calculate width if there is an existing edge in the row
        if (leftX[y] >= 0) {
            total_width += rightX[y] - leftX[y];
            widths_count++;
        }
    }

    if (total_edges < MIN_EDGE_PIXELS || xmax < 0) {
        printf("No shape\n");
        return;
    }
    if (total_edges > MAX_EDGE_PIXELS) {
        printf("Too many edges\n");
        return;
    }

    if (widths_count == 0) {
        printf("No rows\n");
        return;
    }

    // Bounding box dimensions
    int bboxW = xmax - xmin + 1;
    int bboxH = ymax - ymin + 1;

    // Classifier parameter calculation
    int average_width = total_width / widths_count;
    int percent = (average_width * 100) / bboxW;

    // Sample a small area of RGB image at shape's center to determine R, G or B.
    int cx = (xmin + xmax) / 2;
    int cy = (ymin + ymax) / 2;
    uint32_t rsum = 0, gsum = 0, bsum = 0;
    const int RADIUS = 3;   
    int samples = 0;
    for (int dy = -RADIUS; dy <= RADIUS; dy++) {
        for (int dx = -RADIUS; dx <= RADIUS; dx++) {
            int px = cx + dx, py = cy + dy;
            if (px < 0 || px >= W || py < 0 || py >= H) continue;

            // Take sample with proper shifts and bit masks (R: 5, G: 6, B: 5)
            uint16_t rgb = swap_u16(rgb565[py * W + px]);
            rsum += ((rgb >> 11) & 0x1F) << 3;
            gsum += ((rgb >> 5)  & 0x3F) << 2;
            bsum += ( rgb        & 0x1F) << 3;
            samples++;
        }
    }
    // Average color
    rsum /= samples; 
    gsum /= samples; 
    bsum /= samples;

    // Determine color by channel dominance
    const char *color;
    if      (rsum > gsum && rsum > bsum) color = "RED";
    else if (gsum > bsum)                color = "GREEN";
    else                                  color = "BLUE";

    // Specific shape and bbox values
    #ifdef DEBUG
        printf("  bbox x=%d..%d  y=%d..%d  (w=%d h=%d)  edges=%d\n",
            xmin, xmax, ymin, ymax, bboxW, bboxH, (int)total_edges);
        printf("  avgW=%d  pct=%d\n", average_width, percent);
        printf("  rgb=(%u,%u,%u)  color=%s\n", rsum, gsum, bsum, color);
    #endif
    
    // Shape classifier
    const char *shape;         
    if (percent >= SQUARE_PERCENT)
        shape = "Square";
    else
        shape = "Circle"; 
    
    printf("    Found a %s %s at x=%d, y=%d!\n", color, shape, cx, cy);

    // Bounding box 
    bbox_xmin = xmin; bbox_xmax = xmax;
    bbox_ymin = ymin; bbox_ymax = ymax;
    bbox_valid = 1;
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

    // RGB565 mode
    vga[2] = swap_u32(1);    
    vga[3] = swap_u32((uint32_t) &rgb565[0]);

    const int W = camParams.nrOfPixelsPerLine;
    const int H = camParams.nrOfLinesPerImage;

    while (1) {
        #ifdef DEBUG
        profile_reset();
        #endif

        // Capture RGB image
        takeSingleImageBlocking((uint32_t) &rgb565[0]);
        
        #ifdef DEBUG
        profile_read(&cycles, &stall, &idle);
        print_stage("capture", cycles, stall, idle);
        profile_reset();
        #endif

        // Convert RGB to Grayscale
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
        #ifdef DEBUG
        profile_read(&cycles, &stall, &idle);
        print_stage("grayscale", cycles, stall, idle);
        profile_reset();
        #endif

        // Sobel algorithm
        edgeDetection(grayscale, sobelOut, W, H, SOBEL_THRESHOLD);

        #ifdef DEBUG
        profile_read(&cycles, &stall, &idle);
        print_stage("sobel", cycles, stall, idle);
        profile_reset();
        #endif

        // Shape detection
        detect_shapes(W, H);

        #ifdef DEBUG
        profile_read(&cycles, &stall, &idle);
        print_stage("detect", cycles, stall, idle);
        #endif

        

        printf("-\n");
    }
}