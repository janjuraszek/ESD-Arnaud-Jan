#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>

#define CI_GRAY_ID 10
#define CI_DMA_ID 20
#define CI_PROF_ID 12

// ramDmaCi valueA bits
//   bits [8:0]  : CI memory word address                                 
//   bit  [9]    : write enable                                           
//   bits [12:10]: register selector                                      

#define WRITE_BIT        (1U << 9)
#define REG_BUS_ADDR     (1U << 10)   // DMA bus-side start address       
#define REG_MEM_ADDR     (2U << 10)   // DMA CI-memory start address      
#define REG_BLOCK_SIZE   (3U << 10)   // DMA block size (# of 32-bit words)
#define REG_BURST_SIZE   (4U << 10)   // DMA burst size (# words - 1)     
#define REG_CTRL_STATUS  (5U << 10)   // write: control / read: status    

#define BUF_A  0U
#define BUF_B  256U

#define PIXELS_PER_BATCH     512U
#define RGB_WORDS_PER_BATCH  256U    
#define GRAY_WORDS_PER_BATCH 128U    
#define TOTAL_PIXELS         (640U * 480U)   //307200
#define NR_BATCHES           (TOTAL_PIXELS / PIXELS_PER_BATCH)  // 600   

#define BURST_SIZE           15U

// Write a 32-bit word to CI memory (single-cycle operation). 
static inline void ci_write(uint32_t addr, uint32_t data)
{
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(addr | WRITE_BIT), [d]"r"(data));
}

// Read a 32-bit word from CI memory. (Read takes 2 µC cycles) 
static inline uint32_t ci_read(uint32_t addr)
{
    uint32_t v;
    asm volatile ("l.nios_rrr %[v],%[a],r0,20"
                  : [v]"=r"(v) : [a]"r"(addr));
    return v;
}

// Poll until the DMA controller reports idle (status bit 0 = 0)
static inline void dma_wait(void)
{
    uint32_t status;
    do {
        asm volatile ("l.nios_rrr %[s],%[a],r0,20"
                      : [s]"=r"(status) : [a]"r"(REG_CTRL_STATUS));
    } while (status & 1U);
}

// Start a non-blocking DMA transfer: bus memory → CI memory.
static void dma_in(uint32_t bus_addr, uint32_t ci_base)
{
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"(bus_addr));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(ci_base));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BLOCK_SIZE | WRITE_BIT),  [d]"r"(RGB_WORDS_PER_BATCH));
    //Start DMA: bus -> CI memory (control bit 0)
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(1U));
}

// Start a DMA transfer: CI memory → bus memory, then block until done.
static void dma_out(uint32_t ci_base, uint32_t bus_addr)
{
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"(bus_addr));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(ci_base));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BLOCK_SIZE | WRITE_BIT),  [d]"r"(GRAY_WORDS_PER_BATCH));
    // Start DMA: CI memory -> bus (control bit 1) 
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(2U));
    dma_wait();
}

// Grayscale conversion on one CI-memory buffer
static void convert_buf(uint32_t buf_base)
{
    uint32_t i;
    for (i = 0; i < GRAY_WORDS_PER_BATCH; i++) {
        // Read two consecutive RGB565 words -> 4 pixels
        uint32_t w0 = ci_read(buf_base + 2U * i);
        uint32_t w1 = ci_read(buf_base + 2U * i + 1U);
        // 4-pixel grayscale CI: valueA = w0, valueB = w1 -> 4 gray bytes 
        uint32_t gray;
        asm volatile ("l.nios_rrr %[g],%[a],%[b],10" : [g]"=r"(gray) : [a]"r"(w0), [b]"r"(w1));
        // Overwrite same buffer; safe because write addr < next read addr 
        ci_write(buf_base + i, gray);
    }
}

// Profile CI helpers  (CI ID 12)

static inline void profile_reset(void)
{
    // Reset all counters (valueB = 7 selects execution counter start) 
    asm volatile ("l.nios_rrr r0,r0,%[v],0xC" :: [v]"r"(7));
}

static inline void profile_read(uint32_t *cycles, uint32_t *stall, uint32_t *idle)
{
    asm volatile ("l.nios_rrr %[c],r0,%[v],0xC" : [c]"=r"(*cycles) : [v]"r"((1 << 8) | (7 << 4)));
    asm volatile ("l.nios_rrr %[s],%[o],%[v],0xC" : [s]"=r"(*stall)  : [o]"r"(1), [v]"r"(1 << 9));
    asm volatile ("l.nios_rrr %[i],%[o],%[v],0xC" : [i]"=r"(*idle)   : [o]"r"(2), [v]"r"(1 << 10));
}

// Frame buffers (allocated in SDRAM)

volatile uint16_t rgb565[640 * 480];
volatile uint8_t  grayscale[640 * 480];


int main(void)
{
    volatile unsigned int *vga = (unsigned int *) 0x50000020;
    camParameters camParams;
    uint32_t cycles, stall, idle, result;

    vga_clear();

    printf("Initialising camera (up to 3 seconds)...\n");
    camParams = initOv7670(VGA);
    printf("Done! %d x %d @ %d FPS\n",
           camParams.nrOfPixelsPerLine,
           camParams.nrOfLinesPerImage,
           camParams.framesPerSecond);

    // Configure VGA controller for grayscale output
    result = (camParams.nrOfPixelsPerLine <= 320)
             ? camParams.nrOfPixelsPerLine | 0x80000000U
             : camParams.nrOfPixelsPerLine;
    vga[0] = swap_u32(result);

    result = (camParams.nrOfLinesPerImage <= 240)
             ? camParams.nrOfLinesPerImage | 0x80000000U
             : camParams.nrOfLinesPerImage;
    vga[1] = swap_u32(result);

    vga[2] = swap_u32(2U);                            /* grayscale mode  */
    vga[3] = swap_u32((uint32_t) &grayscale[0]);

    // set burst size once
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BURST_SIZE | WRITE_BIT), [d]"r"(BURST_SIZE));

    while (1) {

        // Capture one colour frame into SDRAM
        takeSingleImageBlocking((uint32_t) &rgb565[0]);

        
        volatile uint32_t *rgb_w  = (volatile uint32_t *) &rgb565[0];
        volatile uint32_t *gray_w = (volatile uint32_t *) &grayscale[0];

        // cur  = buffer whose RGB565 data is ready to compute on           
        // nxt  = buffer that will receive the next DMA-in                  
        uint32_t cur = BUF_A;
        uint32_t nxt = BUF_B;

        profile_reset();
        
        //Transfer the very first batch of 512 RGB565 pixels into BUF_A.
        dma_in((uint32_t) &rgb_w[0], cur);
        dma_wait();

        // 599 overlapped iterations
       
        uint32_t b;
        for (b = 0; b < NR_BATCHES - 1U; b++) {

            // DMA-in for the next batch 
            dma_in((uint32_t) &rgb_w[(b + 1U) * RGB_WORDS_PER_BATCH], nxt);

            // Convert the current batch 
            convert_buf(cur);

            // wait for dma-in
            dma_wait();

            dma_out(cur, (uint32_t) &gray_w[b * GRAY_WORDS_PER_BATCH]);

            // wap ping-pong
            uint32_t tmp = cur;
            cur = nxt;
            nxt = tmp;
        }

        // Final batch
        convert_buf(cur);
        dma_out(cur, (uint32_t) &gray_w[(NR_BATCHES - 1U) * GRAY_WORDS_PER_BATCH]);

        // profile counters
        profile_read(&cycles, &stall, &idle);
        printf("cycles: %u  stall: %u  idle: %u  (real work: %u)\n",
               cycles, stall, idle, cycles - stall);
    }
}
  