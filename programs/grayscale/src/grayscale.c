#include <stdio.h>
#include <stdint.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>

//#define CI_GRAY_ID 10
//#define CI_DMA_ID 20
//#define CI_PROF_ID 12

//#define GRAYSCALE
//#define _COLOR_

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

#define PIXELS_PER_BATCH     320U
#define RGB_WORDS_PER_BATCH  160U    
#define GRAY_WORDS_PER_BATCH 80U    
#define TOTAL_PIXELS         (640U * 480U)   //307200
#define NR_BATCHES           (TOTAL_PIXELS / PIXELS_PER_BATCH)  // 960   

#define BURST_SIZE           31U

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
static inline void dma_in(uint32_t bus_addr, uint32_t ci_base)
{
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"(bus_addr));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(ci_base));
    //Start DMA: bus -> CI memory (control bit 0)
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(1U));
}

// Start a DMA transfer: CI memory → bus memory, then block until done.
/*
static void dma_out(uint32_t ci_base, uint32_t bus_addr)
{
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"(bus_addr));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(ci_base));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BLOCK_SIZE | WRITE_BIT),  [d]"r"(GRAY_WORDS_PER_BATCH));
    // Start DMA: CI memory -> bus (control bit 1) 
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(2U));
    dma_wait();
}
*/

// Grayscale conversion on one CI-memory buffer
/*
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
*/ 

static inline uint32_t convert_4_pixels(uint32_t address)
{
	// Read two consecutive RGB565 words -> 4 pixels
	uint32_t w0 = ci_read(address);
	uint32_t w1 = ci_read(address+1);
	//w0 = swap_u32(w0);
	//w1 = swap_u32(w1);
	// 4-pixel grayscale CI: valueA = w0, valueB = w1 -> 4 gray bytes 
	uint32_t gray;
	asm volatile ("l.nios_rrr %[g],%[a],%[b],10" : [g]"=r"(gray) : [a]"r"(w0), [b]"r"(w1));
	// Overwrite same buffer; safe because write addr < next read addr 
	//ci_write(buf_base + i, gray);
	return gray;
}


static inline uint8_t ci_do_sobel(uint32_t gray_pixs)
{
	uint32_t ci_output = 0;
    asm volatile ("l.nios_rrr %[res],%[instr],%[pixels],11" :[res]"=r"(ci_output): [instr]"r"(0), [pixels]"r"(gray_pixs));
    //uint8_t result = ci_output;
    return (int8_t)ci_output;
}

static inline void ci_reset_sobel()
{
    asm volatile ("l.nios_rrr r0,%[instr],r0,11" :: [instr]"r"(1));
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
//volatile uint8_t floyd[640*480];


int main()
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
    
    
    
    
    //#ifdef GRAYSCALE
		//vga[2] = swap_u32(2U);                            /* grayscale mode  */
		//vga[3] = swap_u32((uint32_t) &grayscale[0]);    
	//#endif
	//#ifdef __COLOR__
		//vga[2] = swap_u32(1U);                            /* color mode  */
		//vga[3] = swap_u32((uint32_t) &rgb565[0]);
	//#endif
    
    vga[2] = swap_u32(2);                            /* grayscale mode  */
	vga[3] = swap_u32((uint32_t) &grayscale[0]); 
    

    // set burst size once
    asm volatile ("l.nios_rrr r0,%[a],%[d],20" :: [a]"r"(REG_BURST_SIZE | WRITE_BIT), [d]"r"(BURST_SIZE));
    asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BLOCK_SIZE | WRITE_BIT),  [d]"r"(RGB_WORDS_PER_BATCH));
    
    uint32_t *rgb_w  = (uint32_t *) &rgb565[0];
    uint32_t *gray_w = (uint32_t *) &grayscale[0];

    while (1) {
		//printf("e\n");

        // Capture one colour frame into SDRAM
        takeSingleImageBlocking((uint32_t) &rgb565[0]);
        
		//ci_reset_sobel();
		asm volatile ("l.nios_rrr r0,%[instr],r0,11" :: [instr]"r"(1));
		
        
        // cur  = buffer whose RGB565 data is ready to compute on           
        // nxt  = buffer that will receive the next DMA-in                  
        uint32_t cur = BUF_A;
        uint32_t nxt = BUF_B;
        
        uint32_t counter = 0;

        //profile_reset();
        asm volatile ("l.nios_rrr r0,r0,%[v],0xC" :: [v]"r"(7));
        
        //Transfer the very first batch of 512 RGB565 pixels into BUF_A.
        //dma_in((uint32_t) &rgb_w[0], cur);
        asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"((uint32_t) &rgb_w[0]));
		asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(cur));
		//Start DMA: bus -> CI memory (control bit 0)
		asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(1U));
		
        //dma_wait();
        uint32_t status;
		do {
			asm volatile ("l.nios_rrr %[s],%[a],r0,20" : [s]"=r"(status) : [a]"r"(REG_CTRL_STATUS));
		} while (status & 1U);

        // 599 overlapped iterations
        //printf("for\n");
        for (uint32_t half_line = 0; half_line < NR_BATCHES - 1U; half_line++) {
			//dma_in((uint32_t) &rgb_w[(half_line+1)*RGB_WORDS_PER_BATCH], nxt);
			
			asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_BUS_ADDR | WRITE_BIT),    [d]"r"((uint32_t) &rgb_w[(half_line+1)*RGB_WORDS_PER_BATCH]));
			asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_MEM_ADDR | WRITE_BIT),    [d]"r"(nxt));
			//Start DMA: bus -> CI memory (control bit 0)
			asm volatile ("l.nios_rrr r0,%[a],%[d],20"  :: [a]"r"(REG_CTRL_STATUS | WRITE_BIT), [d]"r"(1U));
			
			
			
			
			
			for (uint32_t group = 0; group < 80; group++) {
				//uint32_t gray_pixels = convert_4_pixels(2*group+cur);
				uint32_t gray_pixels;
				
				
				uint32_t w0 = ci_read(2*group+cur);
				uint32_t w1 = ci_read(2*group+cur+1);
				//w0 = swap_u32(w0);
				//w1 = swap_u32(w1);
				// 4-pixel grayscale CI: valueA = w0, valueB = w1 -> 4 gray bytes 
				//uint32_t gray;
				asm volatile ("l.nios_rrr %[g],%[a],%[b],10" : [g]"=r"(gray_pixels) : [a]"r"(w0), [b]"r"(w1));
				// to display grayscale image
				gray_w[half_line*80+group] = gray_pixels;
				// ----
				
				//uint8_t sobel_result = ci_do_sobel(gray_pixels);
				/*
				counter += (sobel_result & 0x1) ? 1 : 0;
				counter += (sobel_result & 0x2) ? 1 : 0;
				counter += (sobel_result & 0x4) ? 1 : 0;
				counter += (sobel_result & 0x8) ? 1 : 0;
				*/
				//gray_pixels = ((sobel_result & 0x8) ? 0xFF000000U : 0U) | ((sobel_result & 0x4) ? 0x00FF0000U : 0U) | ((sobel_result & 0x2) ? 0x0000FF00U : 0U) | ((sobel_result & 0x1) ? 0x000000FFU : 0U);
				
				//gray_w[half_line*80+group] = gray_pixels;
				
				
			}
			dma_wait();
			uint32_t tmp = cur;
			cur = nxt;
			nxt = tmp;
			
			//printf("forint\n");
        }
        

        // Final batch
        for (uint32_t group = 0; group < 80; group++) {
			uint32_t gray_pixels = convert_4_pixels(2*group+cur);
			
			//uint8_t sobel_result = ci_do_sobel(gray_pixels);
			
			//gray_pixels = ((sobel_result & 0x8) ? 0xFF000000U : 0U) | ((sobel_result & 0x4) ? 0x00FF0000U : 0U) | ((sobel_result & 0x2) ? 0x0000FF00U : 0U) | ((sobel_result & 0x1) ? 0x000000FFU : 0U);
				
			gray_w[(NR_BATCHES-1)*80+group] = gray_pixels;
			/*
			counter += (sobel_result & 0x1) ? 1 : 0;
			counter += (sobel_result & 0x2) ? 1 : 0;
			counter += (sobel_result & 0x4) ? 1 : 0;
			counter += (sobel_result & 0x8) ? 1 : 0;
			* */
			
		}
		//printf("%u\n", counter);
		

        // profile counters
        //profile_read(&cycles, &stall, &idle);
        //printf("cycles: %u  stall: %u  idle: %u  (real work: %u)\n", cycles, stall, idle, cycles - stall);
        //printf("%u\n", idle);
    }
}
  
