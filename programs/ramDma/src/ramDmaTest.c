#include <stdio.h>
#include <stdint.h>


// valueA encoding: A[12:10]=register select, A[9]=write enable, A[8:0]=mem address

// Direct memory read/write (A[12:10]=000)
#define MEM_WRITE(addr, data) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9) | (addr & 0x1FF)), [in2]"r"(data))

#define MEM_READ(addr, res) \
    asm volatile ("l.nios_rrr %[out1],%[in1],r0,0x08" \
        : [out1]"=r"(res) : [in1]"r"(addr & 0x1FF))

// DMA register writes
#define DMA_SET_BUS_ADDR(addr) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9)|(1<<10)), [in2]"r"(addr))

#define DMA_SET_MEM_ADDR(addr) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9)|(2<<10)), [in2]"r"(addr))

#define DMA_SET_BLOCK_SIZE(n) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9)|(3<<10)), [in2]"r"(n))

#define DMA_SET_BURST_SIZE(n) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9)|(4<<10)), [in2]"r"(n))

#define DMA_START() \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08" \
        :: [in1]"r"((1<<9)|(5<<10)), [in2]"r"(1))

#define DMA_STATUS(res) \
    asm volatile ("l.nios_rrr %[out1],%[in1],r0,0x08" \
        : [out1]"=r"(res) : [in1]"r"((5<<10)))

int main() {
    volatile uint32_t status, val;
    volatile uint32_t *sdram = (volatile uint32_t *)0x01000000; 
    int errors = 0;

    printf("CI Memory Test \n");
    MEM_WRITE(0, 0xABABABAB);
    MEM_WRITE(1, 0x12345678);
    MEM_READ(0, val); printf("Mem[0] = %08x (expected: ABABABAB)\n", val);
    MEM_READ(1, val); printf("Mem[1] = %08x (expected: 12345678)\n", val);

    printf("\n Writing to SDRAM\n");
    for (int i = 0; i < 16; i++) {
        sdram[i] = 0xA5A50000 + i;   
    }
    

    // Test single burst
    printf("DMA Test: 4 words, burst of 4\n");
    DMA_SET_BUS_ADDR((uint32_t)sdram);
    DMA_SET_MEM_ADDR(10);
    DMA_SET_BLOCK_SIZE(4);
    DMA_SET_BURST_SIZE(3);     
    DMA_START();
    do { DMA_STATUS(status); } while (status & 0x1);
    if (status & 0x2) { printf("BUS ERROR\n"); errors++; }
    for (int i = 0; i < 4; i++) {
        MEM_READ(10 + i, val);
        uint32_t expect = 0xA5A50000 + i;
        printf("  Mem[%d]=%08x exp %08x %s\n", 10+i, val, expect,
               val == expect ? "OK" : "FAIL");
        if (val != expect) errors++;
    }
    
    return 0;
}