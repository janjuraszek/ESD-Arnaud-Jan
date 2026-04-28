#include <stdio.h>
#include <stdint.h>

// CI instruction number 8
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

    printf("=== CI Memory Test ===\n");    
    MEM_WRITE(0, 0xDEADBEEF);
    MEM_WRITE(1, 0xCAFEBABE);
    MEM_READ(0, val);
    printf("Mem[0] = %08x (expected DEADBEEF)\n", val);
    MEM_READ(1, val);
    printf("Mem[1] = %08x (expected CAFEBABE)\n", val);

    printf("=== DMA Test ===\n");
    // DMA from SDRAM address 0x00200000, 4 words, burst of 4
    DMA_SET_BUS_ADDR(0x00200000);
    DMA_SET_MEM_ADDR(10);          // write to CI mem starting at address 10
    DMA_SET_BLOCK_SIZE(4);
    DMA_SET_BURST_SIZE(3);         // burst of 4 (burstsize+1)
    DMA_START();

    // poll until idle
    do {
        DMA_STATUS(status);
    } while (status & 0x1);

    if (status & 0x2) {
        printf("DMA bus error!\n");
    } else {
        printf("DMA done.\n");
        MEM_READ(10, val); printf("Mem[10] = %08x\n", val);
        MEM_READ(11, val); printf("Mem[11] = %08x\n", val);
        MEM_READ(12, val); printf("Mem[12] = %08x\n", val);
        MEM_READ(13, val); printf("Mem[13] = %08x\n", val);
    }

    return 0;
}