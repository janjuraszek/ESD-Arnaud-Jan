#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>

// Write to CI memory: set bit9 for write, bits[8:0] = address
#define CI_WRITE(address, data)  asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x08"  :: [in1]"r"((1<<9) | (address & 0x1FF)), [in2]"r"(data))

// Read from CI memory: bit9=0, bits[8:0] = address
#define CI_READ(address, result)  asm volatile ("l.nios_rrr %[out1],%[in1],r0,0x08"  : [out1]"=r"(result) : [in1]"r"(address & 0x1FF))

int main() {
    volatile uint32_t read;

    printf("Testing CI-attached RAM \n");

    // Write  
    CI_WRITE(0, 0xABABABAB);
    CI_WRITE(5, 0x12345678);

    // Read
    CI_READ(0, read);
    printf("address 0: %08x (expected ABABABAB)\n", read);
    CI_READ(5, read);
    printf("address 5: %08x (expected 12345678)\n", read);

    printf("Done \n");
    return 0;
}