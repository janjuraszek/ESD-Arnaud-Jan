#include <stdio.h>
#include <ov7670.h>
#include <swap.h>
#include <vga.h>

// Write to CI memory: set bit9 for write, bits[8:0] = address
#define CI_WRITE(addr, data) \
    asm volatile ("l.nios_rrr r0,%[in1],%[in2],0x0" \
        :: [in1]"r"((1<<9) | (addr & 0x1FF)), [in2]"r"(data))

// Read from CI memory: bit9=0, bits[8:0] = address
#define CI_READ(addr, result) \
    asm volatile ("l.nios_rrr %[out1],%[in1],r0,0x0" \
        : [out1]"=r"(result) : [in1]"r"(addr & 0x1FF))

int main() {
    volatile uint32_t readback;

    printf("Testing CI-attached RAM\n");

    // Write test values
    CI_WRITE(0, 0xDEADBEEF);
    CI_WRITE(1, 0xCAFEBABE);
    CI_WRITE(5, 0x12345678);

    // Read back and verify
    CI_READ(0, readback);
    printf("Addr 0: %08x (expected DEADBEEF) %s\n", readback, readback == 0xDEADBEEF ? "OK" : "FAIL");

    CI_READ(1, readback);
    printf("Addr 1: %08x (expected CAFEBABE) %s\n", readback, readback == 0xCAFEBABE ? "OK" : "FAIL");

    CI_READ(5, readback);
    printf("Addr 5: %08x (expected 12345678) %s\n", readback, readback == 0x12345678 ? "OK" : "FAIL");

    printf("Done.\n");
    return 0;
}