#include <stdint.h>
#include <stdio.h>
#include <swap.h>

volatile uint32_t memBuffer[512];

int main() {
  const uint32_t writeBit = 1<<9;
  const uint32_t busStartAddress = 1 << 10;
  const uint32_t memoryStartAddress = 2 << 10;
  const uint32_t blockSize = 3 << 10;
  const uint32_t burstSize = 4 << 10;
  const uint32_t statusControl = 5 << 10;
  const uint32_t usedCiRamAddress = 50;
  const uint32_t usedBlocksize = 512;
  const uint32_t usedBurstSize = 25;
  uint32_t ramAddress, ramData;
  printf("Writing\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    asm volatile("l.nios_rrr r0,%[in1],r0,20" ::[in1] "r"(ramAddress | writeBit)); // we clear the memory
  }
  printf("Comparing\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    asm volatile("l.nios_rrr %[out1],%[in1],r0,20" :[out1]"=r"(ramData):[in1] "r"(ramAddress)); // we control that the memory is empty
    if (ramData != 0) printf("Error at address 0x%03X : 0x%08X\n", ramAddress, ramData);
  }
  printf("Writing\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(ramAddress | writeBit), [in2]"r"(ramAddress ^ 0xFFFFFF)); // we fill the memory
  }
  printf("Comparing\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    asm volatile("l.nios_rrr %[out1],%[in1],r0,20" :[out1]"=r"(ramData):[in1] "r"(ramAddress)); // we control that the memory is correct
    if (ramData != (ramAddress ^ 0xFFFFFF)) printf("Error at address 0x%03X : 0x%08X\n", ramAddress, ramData);
  }
  printf("Emptying mem buffer\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    memBuffer[ramAddress] = 0;
  }
  printf("Initialising DMA registers\n");
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(busStartAddress | writeBit),[in2] "r"((uint32_t) &memBuffer[0]));
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(memoryStartAddress | writeBit),[in2] "r"(usedCiRamAddress));
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(blockSize | writeBit),[in2] "r"(usedBlocksize));
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(burstSize | writeBit),[in2] "r"(usedBurstSize));
  printf("Performing dma from memory to ciRam\n");
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(statusControl | writeBit),[in2] "r"(1));
  printf("verifying\n");
  for (ramAddress = usedCiRamAddress; ramAddress < usedCiRamAddress+usedBlocksize; ramAddress++) {
    asm volatile("l.nios_rrr %[out1],%[in1],r0,20" :[out1]"=r"(ramData):[in1] "r"(ramAddress&0x1FF)); // we control that the memory is correct
    if (ramData != 0) printf("Error at address 0x%03X : 0x%08X\n", ramAddress, ramData);
  }
  printf("Writing ci memory\n");
  for (ramAddress = 0; ramAddress < 512; ramAddress++) {
    asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(ramAddress | writeBit), [in2]"r"(ramAddress ^ 0xFFFFFF00)); // we fill the memory
  }
  printf("Performing dma from ciRam to memory\n");
  asm volatile("l.nios_rrr r0,%[in1],%[in2],20" ::[in1] "r"(statusControl | writeBit),[in2] "r"(2));
  printf("verifying\n");
  for (ramAddress = 0; ramAddress < usedBlocksize; ramAddress++) {
    ramData = swap_u32(memBuffer[ramAddress&0x1FF]);
    if (ramData != (((ramAddress+usedCiRamAddress)&0x1FF) ^ 0xFFFFFF00) ) printf("Error at address 0x%03X : 0x%08X\n", ramAddress, ramData);
  }
  printf("Done\n");
}
