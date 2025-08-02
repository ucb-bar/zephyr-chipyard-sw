/*
 * Minimal test to isolate the hanging issue
 */

#include <stdio.h>
#include <stdlib.h>

extern "C" {
uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}
}

int main(void) {
    printf("Hello from minimal main!\n");
    fflush(stdout);
    
    printf("Testing basic functionality...\n");
    fflush(stdout);
    
    uint64_t cycles = read_cycles();
    printf("Cycles: %llu\n", cycles);
    fflush(stdout);
    
    printf("Test completed successfully!\n");
    fflush(stdout);
    
    return 0;
}