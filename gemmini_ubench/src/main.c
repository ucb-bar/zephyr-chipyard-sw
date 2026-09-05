/*
 * Gemmini conv microbenchmark — Zephyr entry point.
 * Pins nothing / single hart; bench.c does the Gemmini + RVV work.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

extern void bench_all(void);

void bench_print(const char *s)
{
    printk("%s", s);
}

int main(void)
{
    printk("*** gemmini_conv_ubench ***\n");
    bench_all();
    printk("*** done, spinning ***\n");
    for (;;) { k_msleep(1000); }
    return 0;
}
