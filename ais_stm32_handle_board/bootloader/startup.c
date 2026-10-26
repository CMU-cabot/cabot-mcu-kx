#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

int main(void);

__attribute__((noreturn)) void Default_Handler(void) {
  for (;;) {
  }
}

__attribute__((noreturn)) void Reset_Handler(void) {
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;
  while (dst < &_edata) {
    *dst++ = *src++;
  }
  for (dst = &_sbss; dst < &_ebss; ++dst) {
    *dst = 0;
  }
  (void)main();
  Default_Handler();
}

__attribute__((used, section(".isr_vector")))
const void *const vector_table[98] = {
    &_estack,
    Reset_Handler,
    [2 ... 97] = Default_Handler,
};
