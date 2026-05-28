#ifndef PRINTF_LOCK_H
#define PRINTF_LOCK_H

// Create the recursive mutex used by the am_util_stdio_printf wrapper.
// Call once from main() before run_tasks(). Uses static storage — no heap.
void printf_lock_init(void);

#endif
