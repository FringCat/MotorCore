/**
 * @file sysmem.c
 * @brief Newlib heap support (_sbrk) for GCC/newlib-nano.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

extern uint8_t _end;
extern uint8_t _estack;
extern uint32_t _Min_Stack_Size;

static uint8_t *heap_end;

void *_sbrk(ptrdiff_t incr)
{
    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t *max_heap = (uint8_t *)stack_limit;
    uint8_t *prev_heap_end;

    if (heap_end == NULL)
    {
        heap_end = &_end;
    }

    prev_heap_end = heap_end;

    if ((uint32_t)(heap_end + incr) > (uint32_t)max_heap)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}
