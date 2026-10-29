#pragma once
#include <stddef.h>
#include <stdbool.h>
typedef struct {
    void *(*allocate)(size_t, void *);
    void (*deallocate)(void *, void *);
    void *(*reallocate)(void *, size_t, void *);
    void *(*zero_allocate)(size_t, size_t, void *);
    void *state;
} rcutils_allocator_t;
bool rcutils_set_default_allocator(rcutils_allocator_t *allocator);
