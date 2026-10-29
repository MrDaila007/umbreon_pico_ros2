#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef unsigned int uint;
typedef struct { int unused; } spin_lock_t;
static inline int spin_lock_claim_unused(bool required) { (void)required; return 0; }
static inline spin_lock_t *spin_lock_instance(uint n) {
    (void)n;
    static spin_lock_t lock;
    return &lock;
}
static inline uint32_t spin_lock_blocking(spin_lock_t *lock) { (void)lock; return 0; }
static inline void spin_unlock(spin_lock_t *lock, uint32_t saved) { (void)lock; (void)saved; }
