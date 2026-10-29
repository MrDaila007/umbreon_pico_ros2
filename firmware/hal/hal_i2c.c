/* hal_i2c.c - see hal_i2c.h */
#include "hal_i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/time.h"

#define I2C_TIMEOUT_US 5000u
#define I2C_BUS_COUNT 2

typedef struct {
    i2c_inst_t *inst;
    uint sda, scl, baud;
    SemaphoreHandle_t mutex;
    umb_i2c_t api;
} bus_t;

static bus_t s_bus[I2C_BUS_COUNT];

static int do_write(void *ctx, uint8_t addr, const uint8_t *buf, size_t len, bool nostop) {
    bus_t *b = ctx;
    if (!hal_i2c_lock((hal_i2c_bus_t)(b - s_bus))) return -1;
    int r = i2c_write_blocking_until(b->inst, addr, buf, len, nostop,
                                     make_timeout_time_us(I2C_TIMEOUT_US));
    hal_i2c_unlock((hal_i2c_bus_t)(b - s_bus));
    return (r == (int)len) ? 0 : -1;
}

static int do_read(void *ctx, uint8_t addr, uint8_t *buf, size_t len) {
    bus_t *b = ctx;
    if (!hal_i2c_lock((hal_i2c_bus_t)(b - s_bus))) return -1;
    int r = i2c_read_blocking_until(b->inst, addr, buf, len, false,
                                    make_timeout_time_us(I2C_TIMEOUT_US));
    hal_i2c_unlock((hal_i2c_bus_t)(b - s_bus));
    return (r == (int)len) ? 0 : -1;
}

static void do_delay_us(void *ctx, uint32_t us) {
    (void)ctx;
    if (us >= 1000u) {
        TickType_t ticks = (TickType_t)(((uint64_t)us * configTICK_RATE_HZ + 999999u) / 1000000u);
        vTaskDelay(ticks > 0 ? ticks : 1);
    } else {
        busy_wait_us(us);
    }
}

static void bus_setup(int i, i2c_inst_t *inst, uint sda, uint scl, uint baud) {
    bus_t *b = &s_bus[i];
    b->inst = inst;
    b->sda = sda;
    b->scl = scl;
    b->baud = baud;
    b->mutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(b->mutex != NULL);
    b->api = (umb_i2c_t){.write = do_write, .read = do_read, .delay_us = do_delay_us, .ctx = b};
    i2c_init(inst, baud);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

void hal_i2c_init(void) {
    bus_setup(HAL_I2C0, i2c0, 0, 1, 400000);
    bus_setup(HAL_I2C1, i2c1, 2, 3, 100000);
}

const umb_i2c_t *hal_i2c_bus(hal_i2c_bus_t b) {
    return ((unsigned)b < I2C_BUS_COUNT) ? &s_bus[b].api : NULL;
}

bool hal_i2c_lock(hal_i2c_bus_t b) {
    if ((unsigned)b >= I2C_BUS_COUNT || s_bus[b].mutex == NULL) return false;
    return xSemaphoreTakeRecursive(s_bus[b].mutex, portMAX_DELAY) == pdTRUE;
}

void hal_i2c_unlock(hal_i2c_bus_t b) {
    if ((unsigned)b >= I2C_BUS_COUNT || s_bus[b].mutex == NULL) return;
    (void)xSemaphoreGiveRecursive(s_bus[b].mutex);
}
