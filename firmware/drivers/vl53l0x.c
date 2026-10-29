/*
 * vl53l0x.c — Register-level VL53L0X driver.
 *
 * C port of Pololu's vl53l0x-arduino (https://github.com/pololu/vl53l0x-arduino),
 * Copyright (c) 2017-2022 Pololu Corporation, which is based on the VL53L0X API
 * (STSW-IMG005), Copyright (c) 2016 STMicroelectronics International N.V.
 * Licence terms: LICENSE-vl53l0x.txt in this directory.
 *
 * Ported: init (DataInit, StaticInit, PerformRefCalibration), getSpadInfo,
 * setSignalRateLimit, get/setMeasurementTimingBudget with sequence-step
 * timeouts, startContinuous (back-to-back) and readRangeContinuousMillimeters
 * (made non-blocking). Every wait is bounded to 100 ms, counted via delay_us.
 */
#include "vl53l0x.h"

#include <stddef.h>

/* Registers (names from the ST API) */
#define REG_SYSRANGE_START                 0x00
#define REG_SYSTEM_SEQUENCE_CONFIG         0x01
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO   0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR         0x0B
#define REG_RESULT_INTERRUPT_STATUS        0x13
#define REG_RESULT_RANGE_STATUS            0x14
#define REG_FINAL_RANGE_MIN_COUNT_RATE     0x44
#define REG_MSRC_CONFIG_TIMEOUT_MACROP     0x46
#define REG_DYNAMIC_SPAD_NUM_REQUESTED     0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFF  0x4F
#define REG_PRE_RANGE_VCSEL_PERIOD         0x50
#define REG_PRE_RANGE_TIMEOUT_MACROP_HI    0x51
#define REG_MSRC_CONFIG_CONTROL            0x60
#define REG_FINAL_RANGE_VCSEL_PERIOD       0x70
#define REG_FINAL_RANGE_TIMEOUT_MACROP_HI  0x71
#define REG_GPIO_HV_MUX_ACTIVE_HIGH        0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP  0x89
#define REG_I2C_SLAVE_DEVICE_ADDRESS       0x8A
#define REG_SPAD_ENABLES_REF_0             0xB0
#define REG_REF_EN_START_SELECT            0xB6
#define REG_IDENTIFICATION_MODEL_ID        0xC0
#define REG_PAGE                           0xFF

#define MODEL_ID                0xEE
#define SIGNAL_RATE_0_25_MCPS   0x0020   /* 0.25 MCPS in Q9.7 */
#define SEQ_ALL                 0xFF
#define SEQ_DEFAULT             0xE8     /* MSRC and TCC disabled */
#define SEQ_VHV_CAL             0x01
#define SEQ_PHASE_CAL           0x02
#define RANGE_START_STOP        0x01
#define RANGE_BACK_TO_BACK      0x02
#define VHV_INIT_BYTE           0x40
#define INTR_NEW_SAMPLE_READY   0x04
#define INTR_STATUS_MASK        0x07
#define REF_SPAD_COUNT          48
#define FIRST_APERTURE_SPAD     12
#define RANGE_READ_LEN          12       /* 0x14 .. 0x1F; range at +10 */

#define POLL_STEP_US            1000u
#define WAIT_LIMIT_US           100000u

/* Budget overheads (us) from VL53L0X_get_measurement_timing_budget_micro_seconds() */
#define START_OVERHEAD_US       1910u
#define END_OVERHEAD_US         960u
#define MSRC_OVERHEAD_US        660u
#define TCC_OVERHEAD_US         590u
#define DSS_OVERHEAD_US         690u
#define PRE_RANGE_OVERHEAD_US   660u
#define FINAL_RANGE_OVERHEAD_US 550u

#define TRY(expr) do { int rc_ = (expr); if (rc_ < 0) return rc_; } while (0)

typedef struct { const umb_i2c_t *b; uint8_t addr; } io_t;
typedef struct { uint8_t reg, val; } reg_val_t;

typedef struct { bool tcc, msrc, dss, pre_range, final_range; } seq_enables_t;
typedef struct {
    uint8_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_timeouts_t;

/* DefaultTuningSettings from ST's vl53l0x_tuning.h (0xFF selects the page). */
static const reg_val_t TUNING[] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
    {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
    {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
    {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
    {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
    {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
    {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
    {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
    {0xFF, 0x00}, {0x80, 0x00},
};

/* "Set I2C standard mode" and open the page holding the stop variable (0x91). */
static const reg_val_t STOP_VAR_OPEN[] = {
    {0x88, 0x00}, {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00},
};
static const reg_val_t STOP_VAR_CLOSE[] = { {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00} };

static const reg_val_t SPAD_INFO_OPEN[] = {
    {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x06},
};
static const reg_val_t SPAD_INFO_REQUEST[] = {
    {0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01}, {0x94, 0x6B}, {0x83, 0x00},
};
static const reg_val_t SPAD_INFO_CLOSE[] = {
    {0xFF, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
};

#define LEN(a) (sizeof (a) / sizeof (a)[0])

/* ---- register access ---------------------------------------------------- */

static int bus_rc(int rc) { return rc < 0 ? VL53L0X_ERR_BUS : 0; }

static int wr8(io_t io, uint8_t reg, uint8_t val) {
    return bus_rc(umb_i2c_write_reg8(io.b, io.addr, reg, val));
}

static int wr16(io_t io, uint8_t reg, uint16_t val) {
    const uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return bus_rc(io.b->write(io.b->ctx, io.addr, buf, sizeof buf, false));
}

static int wr_spad_map(io_t io, const uint8_t map[6]) {
    uint8_t buf[7] = { REG_SPAD_ENABLES_REF_0 };
    for (int i = 0; i < 6; i++) buf[i + 1] = map[i];
    return bus_rc(io.b->write(io.b->ctx, io.addr, buf, sizeof buf, false));
}

static int rd(io_t io, uint8_t reg, uint8_t *out, size_t n) {
    return bus_rc(umb_i2c_read_regs(io.b, io.addr, reg, out, n));
}

static int rd8(io_t io, uint8_t reg, uint8_t *out) { return rd(io, reg, out, 1); }

static int rd16(io_t io, uint8_t reg, uint16_t *out) {
    uint8_t buf[2];
    TRY(rd(io, reg, buf, 2));
    *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

static int update8(io_t io, uint8_t reg, uint8_t and_mask, uint8_t or_mask) {
    uint8_t v;
    TRY(rd8(io, reg, &v));
    return wr8(io, reg, (uint8_t)((v & and_mask) | or_mask));
}

static int wr_seq(io_t io, const reg_val_t *seq, size_t n) {
    for (size_t i = 0; i < n; i++) TRY(wr8(io, seq[i].reg, seq[i].val));
    return 0;
}

/* Poll reg until (reg & mask) != 0; at most WAIT_LIMIT_US of delay_us. */
static int wait_set(io_t io, uint8_t reg, uint8_t mask) {
    for (uint32_t waited = 0;; waited += POLL_STEP_US) {
        uint8_t v;
        TRY(rd8(io, reg, &v));
        if (v & mask) return 0;
        if (waited >= WAIT_LIMIT_US) return VL53L0X_ERR_TIMEOUT;
        io.b->delay_us(io.b->ctx, POLL_STEP_US);
    }
}

/* ---- timeout math (ST API helpers) -------------------------------------- */

static uint8_t decode_vcsel_period(uint8_t reg_val) { return (uint8_t)((reg_val + 1) << 1); }

/* Macro period in ns: PLL period 1655 ps, 2304 VCSEL clocks per macro period. */
static uint32_t macro_period_ns(uint8_t vcsel_pclks) {
    return ((2304u * vcsel_pclks * 1655u) + 500u) / 1000u;
}

static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel_pclks) {
    return ((mclks * macro_period_ns(vcsel_pclks)) + 500u) / 1000u;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel_pclks) {
    uint32_t period = macro_period_ns(vcsel_pclks);
    return ((us * 1000u) + (period / 2u)) / period;
}

/* Register format: LSByte * 2^MSByte + 1 */
static uint16_t decode_timeout(uint16_t reg_val) {
    return (uint16_t)(((reg_val & 0x00FFu) << ((reg_val & 0xFF00u) >> 8)) + 1u);
}

static uint16_t encode_timeout(uint32_t mclks) {
    if (mclks == 0) return 0;
    uint32_t ls_byte = mclks - 1u;
    uint16_t ms_byte = 0;
    while (ls_byte & 0xFFFFFF00u) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFFu));
}

static int read_enables(io_t io, seq_enables_t *e) {
    uint8_t cfg;
    TRY(rd8(io, REG_SYSTEM_SEQUENCE_CONFIG, &cfg));
    *e = (seq_enables_t){
        .tcc = (cfg >> 4) & 1, .dss = (cfg >> 3) & 1, .msrc = (cfg >> 2) & 1,
        .pre_range = (cfg >> 6) & 1, .final_range = (cfg >> 7) & 1,
    };
    return 0;
}

static int read_timeouts(io_t io, const seq_enables_t *e, seq_timeouts_t *t) {
    uint8_t pre_vcsel, final_vcsel, msrc;
    uint16_t pre_reg, final_reg;
    TRY(rd8(io, REG_PRE_RANGE_VCSEL_PERIOD, &pre_vcsel));
    TRY(rd8(io, REG_MSRC_CONFIG_TIMEOUT_MACROP, &msrc));
    TRY(rd16(io, REG_PRE_RANGE_TIMEOUT_MACROP_HI, &pre_reg));
    TRY(rd8(io, REG_FINAL_RANGE_VCSEL_PERIOD, &final_vcsel));
    TRY(rd16(io, REG_FINAL_RANGE_TIMEOUT_MACROP_HI, &final_reg));

    seq_timeouts_t r = {
        .pre_range_vcsel_pclks = decode_vcsel_period(pre_vcsel),
        .final_range_vcsel_pclks = decode_vcsel_period(final_vcsel),
        .msrc_dss_tcc_mclks = (uint16_t)(msrc + 1),
        .pre_range_mclks = decode_timeout(pre_reg),
        .final_range_mclks = decode_timeout(final_reg),
    };
    if (e->pre_range) r.final_range_mclks = (uint16_t)(r.final_range_mclks - r.pre_range_mclks);
    r.msrc_dss_tcc_us = mclks_to_us(r.msrc_dss_tcc_mclks, r.pre_range_vcsel_pclks);
    r.pre_range_us = mclks_to_us(r.pre_range_mclks, r.pre_range_vcsel_pclks);
    r.final_range_us = mclks_to_us(r.final_range_mclks, r.final_range_vcsel_pclks);
    *t = r;
    return 0;
}

/* Budget used by every enabled step except the final-range timeout itself. */
static uint32_t fixed_budget_us(const seq_enables_t *e, const seq_timeouts_t *t) {
    uint32_t us = START_OVERHEAD_US + END_OVERHEAD_US;
    if (e->tcc) us += t->msrc_dss_tcc_us + TCC_OVERHEAD_US;
    if (e->dss) us += 2u * (t->msrc_dss_tcc_us + DSS_OVERHEAD_US);
    else if (e->msrc) us += t->msrc_dss_tcc_us + MSRC_OVERHEAD_US;
    if (e->pre_range) us += t->pre_range_us + PRE_RANGE_OVERHEAD_US;
    if (e->final_range) us += FINAL_RANGE_OVERHEAD_US;
    return us;
}

/* getMeasurementTimingBudget() */
static int get_budget(io_t io, uint32_t *us) {
    seq_enables_t e;
    seq_timeouts_t t;
    TRY(read_enables(io, &e));
    TRY(read_timeouts(io, &e, &t));
    *us = fixed_budget_us(&e, &t) + (e.final_range ? t.final_range_us : 0u);
    return 0;
}

/* setMeasurementTimingBudget() without the minimum check. */
static int apply_budget(vl53l0x_t *d, io_t io, uint32_t budget_us) {
    seq_enables_t e;
    seq_timeouts_t t;
    TRY(read_enables(io, &e));
    TRY(read_timeouts(io, &e, &t));
    if (!e.final_range) return 0;

    uint32_t used_us = fixed_budget_us(&e, &t);
    if (used_us > budget_us) return VL53L0X_ERR_PARAM;   /* "Requested timeout too big" */

    uint32_t final_mclks = us_to_mclks(budget_us - used_us, t.final_range_vcsel_pclks);
    if (e.pre_range) final_mclks += t.pre_range_mclks;
    TRY(wr16(io, REG_FINAL_RANGE_TIMEOUT_MACROP_HI, encode_timeout(final_mclks)));
    d->timing_budget_us = budget_us;
    return 0;
}

/* ---- init steps --------------------------------------------------------- */

/* VL53L0X_DataInit() */
static int data_init(vl53l0x_t *d, io_t io) {
    TRY(update8(io, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP, 0xFF, 0x01));   /* 2V8 I/O */
    TRY(wr_seq(io, STOP_VAR_OPEN, LEN(STOP_VAR_OPEN)));
    TRY(rd8(io, 0x91, &d->stop_variable));
    TRY(wr_seq(io, STOP_VAR_CLOSE, LEN(STOP_VAR_CLOSE)));
    /* disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
    TRY(update8(io, REG_MSRC_CONFIG_CONTROL, 0xFF, 0x12));
    TRY(wr16(io, REG_FINAL_RANGE_MIN_COUNT_RATE, SIGNAL_RATE_0_25_MCPS));
    return wr8(io, REG_SYSTEM_SEQUENCE_CONFIG, SEQ_ALL);
}

/* getSpadInfo(): reference SPAD count and type from NVM. */
static int get_spad_info(io_t io, uint8_t *count, bool *aperture) {
    uint8_t info;
    TRY(wr_seq(io, SPAD_INFO_OPEN, LEN(SPAD_INFO_OPEN)));
    TRY(update8(io, 0x83, 0xFF, 0x04));
    TRY(wr_seq(io, SPAD_INFO_REQUEST, LEN(SPAD_INFO_REQUEST)));
    TRY(wait_set(io, 0x83, 0xFF));
    TRY(wr8(io, 0x83, 0x01));
    TRY(rd8(io, 0x92, &info));
    *count = info & 0x7F;
    *aperture = (info >> 7) & 0x01;
    TRY(wr8(io, 0x81, 0x00));
    TRY(wr8(io, REG_PAGE, 0x06));
    TRY(update8(io, 0x83, (uint8_t)~0x04, 0x00));
    return wr_seq(io, SPAD_INFO_CLOSE, LEN(SPAD_INFO_CLOSE));
}

/* VL53L0X_set_reference_spads(), assuming the NVM values are valid. */
static int set_reference_spads(io_t io) {
    uint8_t count, map[6];
    bool aperture;
    TRY(get_spad_info(io, &count, &aperture));
    TRY(rd(io, REG_SPAD_ENABLES_REF_0, map, sizeof map));
    TRY(wr8(io, REG_PAGE, 0x01));
    TRY(wr8(io, REG_DYNAMIC_SPAD_REF_EN_START_OFF, 0x00));
    TRY(wr8(io, REG_DYNAMIC_SPAD_NUM_REQUESTED, 0x2C));
    TRY(wr8(io, REG_PAGE, 0x00));
    TRY(wr8(io, REG_REF_EN_START_SELECT, 0xB4));

    uint8_t first = aperture ? FIRST_APERTURE_SPAD : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < REF_SPAD_COUNT; i++) {
        uint8_t bit = (uint8_t)(1u << (i % 8));
        if (i < first || enabled == count) map[i / 8] &= (uint8_t)~bit;
        else if (map[i / 8] & bit) enabled++;
    }
    return wr_spad_map(io, map);
}

/* VL53L0X_SetGpioConfig(): interrupt on new sample ready, active low. */
static int gpio_config(io_t io) {
    TRY(wr8(io, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, INTR_NEW_SAMPLE_READY));
    TRY(update8(io, REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)~0x10, 0x00));
    return wr8(io, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

/* VL53L0X_perform_single_ref_calibration() */
static int single_ref_calibration(io_t io, uint8_t seq, uint8_t vhv_init_byte) {
    TRY(wr8(io, REG_SYSTEM_SEQUENCE_CONFIG, seq));
    TRY(wr8(io, REG_SYSRANGE_START, RANGE_START_STOP | vhv_init_byte));
    TRY(wait_set(io, REG_RESULT_INTERRUPT_STATUS, INTR_STATUS_MASK));
    TRY(wr8(io, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));
    return wr8(io, REG_SYSRANGE_START, 0x00);
}

static int init_sequence(vl53l0x_t *d, io_t io) {
    uint8_t id;
    TRY(rd8(io, REG_IDENTIFICATION_MODEL_ID, &id));
    if (id != MODEL_ID) return VL53L0X_ERR_MODEL;
    TRY(data_init(d, io));
    /* VL53L0X_StaticInit() */
    TRY(set_reference_spads(io));
    TRY(wr_seq(io, TUNING, LEN(TUNING)));
    TRY(gpio_config(io));
    uint32_t budget_us;
    TRY(get_budget(io, &budget_us));
    TRY(wr8(io, REG_SYSTEM_SEQUENCE_CONFIG, SEQ_DEFAULT));
    TRY(apply_budget(d, io, budget_us));
    /* VL53L0X_PerformRefCalibration(): VHV, then phase, then restore sequence */
    TRY(single_ref_calibration(io, SEQ_VHV_CAL, VHV_INIT_BYTE));
    TRY(single_ref_calibration(io, SEQ_PHASE_CAL, 0x00));
    return wr8(io, REG_SYSTEM_SEQUENCE_CONFIG, SEQ_DEFAULT);
}

/* ---- public API --------------------------------------------------------- */

static bool bus_ok(const umb_i2c_t *b) { return b && b->write && b->read && b->delay_us; }

int vl53l0x_init(vl53l0x_t *d, const umb_i2c_t *b, uint8_t addr) {
    if (!d || !bus_ok(b)) return VL53L0X_ERR_PARAM;
    *d = (vl53l0x_t){ .addr = addr };
    int rc = init_sequence(d, (io_t){ b, addr });
    d->ok = (rc == 0);
    return rc;
}

int vl53l0x_set_address(vl53l0x_t *d, const umb_i2c_t *b, uint8_t new_addr) {
    if (!d || !bus_ok(b)) return VL53L0X_ERR_PARAM;
    uint8_t to = new_addr & 0x7F;
    TRY(wr8((io_t){ b, d->addr }, REG_I2C_SLAVE_DEVICE_ADDRESS, to));
    d->addr = to;
    return 0;
}

int vl53l0x_set_timing_budget(vl53l0x_t *d, const umb_i2c_t *b, uint32_t us) {
    if (!d || !bus_ok(b) || us < VL53L0X_MIN_BUDGET_US) return VL53L0X_ERR_PARAM;
    return apply_budget(d, (io_t){ b, d->addr }, us);
}

int vl53l0x_start_continuous(vl53l0x_t *d, const umb_i2c_t *b) {
    if (!d || !bus_ok(b)) return VL53L0X_ERR_PARAM;
    const reg_val_t seq[] = {
        {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0x91, d->stop_variable},
        {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00},
        {REG_SYSRANGE_START, RANGE_BACK_TO_BACK},
    };
    return wr_seq((io_t){ b, d->addr }, seq, LEN(seq));
}

int vl53l0x_poll_range(vl53l0x_t *d, const umb_i2c_t *b, uint16_t *mm, uint8_t *status) {
    if (!d || !bus_ok(b) || !mm) return VL53L0X_ERR_PARAM;
    io_t io = { b, d->addr };
    uint8_t intr, res[RANGE_READ_LEN];
    TRY(rd8(io, REG_RESULT_INTERRUPT_STATUS, &intr));
    if ((intr & INTR_STATUS_MASK) == 0) return 0;
    /* assumes linearity corrective gain 1000 (default), no fractional ranging */
    TRY(rd(io, REG_RESULT_RANGE_STATUS, res, sizeof res));
    TRY(wr8(io, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));
    *mm = (uint16_t)((res[10] << 8) | res[11]);
    if (status) *status = (res[0] >> 3) & 0x0F;
    return 1;
}
