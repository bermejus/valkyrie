#pragma once

#include <sam.h>
#include <cstdint>

#define PWM_INVALID_ARGUMENT	0xFFFF

/* Definitions for PWM channel number. */
enum pwm_ch_t {
	PWM_CHANNEL_0 = 0,
	PWM_CHANNEL_1 = 1,
	PWM_CHANNEL_2 = 2,
	PWM_CHANNEL_3 = 3
};

enum pwm_align_t {
	PWM_ALIGN_LEFT = (0 << 8),	/* The period is left aligned. */
	PWM_ALIGN_RIGHT = (1 << 8)	/* The period is right aligned. */
};

/* Definitions for PWM level. */
enum pwm_level_t {
	PWM_LOW = 0,	/* Low level */
	PWM_HIGH = 1,	/* High level */
	PWM_HIGHZ		/* High Impedance */
};

/* Input parameters when initializing PWM */
struct pwm_clock_t {
	/* Frequency of clock A in Hz (set 0 to turn it off) */
	uint32_t ul_clka;
	/* Frequency of clock B in Hz (set 0 to turn it off) */
	uint32_t ul_clkb;
	/* Frequency of master clock in Hz */
	uint32_t ul_mck;
};

/* Definitions for PWM channels used by motor stepper */
enum pwm_stepper_motor_pair_t {
	PWM_STEPPER_MOTOR_CH_0_1 = 0,	/* Channel 0 and 1 */
	PWM_STEPPER_MOTOR_CH_2_3 = 1,	/* Channel 2 and 3 */
	PWM_STEPPER_MOTOR_CH_4_5 = 2,	/* Channel 4 and 5 */
	PWM_STEPPER_MOTOR_CH_6_7 = 3	/* Channel 6 and 7 */
};

/* Definitions for PWM synchronous channels update mode */
enum pwm_sync_update_mode_t {
	PWM_SYNC_UPDATE_MODE_0 = PWM_SCM_UPDM_MODE0,
	PWM_SYNC_UPDATE_MODE_1 = PWM_SCM_UPDM_MODE1,
	PWM_SYNC_UPDATE_MODE_2 = PWM_SCM_UPDM_MODE2
};

/* Definitions for PWM event */
enum pwm_counter_event_t {
	PWM_EVENT_PERIOD_END = (0 << 10),		/* The channel counter event occurs at the end of the PWM period. */
	PWM_EVENT_PERIOD_HALF_END = (1 << 10)	/* The channel counter event occurs at the half of the PWM period. */
};

/* Definitions for PWM fault input ID */
enum pwm_fault_id_t {
	PWM_FAULT_PWMC0_PWMFI0 = (1 << 0),
	PWM_FAULT_PWMC0_PWMFI1 = (1 << 1),
	PWM_FAULT_PWMC0_PWMFI2 = (1 << 2),
	PWM_FAULT_PWMC1_PWMFI0 = (1 << 0),
	PWM_FAULT_PWMC1_PWMFI1 = (1 << 1),
	PWM_FAULT_PWMC1_PWMFI2 = (1 << 2),
	PWM_FAULT_MAINOSC = (1 << 3),
	PWM_FAULT_AFEC0 = (1 << 4),
	PWM_FAULT_AFEC1 = (1 << 5),
	PWM_FAULT_ACC = (1 << 6),
	PWM_FAULT_TIMER_0 = (1 << 7),
	PWM_FAULT_TIMER_1 = (1 << 7)
};

/* Definitions of PWM register group */
enum pwm_protect_reg_group_t {
	PWM_GROUP_CLOCK = (1 << 0),
	PWM_GROUP_DISABLE = (1 << 1),
	PWM_GROUP_MODE = (1 << 2),
	PWM_GROUP_PERIOD = (1 << 3),
	PWM_GROUP_DEAD_TIME = (1 << 4),
	PWM_GROUP_FAULT = (1 << 5)
};

/* Definitions for PWM comparison interrupt */
enum pwm_cmp_interrupt_t {
	PWM_CMP_MATCH = 8,   /* Comparison unit match */
	PWM_CMP_UPDATE = 16  /* Comparison unit update */
};

/* Definitions for PWM comparison unit */
enum pmc_cmp_unit_t {
	PWM_CMP_UNIT_0 = 0,
	PWM_CMP_UNIT_1 = 1,
	PWM_CMP_UNIT_2 = 2,
	PWM_CMP_UNIT_3 = 3,
	PWM_CMP_UNIT_4 = 4,
	PWM_CMP_UNIT_5 = 5,
	PWM_CMP_UNIT_6 = 6,
	PWM_CMP_UNIT_7 = 7
};

/* Definitions for PWM PDC transfer request mode */
enum pwm_pdc_request_mode_t {
	PWM_PDC_UPDATE_PERIOD_ELAPSED = (0 << 20),  /* PDC transfer request is set as soon as the update period elapses. */
	PWM_PDC_COMPARISON_MATCH = (1 << 20)  		/* PDC transfer request is set as soon as the selected comparison matches. */
};

/* Definitions for PWM PDC transfer interrupt */
enum pwm_pdc_interrupt_t {
	PWM_PDC_TX_END = (1 << 1),   /* PDC Tx end */
	PWM_PDC_TX_EMPTY = (1 << 2)  /* PDC Tx buffer empty */
};

/* Definitions for PWM synchronous channels interrupt */
enum pwm_sync_interrupt_t {
	PWM_SYNC_WRITE_READY = (1 << 0),  /* Write Ready for Synchronous Channels Update */
	PWM_SYNC_UNDERRUN = (1 << 3)      /* Synchronous Channels Update Underrun Error */
};

enum pwm_spread_spectrum_mode_t {
	PWM_SPREAD_SPECTRUM_MODE_TRIANGULAR = 0,
	PWM_SPREAD_SPECTRUM_MODE_RANDOM
};

enum pwm_leading_edge_blanking_mode_t {
	PWM_LEADING_EDGE1_MODE_LINC = PWM_LEBR1_PWMLFEN,
	PWM_LEADING_EDGE1_MODE_LDEC = PWM_LEBR1_PWMLREN,
	PWM_LEADING_EDGE1_MODE_HINC = PWM_LEBR1_PWMHFEN,
	PWM_LEADING_EDGE1_MODE_HDEC = PWM_LEBR1_PWMHREN,
	PWM_LEADING_EDGE2_MODE_LINC = PWM_LEBR2_PWMLFEN,
	PWM_LEADING_EDGE2_MODE_LDEC = PWM_LEBR2_PWMLREN,
	PWM_LEADING_EDGE2_MODE_HINC = PWM_LEBR2_PWMHFEN,
	PWM_LEADING_EDGE2_MODE_HDEC = PWM_LEBR2_PWMHREN
};

/* Configurations of a PWM channel output */
struct pwm_output_t {
	/* Boolean of using override output as PWMH */
	bool b_override_pwmh;
	/* Boolean of using override output as PWML */
	bool b_override_pwml;
	/* Level of override output for PWMH */
	pwm_level_t override_level_pwmh;
	/* Level of override output for PWML */
	pwm_level_t override_level_pwml;
};

/* Configurations of PWM comparison */
struct pwm_cmp_t {
	/* Comparison unit number */
	uint32_t unit;
	/* Boolean of comparison enable */
	bool b_enable;
	/* Comparison value */
	uint32_t ul_value;
	/* Comparison mode */
	bool b_is_decrementing;
	/* Comparison trigger value */
	uint32_t ul_trigger;
	/* Comparison period value */
	uint32_t ul_period;
	/* Comparison update period value */
	uint32_t ul_update_period;
	/* Boolean of generating a match pulse on PWM event line 0 */
	bool b_pulse_on_line_0;
	/* Boolean of generating a match pulse on PWM event line 1 */
	bool b_pulse_on_line_1;
};

/* Configuration of PWM fault input behaviors */
struct pwm_fault_t {
	/* Fault ID */
	pwm_fault_id_t fault_id;
	/* Polarity of fault input */
	pwm_level_t polarity;
	/* Boolean of clearing fault flag */
	bool b_clear;
	/* Boolean of fault filtering */
	bool b_filtered;
};

/* Structure of PWM write-protect information */
struct pwm_protect_t {
	/* Bitmask of PWM register group for write protect hardware status */
	uint32_t ul_hw_status;
	/* Bitmask of PWM register group for write protect software status */
	uint32_t ul_sw_status;
	/* Offset address of PWM register in which a write access has been attempted */
	uint32_t ul_offset;
};

struct pwm_channel_t {
	/* Channel number */
	uint32_t channel;
	/* Channel prescaler */
	uint32_t ul_prescaler;
    /* Channel alignment */
	pwm_align_t alignment;
    /* Channel initial polarity */
	pwm_level_t polarity;
	/* Duty Cycle Value */
	uint32_t ul_duty;
	/* Period Cycle Value */
	uint32_t ul_period;
	/* Channel counter event */
	pwm_counter_event_t counter_event;
    /* Boolean of channel dead-time generator */
	bool b_deadtime_generator;
    /* Boolean of channel dead-time PWMH output inverted */
	bool b_pwmh_output_inverted;
    /* Boolean of channel dead-time PWML output inverted */
	bool b_pwml_output_inverted;
	/* Dead-time Value for PWMH Output */
	uint16_t us_deadtime_pwmh;
	/* Dead-time Value for PWML Output */
	uint16_t us_deadtime_pwml;
	/* Channel output */
	pwm_output_t output_selection;
	/* Boolean of Synchronous Channel */
	bool b_sync_ch;
	/* Fault ID of the channel */
	pwm_fault_id_t fault_id;
	/* Channel PWMH output level in fault protection */
	pwm_level_t ul_fault_output_pwmh;
	/* Channel PWML output level in fault protection */
	pwm_level_t ul_fault_output_pwml;
	/* Spread Spectrum Value */
	uint32_t ul_spread;
	/* Spread Spectrum Mode */
	pwm_spread_spectrum_mode_t spread_spectrum_mode;
	/* Leading Edge Value */
	uint32_t ul_leading_edge_delay;
	/* Leading Edge Mode */
	pwm_leading_edge_blanking_mode_t leading_edge_blanking_mode;
	/* PPM Mode in Channel mode */
	uint32_t ul_ppm_mode;
};

uint32_t pwm_init(Pwm *p_pwm, pwm_clock_t *clock_config);
uint32_t pwm_init_clock_a(Pwm *p_pwm, pwm_clock_t *clock_config);
uint32_t pwm_init_clock_b(Pwm *p_pwm, pwm_clock_t *clock_config);
uint32_t pwm_channel_init(Pwm *p_pwm, pwm_channel_t *p_channel);
uint32_t pwm_channel_update_period(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_period);
uint32_t pwm_channel_update_duty(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_duty);
uint32_t pwm_channel_get_counter(Pwm *p_pwm, pwm_channel_t *p_channel);
void pwm_channel_enable(Pwm *p_pwm, uint32_t ul_channel);
void pwm_channel_disable(Pwm *p_pwm, uint32_t ul_channel);
uint32_t pwm_channel_get_status(Pwm *p_pwm);
uint32_t pwm_channel_get_interrupt_status(Pwm *p_pwm);
uint32_t pwm_channel_get_interrupt_mask(Pwm *p_pwm);
void pwm_channel_enable_interrupt(Pwm *p_pwm, uint32_t ul_event, uint32_t ul_fault);
void pwm_channel_disable_interrupt(Pwm *p_pwm, uint32_t ul_event, uint32_t ul_fault);

void pwm_channel_update_output(Pwm *p_pwm, pwm_channel_t *p_channel, pwm_output_t *p_output, bool b_sync);
void pwm_channel_update_dead_time(Pwm *p_pwm, pwm_channel_t *p_channel, uint16_t us_deadtime_pwmh, uint16_t us_deadtime_pwml);

uint32_t pwm_fault_init(Pwm *p_pwm, pwm_fault_t *p_fault);
uint32_t pwm_fault_get_status(Pwm *p_pwm);
pwm_level_t pwm_fault_get_input_level(Pwm *p_pwm, pwm_fault_id_t id);
void pwm_fault_clear_status(Pwm *p_pwm, pwm_fault_id_t id);

uint32_t pwm_cmp_init(Pwm *p_pwm, pwm_cmp_t *p_cmp);
uint32_t pwm_cmp_change_setting(Pwm *p_pwm, pwm_cmp_t *p_cmp);
uint32_t pwm_cmp_get_period_counter(Pwm *p_pwm, uint32_t ul_cmp_unit);
uint32_t pwm_cmp_get_update_counter(Pwm *p_pwm, uint32_t ul_cmp_unit);
void pwm_cmp_enable_interrupt(Pwm *p_pwm, uint32_t ul_sources, pwm_cmp_interrupt_t type);
void pwm_cmp_disable_interrupt(Pwm *p_pwm, uint32_t ul_sources, pwm_cmp_interrupt_t type);
uint32_t pwm_sync_init(Pwm *p_pwm, pwm_sync_update_mode_t mode, uint32_t ul_update_period);
void pwm_sync_unlock_update(Pwm *p_pwm);
void pwm_sync_change_period(Pwm *p_pwm, uint32_t ul_update_period);
uint32_t pwm_sync_get_period_counter(Pwm * p_pwm);
void pwm_sync_enable_interrupt(Pwm *p_pwm, uint32_t ul_sources);
void pwm_sync_disable_interrupt(Pwm *p_pwm, uint32_t ul_sources);

void pwm_enable_protect(Pwm *p_pwm, uint32_t ul_group, bool b_sw);
void pwm_disable_protect(Pwm *p_pwm, uint32_t ul_group);
bool pwm_get_protect_status(Pwm *p_pwm, pwm_protect_t * p_protect);

uint32_t pwm_get_interrupt_status(Pwm *p_pwm);
uint32_t pwm_get_interrupt_mask(Pwm *p_pwm);

void pwm_stepper_motor_init(Pwm *p_pwm, pwm_stepper_motor_pair_t pair, bool b_enable_gray, bool b_down);

void pwm_channel_update_spread(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_spread);
void pwm_channel_update_leading_edge(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_leading_edge_delay, pwm_leading_edge_blanking_mode_t leading_edge_blanking_mode);

void pwm_set_dma_duty(Pwm *p_pwm, uint32_t ul_dma_duty_value);
void pwm_set_ext_trigger_mode(Pwm *p_pwm, pwm_channel_t *p_channel, uint32_t ul_mode);