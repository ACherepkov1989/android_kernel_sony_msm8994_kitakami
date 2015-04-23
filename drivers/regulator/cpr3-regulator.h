/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __REGULATOR_CPR3_REGULATOR_H__
#define __REGULATOR_CPR3_REGULATOR_H__

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/power/qcom/apm.h>
#include <linux/regulator/driver.h>

struct cpr3_controller;

/**
 * struct cpr3_fuse_param - defines one contiguous segment of a fuse parameter
 *			    that is contained within a given row.
 * @row:	Fuse row number
 * @bit_start:	The first bit within the row of the fuse parameter segment
 * @bit_end:	The last bit within the row of the fuse parameter segment
 *
 * Each fuse row is 64 bits in length.  bit_start and bit_end may take values
 * from 0 to 63.  bit_start must be less than or equal to bit_end.
 */
struct cpr3_fuse_param {
	unsigned		row;
	unsigned		bit_start;
	unsigned		bit_end;
};

/* Each CPR3 sensor has 16 ring oscillators */
#define CPR3_RO_COUNT		16

/* The maximum number of sensors that can be present on a single CPR loop. */
#define CPR3_MAX_SENSOR_COUNT	256

/* This constant is used when allocating array printing buffers. */
#define MAX_CHARS_PER_INT	10

/**
 * struct cpr3_corner - CPR3 virtual voltage corner data structure
 * @floor_volt:		CPR closed-loop floor voltage in microvolts
 * @ceiling_volt:	CPR closed-loop ceiling voltage in microvolts
 * @open_loop_volt:	CPR open-loop voltage (i.e. initial voltage) in
 *			microvolts
 * @last_volt:		Last known settled CPR closed-loop voltage which is used
 *			when switching to a new corner
 * @proc_freq:		Processor frequency in Hertz (only used by platform
 *			specific CPR3 driver for interpolation)
 * @cpr_fuse_corner:	Fused corner index associated with this virtual corner
 *			(only used by platform specific CPR3 driver for
 *			mapping purposes)
 * @target_quot:	Array of target quotient values to use for each ring
 *			oscillator (RO) for this corner.  A value of 0 should be
 *			specified as the target quotient for each RO that is
 *			unused by this corner.
 * @ro_mask:		Bitmap where each of the 16 LSBs indicate if the
 *			corresponding ROs should be masked for this corner
 * @irq_en:		Bitmap of the CPR interrupts to enable for this corner
 *
 * The value of last_volt is initialized inside of the cpr3_regulator_register()
 * call with the open_loop_volt value.  It can later be updated to the settled
 * VDD supply voltage.
 *
 * The values of ro_mask and irq_en are initialized inside of the
 * cpr3_regulator_register() call.
 */
struct cpr3_corner {
	int			floor_volt;
	int			ceiling_volt;
	int			open_loop_volt;
	int			last_volt;
	u32			proc_freq;
	int			cpr_fuse_corner;
	u32			target_quot[CPR3_RO_COUNT];
	u32			ro_mask;
	u32			irq_en;
};

/**
 * struct cpr3_thread - CPR3 hardware thread data structure
 * @thread_id:		Hardware thread ID
 * @of_node:		Device node associated with the device tree child node
 *			of this CPR3 thread
 * @ctrl:		Pointer to the CPR3 controller which manages this thread
 * @rdesc:		Regulator description for this thread
 * @rdev:		Regulator device pointer for the regulator registered
 *			for this thread
 * @ldo_regulator:	Pointer to the LDO supply regulator used to manage
 *			per-cluster LDO voltage and bypass state
 * @ldo_regulator_bypass: Cached copy of the LDO regulator bypass state
 * @ldo_ret_regulator:	Pointer to the LDO retention supply regulator used to
 *			manage LDO retention bypass state
 * @name:		Unique name for this thread which is filled using the
 *			device tree regulator-name property
 * @corner:		Array of all corners supported by this thread
 * @corner_count:	The number of elements in the corner array
 * @platform_fuses:	Pointer to platform specific CPR fuse data (only used by
 *			platform specific CPR3 driver)
 * @speed_bin_fuse:	Value read from the speed bin fuse parameter
 * @cpr_rev_fuse:	Value read from the CPR fusing revision fuse parameter
 * @fuse_combo:		Platform specific enum value identifying the specific
 *			combination of fuse values found on a given chip
 * @fuse_combos_supported: The number of fuse combinations supported by the
 *			device tree configuration for this CPR thread
 * @fuse_corner_count:	Number of corners defined by fuse parameters
 * @step_volt:		Step size in microvolts between available set points
 *			of the VDD supply
 * @consecutive_up:	The number of consecutive CPR step up events needed to
 *			to trigger an up interrupt
 * @consecutive_down:	The number of consecutive CPR step down events needed to
 *			to trigger a down interrupt
 * @up_threshold:	The number CPR error steps required to generate an up
 *			event
 * @down_threshold:	The number CPR error steps required to generate a down
 *			event
 * @current_corner:	Index identifying the currently selected voltage corner
 *			for the thread or less than 0 if no corner has been
 *			requested
 * @last_closed_loop_corner: Index identifying the last voltage corner for the
 *			thread which was configured when operating in CPR
 *			closed-loop mode or less than 0 if no corner has been
 *			requested.  CPR registers are only written to when using
 *			closed-loop mode.
 * @ldo_headroom_volt:	Voltage difference in microvolts required between the
 *			VDD supply voltage and the LDO output in order for the
 *			LDO operate
 * @ldo_adjust_volt:	Voltage in microvolts used to offset margin assigned
 *			to IR drop between PMIC and CPU
 * @ldo_max_volt:	The maximum physically supported LDO voltage in
 *			microvolts
 * @ldo_mode_allowed:	Boolean which indicates if LDO mode is allowed for this
 *			CPR3 thread
 * @vreg_enabled:	Boolean defining the state of the thread's regulator
 *			within the regulator framework.
 *
 * This structure contains both configuration and runtime state data.  The
 * elements current_corner, last_closed_loop_corner, and vreg_enabled are state
 * variables.
 */
struct cpr3_thread {
	u32			thread_id;
	struct device_node	*of_node;
	struct cpr3_controller	*ctrl;
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
	struct regulator	*ldo_regulator;
	bool			ldo_regulator_bypass;
	struct regulator	*ldo_ret_regulator;
	const char		*name;
	struct cpr3_corner	*corner;
	int			corner_count;
	void			*platform_fuses;
	int			speed_bin_fuse;
	int			cpr_rev_fuse;
	int			fuse_combo;
	int			fuse_combos_supported;
	int			fuse_corner_count;
	int			step_volt;
	u32			consecutive_up;
	u32			consecutive_down;
	u32			up_threshold;
	u32			down_threshold;
	int			current_corner;
	int			last_closed_loop_corner;
	int			ldo_headroom_volt;
	int			ldo_adjust_volt;
	int			ldo_max_volt;
	bool			ldo_mode_allowed;
	bool			vreg_enabled;
};

/* Per CPR controller data */
/**
 * enum cpr3_count_mode - CPR3 controller count mode which defines the
 *		method that CPR sensor data is acquired
 * %CPR3_COUNT_MODE_ALL_AT_ONCE_MIN:	Capture all CPR sensor readings
 *					simultaneously and report the minimum
 *					value seen in successive measurements
 * %CPR3_COUNT_MODE_ALL_AT_ONCE_MAX:	Capture all CPR sensor readings
 *					simultaneously and report the maximum
 *					value seen in successive measurements
 * %CPR3_COUNT_MODE_STAGGERED:		Read one sensor at a time in a
 *					sequential fashion
 * %CPR3_COUNT_MODE_ALL_AT_ONCE_AGE:	Capture all CPR aging sensor readings
 *					simultaneously.
 */
enum cpr3_count_mode {
	CPR3_COUNT_MODE_ALL_AT_ONCE_MIN	= 0,
	CPR3_COUNT_MODE_ALL_AT_ONCE_MAX	= 1,
	CPR3_COUNT_MODE_STAGGERED	= 2,
	CPR3_COUNT_MODE_ALL_AT_ONCE_AGE	= 3,
};

/**
 * struct cpr3_controller - CPR3 controller data structure
 * @dev:		Device pointer for the CPR3 controller device
 * @name:		Unique name for the CPR3 controller
 * @cpr_ctrl_base:	Virtual address of the CPR3 controller base register
 * @fuse_base:		Virtual address of fuse row 0
 * @list:		list head used in a global cpr3-regulator list so that
 *			cpr3-regulator structs can be found easily in RAM dumps
 * @thread:		Array of CPR3 threads managed by the CPR3 controller
 * @thread_count:	Number of elements in the thread array
 * @sensor_owner:	Array of thread IDs indicating which thread owns a given
 *			CPR sensor
 * @sensor_count:	The number of CPR sensors found on the CPR loop managed
 *			by this CPR controller.  Must be equal to the number of
 *			elements in the sensor_owner array
 * @lock:		Mutex lock used to ensure mutual exclusion between
 *			all of the threads associated with the controller
 * @vdd_regulator:	Pointer to the VDD supply regulator which this CPR3
 *			controller manages
 * @vdd_limit_regulator: Pointer to the VDD supply limit regulator which is used
 *			for hardware closed-loop in order specify ceiling and
 *			floor voltage limits (platform specific)
 * @core_clk:		Pointer to the CPR3 controller core clock
 * @iface_clk:		Pointer to the CPR3 interface clock (platform specific)
 * @bus_clk:		Pointer to the CPR3 bus clock (platform specific)
 * @irq:		CPR interrupt number
 * @apm:		Handle to the array power mux (APM)
 * @apm_threshold_volt:	APM threshold voltage in microvolts
 * @apm_adj_volt:	Minimum difference between APM threshold voltage and
 *			open-loop voltage which allows the APM threshold voltage
 *			to be used as a ceiling
 * @apm_high_supply:	APM supply to configure if VDD voltage is greater than
 *			or equal to the APM threshold voltage
 * @apm_low_supply:	APM supply to configure if the VDD voltage is less than
 *			the APM threshold voltage
 * @cpr_clock_rate:	CPR reference clock frequency in Hz.
 * @sensor_time:	The time in nanoseconds that each sensor takes to
 *			perform a measurement.
 * @loop_time:		The time in nanoseconds between consecutive CPR
 *			measurements.
 * @up_down_delay_time: The time to delay in nanoseconds between consecutive CPR
 *			measurements when the last measurement recommended
 *			increasing or decreasing the vdd-supply voltage.
 *			(platform specific)
 * @idle_clocks:	Number of CPR reference clock ticks that the CPR
 *			controller waits in transitional states.
 * @step_quot_init_min:	The default minimum CPR step quotient value.  The step
 *			quotient is the number of additional ring oscillator
 *			ticks observed when increasing one step in vdd-supply
 *			output voltage.
 * @step_quot_init_max:	The default maximum CPR step quotient value.
 * @count_mode:		CPR controller count mode
 * @count_repeat:	Number of times to perform consecutive sensor
 *			measurements when using all-at-once count modes.
 * @proc_clock_throttle: Defines the processor clock frequency throttling
 *			register value to use.  This can be used to reduce the
 *			clock frequency when a power domain exits a low power
 *			mode until CPR settles at a new voltage.
 *			(platform specific)
 * @cpr_allowed_hw:	Boolean which indicates if closed-loop CPR operation is
 *			permitted for a given chip based upon hardware fuse
 *			values
 * @cpr_allowed_sw:	Boolean which indicates if closed-loop CPR operation is
 *			permitted based upon software policies
 * @supports_hw_closed_loop: Boolean which indicates if this CPR3 controller
 *			physically supports hardware closed-loop CPR operation
 * @use_hw_closed_loop:	Boolean which indicates that this controller will be
 *			using hardware closed-loop operation in place of
 *			software closed-loop operation.
 * @aggr_corner:	CPR corner containing the most recently aggregated
 *			voltage configurations which are being used currently
 * @cpr_enabled:	Boolean which indicates that the CPR controller is
 *			enabled and operating in closed-loop mode.  CPR clocks
 *			have been prepared and enabled whenever this flag is
 *			true.
 * @cpr_suspended:	Boolean which indicates that CPR has been temporarily
 *			disabled while enterring system suspend.
 * @debugfs:		Pointer to the debugfs directory of this CPR3 controller
 *
 * This structure contains both configuration and runtime state data.  The
 * elements cpr_allowed_sw, use_hw_closed_loop, aggr_corner, cpr_enabled, and
 * cpr_suspended are state variables.
 *
 * The apm* elements do not need to be initialized if the VDD supply managed by
 * the CPR3 controller does not utilize an APM.
 */
struct cpr3_controller {
	struct device		*dev;
	const char		*name;
	void __iomem		*cpr_ctrl_base;
	void __iomem		*fuse_base;
	struct list_head	list;
	struct cpr3_thread	*thread;
	int			thread_count;
	u8			*sensor_owner;
	int			sensor_count;
	struct mutex		lock;
	struct regulator	*vdd_regulator;
	struct regulator	*vdd_limit_regulator;
	struct clk		*core_clk;
	struct clk		*iface_clk;
	struct clk		*bus_clk;
	int			irq;
	struct msm_apm_ctrl_dev *apm;
	int			apm_threshold_volt;
	int			apm_adj_volt;
	enum msm_apm_supply	apm_high_supply;
	enum msm_apm_supply	apm_low_supply;
	u32			cpr_clock_rate;
	u32			sensor_time;
	u32			loop_time;
	u32			up_down_delay_time;
	u32			idle_clocks;
	u32			step_quot_init_min;
	u32			step_quot_init_max;
	enum cpr3_count_mode	count_mode;
	u32			count_repeat;
	u32			proc_clock_throttle;
	bool			cpr_allowed_hw;
	bool			cpr_allowed_sw;
	bool			supports_hw_closed_loop;
	bool			use_hw_closed_loop;
	struct cpr3_corner	aggr_corner;
	bool			cpr_enabled;
	bool			cpr_suspended;
	struct dentry		*debugfs;
};

/* Used for rounding voltages to the closest physically available set point. */
#define CPR3_ROUND(n, d) (DIV_ROUND_UP(n, d) * (d))

#define cpr3_err(cpr3_thread, message, ...) \
	pr_err("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)
#define cpr3_info(cpr3_thread, message, ...) \
	pr_info("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)
#define cpr3_debug(cpr3_thread, message, ...) \
	pr_debug("%s: " message, (cpr3_thread)->name, ##__VA_ARGS__)

/*
 * Offset subtracted from voltage corner values passed in from the regulator
 * framework in order to get internal voltage corner values.  This is needed
 * since the regulator framework treats 0 as an error value at regulator
 * registration time.
 */
#define CPR3_CORNER_OFFSET	1

#ifdef CONFIG_REGULATOR_CPR3

int cpr3_regulator_register(struct platform_device *pdev,
			struct cpr3_controller *ctrl);
int cpr3_regulator_unregister(struct cpr3_controller *ctrl);
int cpr3_regulator_suspend(struct cpr3_controller *ctrl);
int cpr3_regulator_resume(struct cpr3_controller *ctrl);

int cpr3_get_thread_name(struct cpr3_thread *thread,
			struct device_node *thread_node);
int cpr3_allocate_threads(struct cpr3_controller *ctrl, u32 min_thread_id,
			u32 max_thread_id);
int cpr3_map_fuse_base(struct cpr3_controller *ctrl,
			struct platform_device *pdev);
int cpr3_read_fuse_param(void __iomem *fuse_base_addr,
			const struct cpr3_fuse_param *param, u64 *param_value);
int cpr3_convert_open_loop_voltage_fuse(int ref_volt, int step_volt, u32 fuse,
			int fuse_len);
u64 cpr3_interpolate(u64 x1, u64 y1, u64 x2, u64 y2, u64 x);
int cpr3_parse_array_property(struct cpr3_thread *thread,
			const char *prop_name, int corner_count, int corner_sum,
			int combo_offset, u32 *out);
int cpr3_parse_common_corner_data(struct cpr3_thread *thread, int *corner_sum,
			int *combo_offset);
int cpr3_parse_u32(struct cpr3_thread *thread, const char *propname,
		       u32 *out_value, u32 value_min, u32 value_max);
int cpr3_parse_ctrl_u32(struct cpr3_controller *ctrl, const char *propname,
		       u32 *out_value, u32 value_min, u32 value_max);
int cpr3_parse_common_thread_data(struct cpr3_thread *thread);
int cpr3_parse_common_ctrl_data(struct cpr3_controller *ctrl);
int cpr3_limit_open_loop_voltages(struct cpr3_thread *thread);
void cpr3_open_loop_voltage_as_ceiling(struct cpr3_thread *thread);
void cpr3_print_quots(struct cpr3_thread *thread);
int cpr3_adjust_fused_open_loop_voltages(struct cpr3_thread *thread,
		int *fuse_volt);
int cpr3_adjust_open_loop_voltages(struct cpr3_thread *thread, int corner_sum,
		int combo_offset);
int cpr3_quot_adjustment(int ro_scale, int volt_adjust);

#else

static inline int cpr3_regulator_register(struct platform_device *pdev,
			struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_unregister(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_suspend(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_regulator_resume(struct cpr3_controller *ctrl)
{
	return -ENXIO;
}

static inline int cpr3_get_thread_name(struct cpr3_thread *thread,
			struct device_node *thread_node)
{
	return -EPERM;
}

static inline int cpr3_allocate_threads(struct cpr3_controller *ctrl,
			u32 min_thread_id, u32 max_thread_id)
{
	return -EPERM;
}

static inline int cpr3_map_fuse_base(struct cpr3_controller *ctrl,
			struct platform_device *pdev)
{
	return -ENXIO;
}

static inline int cpr3_read_fuse_param(void __iomem *fuse_base_addr,
			const struct cpr3_fuse_param *param, u64 *param_value)
{
	return -EPERM;
}

static inline int cpr3_convert_open_loop_voltage_fuse(int ref_volt,
			int step_volt, u32 fuse, int fuse_len)
{
	return -EPERM;
}

static inline u64 cpr3_interpolate(u64 x1, u64 y1, u64 x2, u64 y2, u64 x)
{
	return 0;
}

static inline int cpr3_parse_array_property(struct cpr3_thread *thread,
			const char *prop_name, int corner_count, int corner_sum,
			int combo_offset, u32 *out)
{
	return -EPERM;
}

static inline int cpr3_parse_common_corner_data(struct cpr3_thread *thread,
			int *corner_sum, int *combo_offset)
{
	return -EPERM;
}

static inline int cpr3_parse_u32(struct cpr3_thread *thread,
			const char *propname, u32 *out_value, u32 value_min,
			u32 value_max)
{
	return -EPERM;
}

static inline int cpr3_parse_ctrl_u32(struct cpr3_controller *ctrl,
			const char *propname, u32 *out_value, u32 value_min,
			u32 value_max)
{
	return -EPERM;
}

static inline int cpr3_parse_common_thread_data(struct cpr3_thread *thread)
{
	return -EPERM;
}

static inline int cpr3_parse_common_ctrl_data(struct cpr3_controller *ctrl)
{
	return -EPERM;
}

static inline int cpr3_limit_open_loop_voltages(struct cpr3_thread *thread)
{
	return -EPERM;
}

static inline void cpr3_open_loop_voltage_as_ceiling(struct cpr3_thread *thread)
{
	return;
}

static inline void cpr3_print_quots(struct cpr3_thread *thread)
{
	return;
}

static inline int cpr3_adjust_fused_open_loop_voltages(
		struct cpr3_thread *thread, int *fuse_volt)
{
	return -EPERM;
}

static inline int cpr3_adjust_open_loop_voltages(struct cpr3_thread *thread,
		int corner_sum, int combo_offset)
{
	return -EPERM;
}

static inline int cpr3_quot_adjustment(int ro_scale, int volt_adjust)
{
	return 0;
}

#endif /* CONFIG_REGULATOR_CPR3 */

#endif /* __REGULATOR_CPR_REGULATOR_H__ */
