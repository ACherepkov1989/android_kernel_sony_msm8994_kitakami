/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/ipc_logging.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <soc/qcom/smsm.h>

#include "loopback_smsm.h"

static int MOCK_SMSM_NUM_ENTRIES = 8;

static int mock_smsm_inited;
#define MOCK_SMSM_IPC_PAGES		5
void *mock_smsm_ipc_log_txt;

#define MOCK_SMSM_LOG(fmt, args...) \
do { \
	if (mock_smsm_ipc_log_txt) { \
		ipc_log_string(mock_smsm_ipc_log_txt, fmt, args); \
	} \
} while (0)

/**
 * struct mock_smsm_state_cb_info - callback registration context
 * @cb_list:	list head struct to allow addition and removal to lists
 * @mask:	state mask defining when callback should be called
 * @data:	data to be passed as first argument to callback
 * @notify:	callback to be called
 */
struct mock_smsm_state_cb_info {
	struct list_head cb_list;
	uint32_t mask;
	void *data;
	void (*notify)(void *data, uint32_t old_state, uint32_t new_state);
};

/**
 * struct mock_smsm_cb_work_struct - struct used to defer callbacks to later
 * time
 * @work:	worker item used to enqueue this to a workqueue
 * @data:	data to be passed to the callback
 * @old_state:	bitmask of old state when work was queued to be passed to
 *		callback
 * @new_state:	bitmask of new state when work was queued to be passed to
 *		callback
 * @notify:	the callback
 *
 * This struct is used to defer state callbacks to another point in time.
 */
struct mock_smsm_cb_work_struct {
	struct work_struct work;
	void *data;
	uint32_t old_state;
	uint32_t new_state;
	void (*notify)(void *data, uint32_t old_state, uint32_t new_state);
};

/**
 *  struct mock_smsm_state_info - maintains the state as well as a list of
 *  callbacks
 *  @callbacks: list_head of the state callbacks
 *  @state:	bitmask of the state
 *
 *  This struct maintains the state, as well as a list of callbacks that will
 *  be called at various state changes.
 */
struct mock_smsm_state_info {
	struct list_head callbacks;
	uint32_t state;
};
static struct mock_smsm_state_info *mock_smsm_states;

static struct workqueue_struct *mock_smsm_cb_wq;
DEFINE_SPINLOCK(mock_smsm_entry_lock);
DEFINE_SPINLOCK(mock_smsm_inited_lock);

static int mock_smsm_cb_init(void);
static int mock_smsm_states_alloc(void);
static int mock_smsm_cb_wq_init(void);
static void mock_smsm_states_entries_init(void);
static void mock_smsm_state_cb_execute(uint32_t smsm_entry, uint32_t
				old_state);
static void mock_smsm_cb_handler(struct work_struct *work);
static int mock_smsm_entry_is_error(uint32_t smsm_entry);

#define DEFAULT_SSLEEP			5

static struct workqueue_struct *dl_init_wq;
static void dl_init_handler(struct work_struct *work);

/**
 * struct dl_init_struct - used to initialize downlink
 * @work: work item used for enqueueing to workqueue
 * @smsm_entry:		the smsm entry whose state should be changed
 * @clear_mask:		bitmask to clear in entry
 * @set_mask:		bitmask to set in entry
 *
 * Struct passed to dl_init_handler to initialize downlink.
 */
struct dl_init_struct {
	struct work_struct work;
	unsigned int sleep_secs;
	uint32_t smsm_entry;
	uint32_t clear_mask;
	uint32_t set_mask;
};

/**
 * mock_smsm_change_state() - mocks the call to smsm_change_state()
 * to provide loopback support
 * @smsm_entry:		smsm_entry
 * @clear_mask:		bits of entry to clear
 * @set_mask:		bits of entry to set
 *
 * Mocks the call to smsm_change_state. First applies the clear mask and the
 * set mask to the smsm_entry specified, calling any applicable state
 * callbacks that may have been triggered.
 *
 * Return: 0 on success, error code otherwise
 */
int mock_smsm_change_state(uint32_t smsm_entry,
	uint32_t clear_mask, uint32_t set_mask)
{
	uint32_t old_state;
	int err = 0;
	unsigned long flags;

	err = mock_smsm_entry_is_error(smsm_entry);
	if (err) {
		MOCK_SMSM_LOG("%s: invalid smsm_entry=%u\n", __func__,
				smsm_entry);
		goto exit;
	}

	spin_lock_irqsave(&mock_smsm_entry_lock, flags);
	old_state = mock_smsm_states[smsm_entry].state;
	mock_smsm_states[smsm_entry].state &= (~clear_mask);
	mock_smsm_states[smsm_entry].state |= set_mask;
	mock_smsm_state_cb_execute(smsm_entry, old_state);
	spin_unlock_irqrestore(&mock_smsm_entry_lock, flags);
exit:
	return err;
}

/**
 * mock_smsm_get_state() - mocks the call to smsm_get_state()
 * to provide loopback support
 * @smsm_entry:		target smsm_entry
 *
 * Mocks the call to smsm_get_state() to provide loopback support. Searches for
 * the specified smsm entry and returns its state.
 *
 * Return: bitmask of state for input enty
 */
uint32_t mock_smsm_get_state(uint32_t smsm_entry)
{
	uint32_t state;
	int err = 0;
	unsigned long flags;

	err = mock_smsm_entry_is_error(smsm_entry);
	if (err) {
		MOCK_SMSM_LOG("%s: invalid smsm_entry=%u\n", __func__,
				smsm_entry);
		pr_err("mock_smsm_get_state: Invalid entry %d",
			smsm_entry);
		return err;
	}

	spin_lock_irqsave(&mock_smsm_entry_lock, flags);
	state = mock_smsm_states[smsm_entry].state;
	spin_unlock_irqrestore(&mock_smsm_entry_lock, flags);
	return state;
}

/**
 * mock_smsm_state_cb_register() - mocks the call to smsm_state_cb_register()
 * to provide loopback support
 * @smsm_entry:		target smsm_entry to register
 * @mask:		bits to register
 * @notify:		callback function to register
 * @data:		data to be passed into callback
 *
 * Mocks the call to smsm_state_cb_register() to provide loopback support.
 * Registers callback for the give bitmask of the given smsm_entry.
 *
 * Return: Error code, 0 - inserted new entry, or 1 - updated mask of existing
 * entry
 */
int mock_smsm_state_cb_register(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t old_state, uint32_t new_state),
	void *data)
{
	struct mock_smsm_state_info *state;
	struct mock_smsm_state_cb_info *cb_info;
	struct mock_smsm_state_cb_info *cb_found = 0;
	int ret = 0, err = 0;
	unsigned long flags;

	if (err) {
		MOCK_SMSM_LOG("%s: smsm_entry error\n", __func__);
		return err;
	}

	spin_lock_irqsave(&mock_smsm_entry_lock, flags);

	state = &mock_smsm_states[smsm_entry];
	list_for_each_entry(cb_info,
			&state->callbacks, cb_list) {
		if (!ret && (cb_info->notify == notify) &&
				(cb_info->data == data)) {
			cb_info->mask |= mask;
			cb_found = cb_info;
			ret = 1;
		}
	}

	if (!cb_found) {
		cb_info = kmalloc(sizeof(struct mock_smsm_state_cb_info),
			GFP_ATOMIC);
		if (!cb_info) {
			ret = -ENOMEM;
			MOCK_SMSM_LOG("%s: cb_info alloc fail, entry %u\n",
					__func__, smsm_entry);
			goto cleanup;
		}

		cb_info->mask = mask;
		cb_info->notify = notify;
		cb_info->data = data;
		INIT_LIST_HEAD(&cb_info->cb_list);
		list_add_tail(&cb_info->cb_list,
			&state->callbacks);
	}

cleanup:
	spin_unlock_irqrestore(&mock_smsm_entry_lock, flags);
	return ret;
}

/**
 * mock_smsm_state_cb_deregister() - mocks the call to
 * smsm_state_cb_deregister() to provide loopback support
 * @smsm_entry:		target smsm_entry to deregister
 * @mask:		bits to deregister(if result is 0, callback is removed)
 * @notify:		callback function to deregister
 * @data:		data passed to callback
 *
 * Mocks the call to smsm_state_cb_deregister() to provide loopback support.
 * Deregisters the callback for the mask of the given entry.
 *
 * Return: error code, 0 - not found, 1 - updated mask, or 2 - removed callback
 */
int mock_smsm_state_cb_deregister(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t, uint32_t), void *data)
{
	struct mock_smsm_state_cb_info *cb_info;
	struct mock_smsm_state_cb_info *cb_tmp;
	struct mock_smsm_state_info *state;
	int ret = 0, err = 0;
	unsigned long flags;

	err = mock_smsm_entry_is_error(smsm_entry);
	if (err) {
		MOCK_SMSM_LOG("%s: Mock SMSM invalid entry\n", __func__);
		return err;
	}

	spin_lock_irqsave(&mock_smsm_entry_lock, flags);

	state = &mock_smsm_states[smsm_entry];
	list_for_each_entry_safe(cb_info, cb_tmp,
			&state->callbacks, cb_list) {
		if (!ret && (cb_info->notify == notify) &&
				(cb_info->data == data)) {
			cb_info->mask &= ~mask;
			ret = 1;
			if (!cb_info->mask) {
				/* no mask bits set, remove callback */
				list_del(&cb_info->cb_list);
				kfree(cb_info);
				ret = 2;
				continue;
			}
		}
	}

	spin_unlock_irqrestore(&mock_smsm_entry_lock, flags);

	return ret;
}

/**
 * dl_init_handler - handler for initializing downlink
 * @work:	work struct used to enqueue work
 *
 * Gathers the dl_init_workstruct and sets the state specified to initialize
 * downlink.
 */
static void dl_init_handler(struct work_struct *work)
{
	struct dl_init_struct *init_struct;
	if (work == NULL) {
		MOCK_SMSM_LOG("%s: work struct null\n", __func__);
		return;
	}
	init_struct = container_of(work, struct dl_init_struct, work);
	if (init_struct == NULL) {
		MOCK_SMSM_LOG("%s: dl_init struct null\n", __func__);
		return;
	}
	ssleep(init_struct->sleep_secs);
	mock_smsm_change_state(init_struct->smsm_entry,
			init_struct->clear_mask, init_struct->set_mask);
	kfree(init_struct);
}

/**
 * mock_smsm_init() - initializes all of the mock_smsm functionality
 *
 * Initialized the mock_smsm functionality, such as workqueues and ipc
 * logging.
 *
 * Return: 0 on success, error code otherwise
 */
int mock_smsm_init(void)
{
	int ret = 0;
	unsigned long flags;
	struct dl_init_struct *init_struct;

	mock_smsm_ipc_log_txt = ipc_log_context_create(MOCK_SMSM_IPC_PAGES,
			"mock_smsm", 0);

	ret = mock_smsm_cb_init();
	if (ret) {
		pr_err("%s: mock_smsm_cb_init fail:%d\n", __func__, ret);
		goto exit;
	}

	init_struct = (struct dl_init_struct *)
			kmalloc(sizeof(struct dl_init_struct),
			GFP_KERNEL);
	if (init_struct) {
		INIT_WORK(&(init_struct->work), dl_init_handler);
		init_struct->sleep_secs = DEFAULT_SSLEEP;
		init_struct->smsm_entry = SMSM_MODEM_STATE;
		init_struct->clear_mask = 0;
		init_struct->set_mask = SMSM_A2_POWER_CONTROL;
		queue_work(dl_init_wq, &(init_struct->work));
	} else {
		pr_err("%s: init struct alloc fail\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	spin_lock_irqsave(&mock_smsm_inited_lock, flags);
	mock_smsm_inited = 1;
	spin_unlock_irqrestore(&mock_smsm_inited_lock, flags);

exit:
	return ret;
}

/**
 * mock_smsm_cb_init() - initializes the callback functionality for the
 * mock_smsm portion of loopback
 *
 * Initializes the state callbacks for the mock smsm.
 *
 * Return: 0 on success, error code otherwise
 */
static int mock_smsm_cb_init(void)
{
	int ret = 0;

	ret = mock_smsm_states_alloc();
	if (ret) {
		MOCK_SMSM_LOG("%s: Mock SMSM state alloc failed\n", __func__);
		goto exit;
	}

	ret = mock_smsm_cb_wq_init();
	if (ret) {
		MOCK_SMSM_LOG("%s:Mock SMSM wq init failed\n", __func__);
		kfree(mock_smsm_states);
		goto exit;
	}

	mock_smsm_states_entries_init();

exit:
	return ret;
}

/**
 * mock_smsm_states_alloc() - allocates memory for mock_smsm_states
 *
 * Allocates memory for mock_smsm_states.
 *
 * Return: 0 on success, error code otherwise
 */
static int mock_smsm_states_alloc(void)
{
	int ret = 0;

	mock_smsm_states = kmalloc(sizeof(struct mock_smsm_state_info) *
			MOCK_SMSM_NUM_ENTRIES, GFP_ATOMIC);
	if (!mock_smsm_states) {
		MOCK_SMSM_LOG("%s:Mock SMSM state alloc failed\n", __func__);
		pr_err("%s:Mock SMSM state alloc failed\n", __func__);
		ret = -ENOMEM;
	}

	return ret;
}

/**
 * mock_smsm_cb_wq_init() - initializes the callback workqueue
 *
 * Initializes the workqueue where the callbacks will be handled.
 *
 * Return: 0 on success, error code otherwise
 */
static int mock_smsm_cb_wq_init(void)
{
	int ret = 0;

	mock_smsm_cb_wq = create_singlethread_workqueue("mock_smsm_cb_wq");
	if (!mock_smsm_cb_wq) {
		MOCK_SMSM_LOG("%s: mock_smsm_cb_wq creation failed\n",
				__func__);
		pr_err("%s: mock_smsm_cb_wq creation failed\n", __func__);
		ret = -EFAULT;
		goto exit;
	}

	dl_init_wq = create_singlethread_workqueue("dl_init_wq");
	if (!dl_init_wq) {
		MOCK_SMSM_LOG("%s: dl_init_wq creation failed\n",
				__func__);
		pr_err("%s: dl_init_wq creation failed\n", __func__);
		ret = -EFAULT;
	}

exit:
	return ret;
}

/**
 * mock_smsm_state_entries_init() - initializes each smsm_state entry
 *
 * Initializes each smsm entry.
 */
static void mock_smsm_states_entries_init(void)
{
	struct mock_smsm_state_info *mock_state_info;
	int n;
	unsigned long flags;

	spin_lock_irqsave(&mock_smsm_entry_lock, flags);
	for (n = 0; n < MOCK_SMSM_NUM_ENTRIES; n++) {
		mock_state_info = &mock_smsm_states[n];
		mock_state_info->state = 0x0;
		INIT_LIST_HEAD(&mock_state_info->callbacks);
	}
	spin_unlock_irqrestore(&mock_smsm_entry_lock, flags);
}

/**
 * mock_smsm_state_cb_execute() - executes smsm_entry's callbacks
 * corresponding to the change from old_state to its current state
 * @smsm_entry:		entry whose state was changed
 * @old_state:		previous state of smsm_entry
 *
 * Executes smsm_entry's callbacks corresponding to the change from its old
 * state to its current state. Note, must be called with mock_smsm_entry_lock
 * held, and entry validated.
 */
static void mock_smsm_state_cb_execute(uint32_t smsm_entry,
	 uint32_t old_state)
{
	struct mock_smsm_state_cb_info *cb_info;
	struct mock_smsm_state_info *state;
	struct mock_smsm_cb_work_struct *cb_work_struct;
	uint32_t new_state, changed_state;

	state = &mock_smsm_states[smsm_entry];
	new_state = state->state;
	changed_state = old_state ^ new_state;

	list_for_each_entry(cb_info, &state->callbacks, cb_list) {
		if (cb_info->mask & changed_state) {
			MOCK_SMSM_LOG("%s: entry %u caused callback\n",
					__func__, smsm_entry);
			cb_work_struct = (
				struct mock_smsm_cb_work_struct *)(kmalloc(
				sizeof(struct mock_smsm_cb_work_struct),
								GFP_ATOMIC));

			if (cb_work_struct) {
				INIT_WORK(&(cb_work_struct->work),
					mock_smsm_cb_handler);
				cb_work_struct->notify = cb_info->notify;
				cb_work_struct->data = cb_info->data;
				cb_work_struct->old_state = old_state;
				cb_work_struct->new_state = new_state;
				queue_work(mock_smsm_cb_wq,
					&(cb_work_struct->work));
			} else {
				MOCK_SMSM_LOG("%s: work struct alloc fail\n",
						__func__);
			}
		}
	}
}

/**
 * mock_smsm_cb_handler() - handler function for callbacks for the mock_smsm
 * @wrk:	work_struct pointer passed to handler by convention
 *
 * Handler function responsible for deferring the work of the state callbacks
 */
static void mock_smsm_cb_handler(struct work_struct *wrk)
{
	struct mock_smsm_cb_work_struct *cb_work_struct =
		container_of(wrk, struct mock_smsm_cb_work_struct, work);

	if (!cb_work_struct) {
		MOCK_SMSM_LOG("%s: work struct null\n", __func__);
		return;
	}

	if (!cb_work_struct->notify) {
		MOCK_SMSM_LOG("%s: notify null\n", __func__);
		kfree(cb_work_struct);
		return;
	}

	(*(cb_work_struct->notify))(cb_work_struct->data,
		cb_work_struct->old_state, cb_work_struct->new_state);
	kfree(cb_work_struct);
}

/**
 * mock_smsm_entry_is_error() - ensures that mock_smsm is
 * inited at that smsm_entry is valid.
 * @smsm_entry:		entry to be validated
 *
 * Ensures that there is no error with the specified smsm_entry.
 *
 * Return: 0 if valid entry, error code otherwise
 */
static int mock_smsm_entry_is_error(uint32_t smsm_entry)
{
	int err = 0;
	unsigned long flags;

	spin_lock_irqsave(&mock_smsm_inited_lock, flags);
	if (!mock_smsm_inited) {
		err = -ENODEV;
		spin_unlock_irqrestore(&mock_smsm_inited_lock, flags);
		MOCK_SMSM_LOG("%s: mock smsm not inited\n", __func__);
		goto exit;
	}
	spin_unlock_irqrestore(&mock_smsm_inited_lock, flags);

	if (smsm_entry >= MOCK_SMSM_NUM_ENTRIES) {
		err = -EINVAL;
		MOCK_SMSM_LOG("%s: entry=%u max=%u\n", __func__, smsm_entry,
				MOCK_SMSM_NUM_ENTRIES);
	}

exit:
	return err;
}
