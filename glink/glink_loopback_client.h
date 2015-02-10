/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#ifndef GLINK_MOCK_XPRT_H
#define GLINK_LOOPBACK_CLIENT_H

#include <soc/qcom/glink.h>
#include "glink_loopback_commands.h"

#define GLINK_LL_CLNT_INFO(x...) do {                              \
	if (glink_get_debug_mask() & QCOM_GLINK_INFO) \
			GLINK_IPC_LOG_STR("<LL_CLNT> " x);  \
} while (0)

#define GLINK_LL_CLNT_DBG(x...) do {                              \
	if (glink_get_debug_mask() & QCOM_GLINK_DEBUG) \
			GLINK_IPC_LOG_STR("<LL_CLNT> " x);  \
} while (0)

#define GLINK_LL_CLNT_ERR(x...) do {                              \
	if (!(glink_get_debug_mask() & QCOM_GLINK_PERF)) \
		pr_err("<LL_CLNT> " x); \
	GLINK_IPC_LOG_STR("<LL_CLNT> " x);  \
} while (0)

#define GLINK_LL_CLNT_INFO_PERF(x...) do {                              \
	if (glink_get_debug_mask() & (QCOM_GLINK_INFO | QCOM_GLINK_PERF)) \
			GLINK_IPC_LOG_STR("<LL_CLNT> " x);  \
} while (0)

#define GLINK_LL_CLNT_PERF(x...) do {                              \
	if (glink_get_debug_mask() & QCOM_GLINK_PERF) \
			GLINK_IPC_LOG_STR("<LL_CLNT> " x);  \
} while (0)

/**
 * enum xprts - Representation of the supported loopback transports
 * @LOCAL_LOOPBACK_XPRT:	The local loopback transport
 * @REMOTE_MPSS_SMEM_XPRT:	The remote loopback SMEM transport to the modem
 * @REMOTE_WCNSS_LOOPBACK_XPRT:	The remote loopback transport to the WCNSS
 * @MAX_NUM_XPRT:		The maximum number of supported loopback
 *				transports
 */
enum xprts {
	LOCAL_LOOPBACK_XPRT = 0,
	REMOTE_MPSS_SMEM_XPRT,
	REMOTE_WCNSS_LOOPBACK_XPRT,
	MAX_NUM_XPRTS,
};

/**
 * struct ch_info - Channel information structure for associated control and
 *		data channels
 * @ch_cntl_name:	Name of the control channel
 * @ch_data_name:	Name of the data channel
 * @edge:		Name of the edge
 * @xprt:		Name of the transport
 */
struct ch_info {
	char *ch_cntl_name;
	char *ch_data_name;
	char *edge;
	char *xprt;
};

/**
 * struct ut_notify_data - Callback data notification structure
 * @rx_notify:		Flag denoting whether the RX notification callback has
 *			executed
 * @size:		Size of @tx_data or @rx_data
 * @tx_done:		Set to true when a data transmit has finished
 * @send_intent:	Flag indicating whether the intent associated with a
 *			remote RX intent request can be sent
 * @intent_cb_ntfy:	Flag indicating whether the remote RX intent request
 *			callback has executed
 * @event:		The event associated with this callback structure
 * @tx_data:		Transmit data
 * @tx_vbuf_provider:	For the vector transmit mode, a virtual buffer provider
 *			function
 * @tx_pbuf_provider:	For the vector transmit mode, a physical buffer provider
 *			function
 * @rx_data:		Receive data
 * @rx_vbuf_provider:	For the vector receive mode, a virtual buffer provider
 *			function
 * @rx_pbuf_provider:	For the vector receive mode, a physical buffer provider
 *			function
 * @cb_completion:	Multipurpose completion used in testing
 * @old_sigs:		Old TIOCM signals data
 * @new_sigs:		New TIOCM signals data
 */
struct ut_notify_data {
	bool rx_notify;
	size_t size;
	bool tx_done;
	bool send_intent;
	bool intent_cb_ntfy;
	unsigned event;
	void *tx_data;
	void * (*tx_vbuf_provider)(void *iovec, size_t offset,
			size_t *buf_size);
	void * (*tx_pbuf_provider)(void *iovec, size_t offset,
			size_t *buf_size);
	void *rx_data;
	void * (*rx_vbuf_provider)(void *iovec, size_t offset,
			size_t *buf_size);
	void * (*rx_pbuf_provider)(void *iovec, size_t offset,
			size_t *buf_size);
	struct completion cb_completion;
	uint32_t old_sigs;
	uint32_t new_sigs;
};

/**
 * enum ch_type - Enumerated type representing the type of loopback channel,
 *		data or control
 * @CH_CNTL_TYPE:	Control channel
 * @CH_DATA_TYPE:	Data channel
 */
enum ch_type {
	CH_CNTL_TYPE,
	CH_DATA_TYPE,
};

/**
 * enum rx_type - Enumerated type representing the RX callback registered by
 *		glink_loopback_open()
 * @LINEAR_RX:		Register the linear RX callback (notify_rx_cb)
 * @VECTOR_RX:		Register the vector RX callback (notify_rxv_cb)
 * @LINEAR_VECTOR_RX:	Register for both the linear and vector RX callbacks
 * @LINEAR_RX_NOWAIT:	Register for the performance RX callback which does not
 *			block after each packet is received
 *			(notify_rx_nowait_cb)
 */
enum rx_type {
	LINEAR_RX,
	VECTOR_RX,
	LINEAR_VECTOR_RX,
	LINEAR_RX_NOWAIT,
};

/**
 * struct rx_done_completion - Structure for receive completions
 * @completion:		The completion
 * @rx_done_lock_lha0:	Lock for the completion
 * @orphaned:		Boolean telling whether or not the request associated
 *			with this receive completion has been orphaned
 */
struct rx_done_completion {
	struct completion completion;
	spinlock_t rx_done_lock_lha0;
	bool orphaned;
};


/**
 * struct loopback_channel - Loopback channel representation structure
 * @xprt_id:		The ID of the transport associated with this loopback
 *			channel
 * @ch_type:		The type of this loopback channel, data or control
 * @handle:		Handle returned by glink_loopback_open()
 * @open_cfg:		G-Link channel configuration passed to glink_open()
 * @ut_notify_data:	Callback notification data structure
 * @rx_reuse:		Indicates whether or not this channel should re-use RX
 *			intents
 */
struct loopback_channel {
	int xprt_id;
	int ch_type;
	void *handle;
	struct glink_open_config open_cfg;
	struct ut_notify_data cb_data;
	bool rx_reuse;
};

void cb_data_init(struct ut_notify_data *cb_data);
void cb_data_reset(struct ut_notify_data *cb_data);

/**
 * struct mt_cb_data - Callback data structure for clients that handle any
 *		echoed data after completing the transmission
 * @test_comp:		Waitqueue to signal the test completion.
 * @num_tx_pkts:	Number of packets transmitted.
 * @num_rx_pkts:	Number of packets received.
 */
struct mt_cb_data {
	wait_queue_head_t test_comp;
	atomic_t num_tx_pkts;
	atomic_t num_rx_pkts;
};

/**
 * glink_loopback_send_request() - Sends the loopback request to the loopback
 *				server
 * @handle:	Handle returned by glink_loopback_open()
 * @pkt:	Loopback request packet structure
 * @type:	Type of the loopback request
 * @req_intent:	Specifies whether or not the tx() call should block waiting
 *		for an RX intent
 *
 * This function creates and sends a loopback request to the loopback server
 * and queues an RX intent to receive the response.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_send_request(void *handle, struct req *pkt, int type,
		bool req_intent);

/**
 * glink_loopback_open() - Open the loopback channel
 * @transport:	Transport of the control or data channel
 * @edge:	Edge of the control or data channel
 * @name:	Name of the control or data channel
 * @ch_type:	Type of control or data channel
 * @rx_reuse:	Reuse RX intents
 *
 * This function is used to open the loopback channel on @transport & @edge with
 * @name and waits until the GLINK_CONNECTED event is received or a timeout is
 * reached.
 *
 * Return: Handle to loopback channel or NULL in error case
 */
void *glink_loopback_open(const char *transport, const char *edge,
			const char *name, int ch_type, int rx_type,
			bool rx_reuse);

/**
 * glink_loopback_close() - Close the loopback channel
 * @handle:	Handle returned by glink_loopback_open()
 *
 * This function is used to close the loopback channel and waits until the
 * GLINK_DISCONNECTED event is received or a timeout is reached.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_close(void *handle);

/**
 * glink_loopback_tx() - Send the data on loopback channel
 * @handle:	Handle returned by glink_loopback_open()
 * @data:	Data to send on the loopback channel
 * @size:	Size of the data to send on the loopback channel
 * @req_intent:	Remote intent request flag
 * @first_tx:	First tx since channel was opened
 * @rx_reuse:	Reuse the RX intents
 *
 * This function sends data on the loopback channel, queues an RX intent to
 * receive the loopback data, and waits until the loopback data is received
 * or a timeout is reached.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_tx(void *handle, void *data, size_t size, bool req_intent,
		      bool first_tx, bool rx_reuse);

/**
 * glink_loopback_txv() - Send data on the loopback channel in vector form
 * @handle:		Handle returned by glink_loopback_open()
 * @iovec:		Pointer to the vector
 * @size:		Size of data/vector
 * @vbuf_provider:	Client provided helper function to iterate the vector
 *			in virtual address space
 * @pbuf_provider:	Client provided helper function to iterate the vector
 *			in physical address space
 * @req_intent:		Flag indicating whether or not to request an intent
 *			from the remote side
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_txv(void *handle, void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size),
	bool req_intent);

/**
 * glink_loopback_sigs_set() - Set the signals on a loopback channel
 * @handle:	Handle returned by glink_open()
 * @sigs:	New signals to be set to the channel
 *
 * This function is used to set the signals for a loopback channel. It waits for
 * the remote loopback channel to be set with the same signals and returns the
 * callback.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_sigs_set(void *handle, uint32_t sigs);

/**
 * glink_loopback_client_init() - Basic loopback client init
 *
 * This function performs the basic initialization for the loopback
 * client.
 */
void glink_loopback_client_init(void);

/**
 * glink_loopback_tx_nowait() - Send the data on loopback channel
 *				without waiting for the echo of the data.
 * @handle:	Handle returned by glink_open()
 * @data:	Data to send on the loopback channel
 * @size:	Size of the data to send on the loopback channel
 * @req_intent:	Remote intent request flag
 * @cb_data:	Callback data to be used when the data arrives back
 *
 * This function is used to send the data on the loopback channel
 * and queue the RX intent to receive the loopback data.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_tx_nowait(void *handle, void *data, size_t size,
		      bool req_intent, struct mt_cb_data *cb_data);
#endif
