/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB Device Controller (UDC) driver for NCT family devices
 * Based on usb_dc_nct.c implementation adapted for UDC API.
 */

#include "udc_common.h"

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/dt-bindings/usb/usb.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_nct, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT nuvoton_nct_usbd

/* Timeout for USB PHY clock ready (Unit: ms) */
#define NCT_USB_PHY_TIMEOUT			(50)

/* Timeout for USB Write Data to Host (Unit: ms) */
#define NCT_USB_WRITE_TIMEOUT			(200)

/* move data between USB DATA FIFO and user RAM */
#define UDC_NCT_DMA_ENABLE

/* defined only for debug ISR */
// #define DBG_USBD_IRQ

/* defined only for debug BULK_IN */
// #define DBG_USBD_BULK_IN

/* Endpoint index enum */
enum {
	EPA = 0,
	EPB,
	EPC,
	EPD,
	EPE,
	EPF,
	EPG,
	EPH,
	EPI,
	EPJ,
	EPK,
	EPL,
	CEP = 0xFF,
};

#define NCT_EP_HW_FIRST				(EPA)
#define NCT_EP_HW_LAST				(EPL)
#define NCT_EP_CEP					(CEP)

// num_bidir_endpoints = 13 => 1 cep + 12 ep
// Example for mctp usbd
//   cfg.ep0 = cepout + cepin
//   cep.ep1 = bulk_out, 
//   cep.ep2 = bulk_in 

#define NUM_OF_EP_MAX					(DT_INST_PROP(0, num_bidir_endpoints))
#define USB_RAM_SIZE					(DT_INST_PROP(0, usbd_ram_size))
	#define CEP_BUF_BASE					(0)
	#define CEP_MAX_PKT_SIZE				(64)
	#define EP_MAX_PKT_SIZE					(1024)

#define USBD_BASE_ADDR					(DT_INST_REG_ADDR(0))
#define USBD							((struct usbd_reg *) USBD_BASE_ADDR)


#define USBD_EP_CFG_DIR_MASK			((uint32_t)0x00000008ul)
	#define USBD_EP_CFG_DIR_IN				((uint32_t)0x00000008ul)
	#define USBD_EP_CFG_DIR_OUT				((uint32_t)0x00000000ul)
#define USBD_EP_CFG_TYPE_MASK			((uint32_t)0x00000006ul)
	#define USBD_EP_CFG_TYPE_BULK			((uint32_t)0x00000002ul)
	#define USBD_EP_CFG_TYPE_INT			((uint32_t)0x00000004ul)
	#define USBD_EP_CFG_TYPE_ISO			((uint32_t)0x00000006ul)

#define USBD_SET_CEP_STATE(flag)		(USBD->USBD_CEPCTL = (flag))
#define USBD_CEPCTL_NAKCLR				((uint32_t)0x00000000ul)
#define USBD_CEPCTL_STALL				((uint32_t)0x00000002ul)
#define USBD_CEPCTL_ZEROLEN				((uint32_t)0x00000004ul)
#define USBD_CEPCTL_FLUSH				((uint32_t)0x00000008ul)


/*
 * Non-control EP RAM allocation policy
 */
#define UDC_NCT_EP_RAM_ALLOC_AVG_BY_COUNT				(0U)
#define UDC_NCT_EP_RAM_ALLOC_BULKIN_MULT_BULKOUT_MPS	(1U)

/* Default policy: CEP=64, remaining RAM split equally to 2 non-control EPs. */
#define UDC_NCT_EP_RAM_ALLOC_STRATEGY			UDC_NCT_EP_RAM_ALLOC_BULKIN_MULT_BULKOUT_MPS
#define UDC_NCT_EP_RAM_AVG_NON_CTRL_EP_COUNT	(2U)
	#define UDC_NCT_EP_RAM_MULTIPLIER				(2U)

struct udc_nct_config {
	struct usbd_reg *base;
	const struct pinctrl_dev_config *pincfg;
	struct udc_ep_config *ep_cfg_out;
	struct udc_ep_config *ep_cfg_in;
	void (*make_thread)(const struct device *dev);
	uint32_t num_bidir_endpoints;
	int speed_idx;
};

struct udc_nct_ep_data {
	volatile uint16_t mps;
	volatile uint16_t in_batch_pkts;
	volatile uint32_t ram_base;	// ep.buf_start
	volatile uint32_t ram_len;	// ep.buf_end = buf_start + buf_len - 1
	volatile uint32_t type;
};

enum udc_nct_ctrl_evt_type {
	CTRL_EVT_SETUP = 0,
	CTRL_EVT_RXPK,
	CTRL_EVT_TXPK,
	CTRL_EVT_STSDONE,
	CTRL_EVT_RESET,
};

struct udc_nct_ctrl_evt {
	uint8_t type;
	uint8_t was_status_out;
	uint16_t rx_len;
	uint16_t tx_len;	/* TXPK: bytes transmitted in this packet */
	uint32_t evt_seq;
	uint32_t setup_seq;
	struct net_buf *buf;
};

#define NCT_CTRL_EVT_Q_LEN			(16U)

struct udc_nct_data {
	struct k_thread thread_data;
	struct k_work ctrl_status_out_work;
	struct k_msgq ctrl_evt_msgq;
	const struct device *dev;
	char ctrl_evt_msgq_buffer[NCT_CTRL_EVT_Q_LEN * sizeof(struct udc_nct_ctrl_evt)];
	uint8_t setup[8];
	volatile uint8_t ctrl_out_refill_force;
	volatile uint16_t ctrl_out_queued;
	volatile uint16_t ctrl_out_feed_len;
	volatile uint32_t ram_offset;	// record usbd ram allocated
	volatile uint32_t ep_status;
	volatile uint32_t req_len;
	volatile uint32_t req_type;
	volatile uint32_t new_addr;
	volatile uint8_t ctrl_reset_pending;
	uint32_t dbg_evt_seq;
	uint32_t dbg_setup_seq;
	uint32_t dbg_flow_seq;
	uint32_t dbg_kickep_seq;
	uint32_t dbg_kickep_last_log_ms;
	uint32_t dbg_state_seq;
	uint32_t dbg_state_last_sig;
	uint32_t dbg_last_stage;
	uint32_t dbg_last_cepe;
	uint32_t dbg_last_ceps;
	uint8_t in_batch_buf[EP_MAX_PKT_SIZE];
	struct udc_nct_ep_data ep_data[NUM_OF_EP_MAX];
	struct k_spinlock dma_lock;
};

static const char *udc_nct_evt_name(uint8_t type)
{
	switch (type) {
	case CTRL_EVT_SETUP:
		return "SETUP";
	case CTRL_EVT_RXPK:
		return "RXPK";
	case CTRL_EVT_TXPK:
		return "TXPK";
	case CTRL_EVT_STSDONE:
		return "STSDONE";
	case CTRL_EVT_RESET:
		return "RESET";
	default:
		return "UNKNOWN";
	}
}

#define SETUP_PKT_SIZE sizeof(struct usb_setup_packet)

static inline const struct device *nct_udc_device_get(void)
{
	return DEVICE_DT_INST_GET(0);
}

static inline bool usbd_ep_is_enabled(uint8_t ep_idx)
{
	if (ep_idx == 0U) {
		return true;
	}

	return IS_BIT_SET(USBD->EP[ep_idx - 1].USBD_EPCFG, NCT_USBD_EPCFG_EPEN);
}

static inline void usbd_set_ep_max_payload(uint32_t ep, uint32_t size)
{
	USBD->EP[ep - 1].USBD_EPMPS = size;
}

static inline void usbd_set_ep_buf_addr(uint32_t ep, uint32_t base, uint32_t len)
{
	if (ep == NCT_EP_CEP) {
		USBD->USBD_CEPBUFSTART = base;
		USBD->USBD_CEPBUFEND = base + len - 1ul;
	} else {
		USBD->EP[ep - 1].USBD_EPBUFSTART = base;
		USBD->EP[ep - 1].USBD_EPBUFEND = base + len - 1ul;
	}
}

#define RUN_AUTO_VALIDATE_MODE 		0
#define RUN_MANUAL_VALIDATE_MODE 	1
#define RUN_FLY_MODE 				2
#define RUN_BULK_IN_MODE	 		RUN_AUTO_VALIDATE_MODE

static inline void usbd_config_ep(uint32_t ep, uint32_t ep_type, uint32_t ep_dir)
{
	// Flush OUT-PIPE FIFO DATA, even for IN-PIPE, to ensure no stale data remains in FIFO
	// set IN-PIPE mode to manual validation for bulk and interrupt endpoint
	if (ep_type == USBD_EP_CFG_TYPE_BULK) {
		USBD->EP[ep - 1].USBD_EPRSPCTL = (BIT(NCT_USBD_EPRSPCTL_FLUSH) | (0x01 << 1));
	} else if (ep_type == USBD_EP_CFG_TYPE_INT) {
		USBD->EP[ep - 1].USBD_EPRSPCTL = (BIT(NCT_USBD_EPRSPCTL_FLUSH) | (0x01 << 1));
	} else if (ep_type == USBD_EP_CFG_TYPE_ISO) {
		USBD->EP[ep - 1].USBD_EPRSPCTL = (BIT(NCT_USBD_EPRSPCTL_FLUSH) | (0x10 << 1));
	}

	USBD->EP[ep - 1].USBD_EPCFG = (ep_type | ep_dir | ((ep) << 4));
}

static inline void usbd_reset_dma(void)
{
	USBD->USBD_DMACNT = 0ul;
	USBD->USBD_DMACTL = 0x80ul;
	USBD->USBD_DMACTL = 0x00ul;
}

/**
 * Write endpoint data from RAM to USB FIFO.
 * @param ep_idx Endpoint index (1-based for EPA-EPL, 0xFF for CEP)
 * @param data Pointer to data buffer
 * @param len Number of bytes to transfer
 * @return 0 on success, negative error code otherwise
 */
static inline int usbd_write_ep(uint8_t ep_idx, const uint8_t *data, uint32_t len)
{
	if ((ep_idx > NCT_EP_HW_LAST) && (ep_idx != NCT_EP_CEP)) {
		return -EINVAL;
	}

#if (RUN_BULK_IN_MODE == RUN_AUTO_VALIDATE_MODE)
	uint32_t rspctl = BIT(2) | BIT(1);
	uint32_t mps = USBD->EP[ep_idx - 1].USBD_EPMPS;

	USBD->EP[ep_idx - 1].USBD_EPRSPCTL = 0;

#if defined(DBG_USBD_BULK_IN)		
	/* Performance measurement: start timing */
	int64_t start_time = k_uptime_ticks();
#endif	

	#if defined(UDC_NCT_DMA_ENABLE)
    const struct device *dev = nct_udc_device_get();
    struct udc_nct_data *priv = udc_get_private(dev);
    k_spinlock_key_t key = k_spin_lock(&priv->dma_lock);

	/* Wait for any previous DMA to complete before starting new transfer */
	while (USBD->USBD_DMACTL & BIT(NCT_USBD_DMACTL_DMAEN)) {
		/* This should be fast, just checking if previous DMA is done */
	}

	/* Configure DMA to transfer data from RAM to FIFO */
	USBD->USBD_DMAADDR = (uint32_t)data;
	USBD->USBD_DMACNT = len;
	
	/* Trigger DMA transfer (hardware will copy len bytes from data to FIFO) */
	USBD->USBD_DMACTL = BIT(NCT_USBD_DMACTL_DMAEN) | BIT(NCT_USBD_DMACTL_SVINEP) | BIT(NCT_USBD_DMACTL_DMARD) | (ep_idx & 0x0FU);

	/* Wait for DMA to complete (DMAEN will be cleared by hardware automatically) */
	while (USBD->USBD_DMACTL & BIT(NCT_USBD_DMACTL_DMAEN)) {
		/* Busy-wait for DMA completion */
	}

	k_spin_unlock(&priv->dma_lock, key);

#if defined(DBG_USBD_BULK_IN)	
	int64_t end_time = k_uptime_ticks();
	int64_t dma_time_us = k_ticks_to_us_near64(end_time - start_time);
	LOG_INF("DMA transfer: len=%u bytes, time=%lld us (%.3f us/byte)", 
		len, dma_time_us, (double)dma_time_us / (double)len);
#endif		

	#else	
	for (uint32_t i = 0; i < len; i++) {
		M8(&USBD->EP[ep_idx - 1].USBD_EPDAT_BYTE) = data[i];
	}

#if defined(DBG_USBD_BULK_IN)
	int64_t end_time = k_uptime_ticks();
	int64_t pio_time_us = k_ticks_to_us_near64(end_time - start_time);
	LOG_INF("PIO transfer:  len=%u bytes, time=%lld us (%.3f us/byte)", 
		len, pio_time_us, (double)pio_time_us / (double)len);
#endif		
	#endif

	/* In auto mode, mode bits must be set for both short and full-MPS transfers. */
	if ((mps != 0U) && ((len % mps) != 0U)) {
		rspctl |= BIT(NCT_USBD_EPRSPCTL_SHORTTXEN);
	}

	USBD->EP[ep_idx - 1].USBD_EPRSPCTL = rspctl;

#elif (RUN_BULK_IN_MODE == RUN_MANUAL_VALIDATE_MODE)
	/*
	 * In manual mode, avoid EPRSPCTL read-modify-write.
	 * TOGGLE (bit3) may have write side effects on this controller, so
	 * program only the required response bits explicitly per transfer.
	 */
	uint32_t rspctl = BIT(1);

	/* Performance measurement: start timing */
	int64_t start_time = k_uptime_ticks();

	#if defined(UDC_NCT_DMA_ENABLE)
    const struct device *dev = nct_udc_device_get();
    struct udc_nct_data *priv = udc_get_private(dev);
    k_spinlock_key_t key = k_spin_lock(&priv->dma_lock);

	/* Wait for any previous DMA to complete before starting new transfer */
	while (USBD->USBD_DMACTL & BIT(NCT_USBD_DMACTL_DMAEN)) {
		/* This should be fast, just checking if previous DMA is done */
	}

	/* Configure DMA to transfer data from RAM to FIFO */
	USBD->USBD_DMAADDR = (uint32_t)data;
	USBD->USBD_DMACNT = len;
	
	/* Trigger DMA transfer (hardware will copy len bytes from data to FIFO) */
	USBD->USBD_DMACTL = BIT(NCT_USBD_DMACTL_DMAEN) | BIT(NCT_USBD_DMACTL_SVINEP) | BIT(NCT_USBD_DMACTL_DMARD) | (ep_idx & 0x0FU);

	/* Wait for DMA to complete (DMAEN will be cleared by hardware automatically) */
	while (USBD->USBD_DMACTL & BIT(NCT_USBD_DMACTL_DMAEN)) {
		/* Busy-wait for DMA completion */
	}

	k_spin_unlock(&priv->dma_lock, key);

	int64_t end_time = k_uptime_ticks();
	int64_t dma_time_us = k_ticks_to_us_near64(end_time - start_time);
	LOG_INF("DMA transfer: len=%u bytes, time=%lld us (%.3f us/byte)", 
		len, dma_time_us, (double)dma_time_us / (double)len);

	#else	
	for (uint32_t i = 0; i < len; i++) {
		M8(&USBD->EP[ep_idx - 1].USBD_EPDAT_BYTE) = data[i];
	}

	int64_t end_time = k_uptime_ticks();
	int64_t pio_time_us = k_ticks_to_us_near64(end_time - start_time);
	LOG_INF("PIO transfer:  len=%u bytes, time=%lld us (%.3f us/byte)", 
		len, pio_time_us, (double)pio_time_us / (double)len);
	#endif

	if (len % USBD->EP[ep_idx - 1].USBD_EPMPS) {
		rspctl |= BIT(NCT_USBD_EPRSPCTL_SHORTTXEN);
	}

	USBD->EP[ep_idx - 1].USBD_EPRSPCTL = rspctl;	
	USBD->EP[ep_idx - 1].USBD_EPTXCNT = len;

#elif (RUN_BULK_IN_MODE == RUN_FLY_MODE)
	// Only support only when every data payload = MPS
#endif

	return 0;
}

/**
 * Read endpoint data from USB FIFO to RAM.
 * @param ep_idx Endpoint index (1-based for EPA-EPL, 0xFF for CEP)
 * @param buf Pointer to net_buf structure
 * @param len Number of bytes to transfer
 * @return 0 on success, negative error code otherwise
 */
static inline int usbd_read_ep(uint8_t ep_idx, struct net_buf *buf, uint32_t len)
{
	uint32_t i;

	if ((ep_idx > NCT_EP_HW_LAST) && (ep_idx != NCT_EP_CEP)) {
		return -EINVAL;
	}

	if (len == 0 || len > 0xFFFFFU) {
		return -EINVAL;
	}

	if ((buf == NULL) || (buf->data == NULL)) {
		return -EINVAL;
	}

	for (i = 0U; i < len; i++) {
		if (net_buf_tailroom(buf) > 0) {
			net_buf_add_u8(buf, USBD->EP[ep_idx].USBD_EPDAT_BYTE);
		}
	}
LOG_DBG("Read %u bytes from EP%d FIFO", len, ep_idx);



#if 0	
	while (USBD->USBD_DMACTL & BIT(NCT_USBD_DMACTL_DMAEN)) {
		/* wait until DMA is ready (previous transfer is done) */
	}

	/* Configure DMA address and count */
	USBD->USBD_DMAADDR = (uint32_t)data;
	USBD->USBD_DMACNT = len;

	/* Enable DMA for read (host sends to EP):
	 * - EPNUM: endpoint index (bits 0-3)
	 * - DMAEN: enable DMA (bit 5)
	 * - DMARD: 1 for read (EP receives from host)
	 */
	uint32_t dmactl = (ep_idx & 0x0FU) | BIT(NCT_USBD_DMACTL_DMAEN) | BIT(NCT_USBD_DMACTL_DMARD);
	USBD->USBD_DMACTL = dmactl;
#endif
	return 0;
}

static inline void usbd_flush_all_ep(void)
{
	uint8_t i;

	for (i = NCT_EP_HW_FIRST; i <= NCT_EP_HW_LAST; i++) {
		USBD->EP[i].USBD_EPRSPCTL = BIT(NCT_USBD_EPRSPCTL_FLUSH) | BIT(1);
	}
}

static inline void usbd_clear_all_ep_intsts(void)
{
	uint8_t i;

	USBD->USBD_CEPINTSTS = 0x1FFF;

	for (i = NCT_EP_HW_FIRST; i <= NCT_EP_HW_LAST; i++) {
		USBD->EP[i].USBD_EPINTSTS = 0x1FFF;
	}
}

static inline void usbd_setup_control_pipe(void)
{
	USBD->USBD_CEPINTEN = BIT(NCT_USBD_CEPINTEN_SETUPPKIEN) |
						  BIT(NCT_USBD_CEPINTEN_RXPKIEN) |
						  BIT(NCT_USBD_CEPINTEN_TXPKIEN) |
						  BIT(NCT_USBD_CEPINTEN_STSDONEIEN);	
}


static int udc_nct_kick_ep(const struct device *dev, struct udc_ep_config *cfg)
{
	struct net_buf *buf;
	struct udc_nct_data *priv = udc_get_private(dev);
	uint8_t ep = cfg->addr;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);

	if (!cfg->stat.enabled || cfg->stat.halted) {
		return 0;
	}

	if (udc_ep_is_busy(dev, ep)) {
		if (USB_EP_DIR_IS_IN(ep)) {
			sys_slist_t q_list;
			struct net_buf *q_iter;
			uint16_t q_depth = 0U;

			q_list.head = k_fifo_peek_head(&cfg->fifo);
			q_list.tail = k_fifo_peek_tail(&cfg->fifo);
			SYS_SLIST_FOR_EACH_CONTAINER(&q_list, q_iter, node) {
				q_depth++;
			}

			if (q_depth > 1U) {
				LOG_DBG("EP 0x%02x IN: busy skip kick, q_depth=%u", ep,
					(uint32_t)q_depth);
			}
		}
		return 0;
	}

	buf = udc_buf_peek(dev, ep);
	if (buf == NULL) {
		return 0;
	}

	if (ep_idx == 0U) {
		if (USB_EP_DIR_IS_OUT(ep)) {
			if (udc_ctrl_stage_is_status_out(dev)) {
				LOG_DBG("CEP RX arm (status OUT wait) ceps=0x%08x cepe=0x%08x q=%u setup#%u",
					(uint32_t)USBD->USBD_CEPINTSTS, (uint32_t)USBD->USBD_CEPINTEN,
					(uint32_t)priv->ctrl_out_queued, (uint32_t)priv->dbg_setup_seq);

				// For the Control Read transaction, 
				// we must set NAKCLR here to let NCT6694D ack the STATUS OUT sent by host.
				USBD_SET_CEP_STATE(USBD_CEPCTL_NAKCLR);
				udc_ep_set_busy(dev, ep, true);
			} else if (udc_ctrl_stage_is_data_out(dev)) {
				LOG_DBG("CEP RX arm (data OUT wait)");
				udc_ep_set_busy(dev, ep, true);
			}
			return 0;
		}

		LOG_DBG("CEP TX write len=%u", buf->len);

		if (buf->len == 0U) {
			USBD_SET_CEP_STATE(USBD_CEPCTL_ZEROLEN);
			udc_ep_set_busy(dev, ep, true);
			return 0;
		}

		/*
		 * Control endpoint DATA_IN: limit each write to CEP_MAX_PKT_SIZE (64 bytes).
		 * Hardware FIFO cannot hold more than one packet. If data > 64 bytes,
		 * we transmit in multiple packets: write <= 64, wait TXPK, then continue.
		 */
		uint32_t xfer_len = MIN(buf->len, CEP_MAX_PKT_SIZE);
		for (uint32_t i = 0U; i < xfer_len; i++) {
			M8(&USBD->USBD_CEPDAT) = buf->data[i];
		}

		/*
		 * Arm TXPK interrupt before starting transfer to avoid losing a fast
		 * TX completion event during mask/flag update.
		 */
		udc_ep_set_busy(dev, ep, true);
		USBD->USBD_CEPTXCNT = xfer_len;
		return 0;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		LOG_DBG("EP 0x%02x OUT: arm RX, busy=true", ep);
		udc_ep_set_busy(dev, ep, true);
		/* Drop stale OUT receive flags before enabling RX interrupts. */
		USBD->EP[ep_idx - 1].USBD_EPINTSTS = BIT(NCT_USBD_EPINTSTS_RXPKIF) |
						 BIT(NCT_USBD_EPINTSTS_SHORTRXIF);
		USBD->EP[ep_idx - 1].USBD_EPINTEN = BIT(NCT_USBD_EPINTEN_RXPKIEN) |
						 BIT(NCT_USBD_EPINTEN_SHORTRXIEN);
		return 0;
	}

	const uint8_t *tx_data = buf->data;
	uint32_t tx_len = buf->len;
	uint16_t tx_pkts = 1U;

	/*
	 * Aggregate queued IN requests when endpoint is idle, bounded by endpoint
	 * RAM length so one kick can transmit as much queued data as possible.
	 */
	if (priv->ep_data[ep_idx].ram_len > 0U) {
		sys_slist_t list;
		struct net_buf *iter;
		struct net_buf *depth_iter;
		uint32_t cap = priv->ep_data[ep_idx].ram_len;
		uint32_t merged = 0U;
		uint16_t merged_pkts = 0U;
		uint16_t q_depth = 0U;

		if (cap > sizeof(priv->in_batch_buf)) {
			cap = sizeof(priv->in_batch_buf);
		}

		list.head = k_fifo_peek_head(&cfg->fifo);
		list.tail = k_fifo_peek_tail(&cfg->fifo);

		SYS_SLIST_FOR_EACH_CONTAINER(&list, depth_iter, node) {
			q_depth++;
		}

		SYS_SLIST_FOR_EACH_CONTAINER(&list, iter, node) {
			if (iter->len == 0U) {
				if (merged_pkts == 0U) {
					tx_len = 0U;
					tx_pkts = 1U;
				}
				break;
			}

			if ((merged + iter->len) > cap) {
				break;
			}

			memcpy(&priv->in_batch_buf[merged], iter->data, iter->len);
			merged += iter->len;
			merged_pkts++;
		}

		if (merged_pkts > 1U) {
			tx_data = priv->in_batch_buf;
			tx_len = merged;
			tx_pkts = merged_pkts;
			LOG_DBG("EP 0x%02x IN: aggregate %u pkts total=%u cap=%u", ep,
				(uint32_t)tx_pkts, tx_len, cap);
		} else if (q_depth > 1U) {
			LOG_DBG("EP 0x%02x IN: q_depth=%u but aggregate miss (merged=%u cap=%u)",
				ep, (uint32_t)q_depth, merged, cap);
		}
	}

	LOG_DBG("EP 0x%02x IN: write_ep len=%u pkts=%u", ep, tx_len, (uint32_t)tx_pkts);
	int ret = usbd_write_ep(ep_idx, tx_data, tx_len);
	if (ret < 0) {
		LOG_ERR("EP 0x%02x IN: write_ep failed %d", ep, ret);
		priv->ep_data[ep_idx].in_batch_pkts = 0U;
		return ret;
	}

	priv->ep_data[ep_idx].in_batch_pkts = tx_pkts;

	udc_ep_set_busy(dev, ep, true);
	/*
	 * IN completion must be keyed off INTKIF (host IN token/data handshake),
	 * not BUFEMPTYIF (FIFO condition), to avoid premature completion.
	 */
	USBD->EP[ep_idx - 1].USBD_EPINTSTS = BIT(NCT_USBD_EPINTSTS_INTKIF) |
					 BIT(NCT_USBD_EPINTSTS_TXPKIF) |
					 BIT(NCT_USBD_EPINTSTS_SHORTTXIF);
	USBD->EP[ep_idx - 1].USBD_EPINTEN = BIT(NCT_USBD_EPINTEN_INTKIEN);
	LOG_DBG("EP 0x%02x IN: armed INTKIEN, busy=true, EPINTEN=0x%02x", ep,
		USBD->EP[ep_idx - 1].USBD_EPINTEN);

	return 0;
}

static int usbd_ctrl_feed_dout(const struct device *dev, const size_t length)
{
	struct udc_nct_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	if (k_is_in_isr()) {
		LOG_ERR("usbd_ctrl_feed_dout in ISR, len=%u (defer required)", (uint32_t)length);
		return -EWOULDBLOCK;
	}

	if (cfg == NULL) {
		LOG_ERR("control OUT endpoint config missing");
		return -ENODEV;
	}

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_put(&cfg->fifo, buf);
	priv->ctrl_out_queued++;

	return 0;
}

static void udc_nct_ctrl_out_buf_popped(const struct device *dev)
{
	struct udc_nct_data *priv = udc_get_private(dev);

	if (priv->ctrl_out_queued > 0U) {
		priv->ctrl_out_queued--;
	}
}

static void udc_nct_ep_purge_queued(const struct device *dev, uint8_t ep_addr)
{
	struct net_buf *buf;

	buf = udc_buf_get_all(dev, ep_addr);
	if (buf != NULL) {
		udc_ep_set_busy(dev, ep_addr, false);
		(void)udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}
}

static void udc_nct_schedule_ctrl_out_feed(const struct device *dev,
					   const size_t length,
					   const char *reason)
{
	struct udc_nct_data *priv = udc_get_private(dev);

	priv->ctrl_out_feed_len = (uint16_t)length;
	(void)k_work_submit(&priv->ctrl_status_out_work);
	LOG_DBG("CTL[%s] defer control OUT feed len=%u", reason, (uint32_t)length);
}

static void udc_nct_schedule_ctrl_out_refill_force(const struct device *dev,
						    const size_t length,
						    const char *reason)
{
	struct udc_nct_data *priv = udc_get_private(dev);

	priv->ctrl_out_refill_force = 1U;
	udc_nct_schedule_ctrl_out_feed(dev, length, reason);
}

static void udc_nct_ctrl_status_out_work_handler(struct k_work *work)
{
	struct udc_nct_data *priv = CONTAINER_OF(work, struct udc_nct_data, ctrl_status_out_work);
	const struct device *dev = priv->dev;
	struct udc_data *data = dev->data;
	bool force_refill = (priv->ctrl_out_refill_force != 0U);
	int err;
	size_t length = priv->ctrl_out_feed_len;

	if (!udc_is_enabled(dev)) {
		return;
	}

	/*
	 * Drop stale work items that may run after bus reset/stage change.
	 * Only arm control OUT when controller can receive setup/data/status OUT.
	 */
	if (!force_refill &&
	    !udc_ctrl_stage_is_status_out(dev) &&
	    !udc_ctrl_stage_is_data_out(dev) &&
	    data->stage != CTRL_PIPE_STAGE_SETUP) {
		LOG_DBG("skip deferred status OUT feed (stage changed)");
		return;
	}

	/*
	 * Use setup-sized buffer if caller did not specify a length.
	 */
	if (length == 0U) {
		if (udc_ctrl_stage_is_data_out(dev) && data->setup != NULL) {
			length = udc_data_stage_length(data->setup);
		}

		if (length == 0U) {
			length = SETUP_PKT_SIZE;
		}
	}

	err = usbd_ctrl_feed_dout(dev, length);	
	priv->ctrl_out_refill_force = 0U;
	if (err != 0) {
		LOG_ERR("feed control OUT failed: %d (len=%u)", err, (uint32_t)length);
	} else {
		LOG_DBG("feed control OUT done: len=%u queued=%u force=%u",
			(uint32_t)length,
			(uint32_t)priv->ctrl_out_queued,
			(uint32_t)force_refill);
	}
}

static int udc_nct_handle_ctrl_in_done(const struct device *dev,
				       struct net_buf *buf,
				       const struct udc_nct_ctrl_evt *evt)
{
	int err = 0;

	if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
		/* Status stage completed, notify upper layer. */
		err = udc_ctrl_submit_status(dev, buf);
	}

	/*
	 * DATA_IN stage: Check if more data is pending.
	 * Pull the bytes that were just transmitted, then check remaining.
	 * If buf still has data, write next packet and re-arm TXPK (do not transition stage).
	 * Only transition to STATUS_OUT when all data has been sent.
	 */
	if (udc_ctrl_stage_is_data_in(dev)) {
		uint16_t tx_len = evt ? evt->tx_len : 0U;

		net_buf_pull(buf, tx_len);

		if (buf->len > 0U) {
			/*
			 * More data pending: transmit next packet without changing stage.
			 * This continues the DATA_IN stage until all data is sent.
			 *
			 * The ISR popped this buffer from the IN queue via udc_buf_get().
			 * Put it back so the next TXPK ISR can find it and pass it to the
			 * thread for continued multi-packet processing.
			 */
			uint32_t xfer_len = MIN(buf->len, CEP_MAX_PKT_SIZE);

			LOG_DBG("CTL DATA_IN multi-packet: sent=%u remaining=%u next_xfer=%u",
				(uint32_t)tx_len, buf->len, xfer_len);

			for (uint32_t i = 0U; i < xfer_len; i++) {
				M8(&USBD->USBD_CEPDAT) = buf->data[i];
			}

			struct udc_ep_config *in_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);

			udc_buf_put(in_cfg, buf);
			udc_ep_set_busy(dev, USB_CONTROL_EP_IN, true);
			USBD->USBD_CEPTXCNT = xfer_len;
			return 0;
		}

		/* All DATA_IN sent, now advance to STATUS_OUT */
		udc_ctrl_update_stage(dev, buf);
	} else {
		/* Advance to next control stage (for STATUS_IN, NO_DATA). */
		udc_ctrl_update_stage(dev, buf);
	}

	if (udc_ctrl_stage_is_status_out(dev)) {

		/*
		 * DATA IN stage finished, all packets sent. Release IN buffer and
		 * arm control OUT for zero-length status packet from host.
		 */
		net_buf_unref(buf);

		/*
		 * While waiting for STATUS OUT completion, do not accept a new SETUP.
		 * This keeps the current control transfer deterministic and prevents
		 * back-to-back SETUP preemption before status is fully completed.
		 */
		LOG_DBG("CTL[tx-status-out] armed ceps=0x%08x cepe=0x%08x q=%u setup#%u",
			(uint32_t)USBD->USBD_CEPINTSTS,
			(uint32_t)USBD->USBD_CEPINTEN,
			(uint32_t)((struct udc_nct_data *)udc_get_private(dev))->ctrl_out_queued,
			(uint32_t)((struct udc_nct_data *)udc_get_private(dev))->dbg_setup_seq);

		err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
		if (err != 0) {
			LOG_ERR("CTL[tx-status-out] feed failed: %d", err);
		}

		return 0;
	}

	return err;
}

static void udc_nct_ctrl_enqueue_evt(const struct device *dev,
					     const struct udc_nct_ctrl_evt *evt)
{
	struct udc_nct_data *priv = udc_get_private(dev);
	struct udc_nct_ctrl_evt msg = *evt;

	msg.evt_seq = ++priv->dbg_evt_seq;
	if (msg.setup_seq == 0U) {
		msg.setup_seq = priv->dbg_setup_seq;
	}

	/*
	 * Once reset is observed, allow only RESET event to pass.
	 * Drop stale control events from the pre-reset timeline.
	 */
	if (priv->ctrl_reset_pending != 0U && msg.type != CTRL_EVT_RESET) {
		LOG_DBG("drop ctrl evt during reset-pending #%u %s",
			(uint32_t)msg.evt_seq, udc_nct_evt_name(msg.type));

		if (msg.buf != NULL) {
			net_buf_unref(msg.buf);
		}

		return;
	}

	if (k_msgq_put(&priv->ctrl_evt_msgq, &msg, K_NO_WAIT) != 0) {
		LOG_ERR("drop ctrl evt #%u %s (queue full)",
			(uint32_t)msg.evt_seq, udc_nct_evt_name(msg.type));

		if (msg.buf != NULL) {
			net_buf_unref(msg.buf);
		}
	}
}

static void udc_nct_ctrl_process_evt(const struct device *dev,
					     const struct udc_nct_ctrl_evt *evt)
{
	struct net_buf *buf = evt->buf;
	struct udc_nct_data *priv = udc_get_private(dev);
	const struct udc_nct_config *config = dev->config;
	struct udc_data *data = dev->data;
	int err = 0;
	uint16_t wLength;	

	switch (evt->type) {
	case CTRL_EVT_SETUP:
		if (buf == NULL) {
			LOG_ERR("SETUP evt without buffer");
			return;
		}

		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_data_out(dev)) {
			wLength = udc_data_stage_length(buf);
			err = usbd_ctrl_feed_dout(dev, wLength);
			if (err == -ENOMEM) {
				err = udc_submit_ep_event(dev, buf, err);
			}
		} else if (udc_ctrl_stage_is_data_in(dev)) {
			err = udc_ctrl_submit_s_in_status(dev);
		} else {
			err = udc_ctrl_submit_s_status(dev);
		}

		if (err != 0) {
			LOG_ERR("CEP setup submit failed: %d", err);
		}

		udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);
		break;

	case CTRL_EVT_RXPK:
		if (buf == NULL) {
			LOG_ERR("RXPK evt without buffer");
			return;
		}

		LOG_DBG("CTL RXPK evt: stage=%u was_status_out=%u rx_len=%u",
			(uint32_t)data->stage,
			(uint32_t)evt->was_status_out,
			(uint32_t)evt->rx_len);

		if (evt->was_status_out) {
			udc_ctrl_update_stage(dev, buf);
			err = udc_ctrl_submit_status(dev, buf);
			if (err != 0) {
				LOG_ERR("control status OUT submit failed: %d", err);
			}
			/*
			 * STATUS OUT completion reached via RXPK path.
			 * Re-arm SETUP immediately for the next control transaction.
			 */
			err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
			if (err == -ENOMEM) {
				LOG_ERR("CTL[rx-status-out] setup refill failed: %d", err);
			}
		} else {
			err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
			if (err == -ENOMEM) {
				err = udc_submit_ep_event(dev, buf, err);
			}

			udc_ctrl_update_stage(dev, buf);
			if (udc_ctrl_stage_is_status_in(dev)) {
				err = udc_ctrl_submit_s_out_status(dev, buf);
				if (err != 0) {
					LOG_ERR("control OUT status-IN submit failed: %d", err);
				}
			}
		}
		break;

	case CTRL_EVT_TXPK:
		if (buf == NULL) {
			LOG_ERR("TXPK evt without buffer");
			return;
		}

		err = udc_nct_handle_ctrl_in_done(dev, buf, evt);
		if (err != 0) {
			LOG_ERR("ctrl IN done handling failed: %d", err);
		}
		break;

	case CTRL_EVT_STSDONE:
		if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/*
			 * EP0 zero-length status ACK is completed by STSDONE on this controller.
			 * udc_nct_handle_ctrl_in_done() will call udc_ctrl_submit_status()
			 * which synchronously invokes event_cb() with status flag set.
			 */
			struct net_buf *in_buf = udc_buf_get(dev, USB_CONTROL_EP_IN);

			if (in_buf != NULL) {
				udc_ep_set_busy(dev, USB_CONTROL_EP_IN, false);
				err = udc_nct_handle_ctrl_in_done(dev, in_buf, NULL);
				if (err != 0) {
					LOG_ERR("control status IN submit on stsdone failed: %d", err);
				}
			} else {
				LOG_DBG("CTL[stsdone] status-in without IN buffer");
			}
		} else if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * Some controllers signal STATUS OUT completion via STSDONE rather than
			 * RXPK for zero-length status stage. Treat this as a fallback completion
			 * path to avoid getting stuck in STATUS OUT and triggering host reset.
			 * udc_ctrl_submit_status() synchronously invokes event_cb() with status flag set.
			 */
			struct net_buf *st_buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);

			if (st_buf != NULL) {
				udc_nct_ctrl_out_buf_popped(dev);
				udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);
				udc_ctrl_update_stage(dev, st_buf);
				err = udc_ctrl_submit_status(dev, st_buf);
				if (err != 0) {
					LOG_ERR("control status OUT submit on stsdone failed: %d", err);
				}
			} else {
				LOG_DBG("CTL[stsdone] status-out without OUT buffer");
			}
		}

		/* prepare for next SETUP. */
		err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
		if (err == -ENOMEM) {
			LOG_ERR("CTL[stsdone] setup refill failed: %d", err);
		}
		break;

	case CTRL_EVT_RESET:
		/* Drop stale queued control events captured before bus reset. */
		k_msgq_purge(&priv->ctrl_evt_msgq);
		data->stage = CTRL_PIPE_STAGE_SETUP;

		for (int i = 0; i < config->num_bidir_endpoints; i++) {
			config->ep_cfg_out[i].stat.halted = false;
			config->ep_cfg_in[i].stat.halted = false;
			udc_ep_set_busy(dev, config->ep_cfg_out[i].addr, false);
			udc_ep_set_busy(dev, config->ep_cfg_in[i].addr, false);
			priv->ep_data[i].in_batch_pkts = 0U;
		}

		/* Reset software queue state before re-arming control transfer flow. */
		udc_nct_ep_purge_queued(dev, USB_CONTROL_EP_IN);
		priv->ctrl_out_queued = 0U;
		USBD->USBD_FADDR = 0;
		usbd_reset_dma();
		usbd_flush_all_ep();
		usbd_setup_control_pipe();
		/* Drop latched CEP/EP status bits from pre-reset transactions. */
		usbd_clear_all_ep_intsts();
		/* Re-arm control endpoint setup reception after bus reset. */
		USBD->USBD_GINTEN |= BIT(NCT_USBD_GINTEN_CEPIEN);
		/*
		 * Top-up control OUT setup buffers immediately in event thread
		 * to minimize the reset re-arm interrupt window.
		 */
		err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
		if (err != 0) {
			LOG_WRN("bus-reset-refill immediate feed failed: %d, fallback defer", err);
			udc_nct_schedule_ctrl_out_refill_force(dev, SETUP_PKT_SIZE,
					      "bus-reset-refill-fallback");
		}
		LOG_DBG("USB RSTIF: rearm CEP GINTEN=0x%08x CEPINTEN=0x%08x",
			USBD->USBD_GINTEN, USBD->USBD_CEPINTEN);
		if (udc_is_enabled(dev)) {
			udc_submit_event(dev, UDC_EVT_RESET, 0);
		}
		priv->ctrl_reset_pending = 0U;
		break;

	default:
		LOG_WRN("unknown ctrl evt type=%u", evt->type);
		break;
	}
}

static void udc_nct_cep_isr(const struct device *dev)
{
	volatile uint32_t cep_sts = USBD->USBD_CEPINTSTS;
	volatile uint32_t cep_en = USBD->USBD_CEPINTEN;
	volatile uint32_t irq = cep_sts & cep_en;
	struct udc_nct_data *priv = udc_get_private(dev);
	struct udc_data *data = dev->data;
	struct udc_nct_ctrl_evt evt = { 0 };
	struct net_buf *buf;

#if defined(DBG_USBD_IRQ)
	LOG_DBG("CEP IRQ: raw=0x%08x en=0x%08x irq=0x%08x stage=%u", 
		(uint32_t)cep_sts, (uint32_t)cep_en, (uint32_t)irq, (uint32_t)data->stage);
#endif		

	/*
	 * Check STSDONE before SETUP to ensure correct event ordering when both
	 * arrive simultaneously (irq=0x402). STSDONE must be enqueued first so it's
	 * processed before the new SETUP changes the control stage state.
	 */
	if (IS_BIT_SET(irq, NCT_USBD_CEPINTSTS_STSDONEIF)) {
		USBD->USBD_CEPINTSTS = BIT(NCT_USBD_CEPINTSTS_STSDONEIF);

		evt.type = CTRL_EVT_STSDONE;
		if (IS_BIT_SET(irq, NCT_USBD_CEPINTSTS_SETUPPKIF)) {
			evt.setup_seq = priv->dbg_setup_seq > 0 ? priv->dbg_setup_seq - 1 : 0;
		} else {
			evt.setup_seq = priv->dbg_setup_seq;
		}
		udc_nct_ctrl_enqueue_evt(dev, &evt);
	}

	if (IS_BIT_SET(irq, NCT_USBD_CEPINTSTS_SETUPPKIF)) {
		USBD->USBD_CEPINTSTS = BIT(NCT_USBD_CEPINTSTS_SETUPPKIF);

		udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);
		udc_ep_set_busy(dev, USB_CONTROL_EP_IN, false);

		struct udc_ep_config *ep0_out_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
		if (ep0_out_cfg != NULL) {
			ep0_out_cfg->stat.halted = false;
		}

		struct udc_ep_config *ep0_in_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
		if (ep0_in_cfg != NULL) {
			ep0_in_cfg->stat.halted = false;
		}

		// copy setup packet from hardware registers to private buffer
		priv->setup[0] = (uint8_t)(USBD->USBD_SETUP1_0 & 0xfful);
		priv->setup[1] = (uint8_t)((USBD->USBD_SETUP1_0 >> 8) & 0xfful);
		priv->setup[2] = (uint8_t)(USBD->USBD_SETUP3_2 & 0xfful);
		priv->setup[3] = (uint8_t)((USBD->USBD_SETUP3_2 >> 8) & 0xfful);
		priv->setup[4] = (uint8_t)(USBD->USBD_SETUP5_4 & 0xfful);
		priv->setup[5] = (uint8_t)((USBD->USBD_SETUP5_4 >> 8) & 0xfful);
		priv->setup[6] = (uint8_t)(USBD->USBD_SETUP7_6 & 0xfful);
		priv->setup[7] = (uint8_t)((USBD->USBD_SETUP7_6 >> 8) & 0xfful);	

#if defined(DBG_USBD_IRQ)		
		LOG_DBG("SETUP bm=0x%02x bReq=0x%02x wValue=0x%04x wIndex=0x%04x wLen=%u stage=%u",
			priv->setup[0],
			priv->setup[1],
			(uint32_t)(priv->setup[2] | ((uint16_t)priv->setup[3] << 8)),
			(uint32_t)(priv->setup[4] | ((uint16_t)priv->setup[5] << 8)),
			(uint32_t)(priv->setup[6] | ((uint16_t)priv->setup[7] << 8)),
			(uint32_t)data->stage);
#endif			

		priv->dbg_setup_seq++;

		buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
		if (buf == NULL) {
			/* buffer might be allocated in the event handler for the last event*/
			return;
		}

		udc_nct_ctrl_out_buf_popped(dev);

		net_buf_reset(buf);
		net_buf_add_mem(buf, priv->setup, SETUP_PKT_SIZE);
		udc_ep_buf_set_setup(buf);

		evt.type = CTRL_EVT_SETUP;
		evt.setup_seq = priv->dbg_setup_seq;
		evt.buf = buf;
		udc_nct_ctrl_enqueue_evt(dev, &evt);
	}	

	if (IS_BIT_SET(irq, NCT_USBD_CEPINTSTS_RXPKIF)) {
		uint32_t len;
		uint32_t i;
		bool was_status_out;

		USBD->USBD_CEPINTSTS = BIT(NCT_USBD_CEPINTSTS_RXPKIF);

		// data_out or status_out packet received, read bytes from hardware registers to buffer
		len = USBD->USBD_CEPDATCNT & 0xFFFFul;
		buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
		if (buf != NULL) {
			udc_nct_ctrl_out_buf_popped(dev);

			was_status_out = udc_ctrl_stage_is_status_out(dev);
			for (i = 0U; i < len; i++) {
				if (net_buf_tailroom(buf) > 0) {
					net_buf_add_u8(buf, USBD->USBD_CEPDAT_BYTE);
				}
			}

			udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);

			evt.type = CTRL_EVT_RXPK;
			evt.buf = buf;
			evt.setup_seq = priv->dbg_setup_seq;
			evt.was_status_out = was_status_out ? 1U : 0U;
			evt.rx_len = (uint16_t)len;
			udc_nct_ctrl_enqueue_evt(dev, &evt);
		} 
		else {
			/*
			* Expected timing window:
			* RXPK can arrive before the queued SETUP event is processed in thread
			* context and before the next OUT buffer is prepared. In this case the
			* host retries in a later frame after software re-arms the buffer.
			*/
		}
	}

	if (IS_BIT_SET(irq, NCT_USBD_CEPINTSTS_TXPKIF)) {
		USBD->USBD_CEPINTSTS = BIT(NCT_USBD_CEPINTSTS_TXPKIF);

		buf = udc_buf_get(dev, USB_CONTROL_EP_IN);
		if (buf != NULL) {
			udc_ep_set_busy(dev, USB_CONTROL_EP_IN, false);

			evt.type = CTRL_EVT_TXPK;
			evt.setup_seq = priv->dbg_setup_seq;
			evt.buf = buf;
			evt.tx_len = MIN(buf->len, CEP_MAX_PKT_SIZE);
			udc_nct_ctrl_enqueue_evt(dev, &evt);
		} else {
			LOG_ERR("CEP TXPK drop: no IN buf stage=%u", (uint32_t)data->stage);
		}
	}
}

/**
  * @brief  USBD_SetEpBufAddr, Set Endpoint buffer address
  * @param[in]  u32Ep      Endpoint Number
  * @param[in]  u32Base    Buffer Start Address
  * @param[in]  u32Len     Buffer length
  * @retval None.
  */
__STATIC_INLINE void USBD_SetEpBufAddr(uint32_t u32Ep, uint32_t u32Base, uint32_t u32Len)
{
    if (u32Ep == CEP)
    {
        USBD->USBD_CEPBUFSTART = u32Base;
        USBD->USBD_CEPBUFEND   = u32Base + u32Len - 1ul;
    }
    else
    {
        USBD->EP[u32Ep].USBD_EPBUFSTART = u32Base;
        USBD->EP[u32Ep].USBD_EPBUFEND = u32Base + u32Len - 1ul;
    }
}



static void InitForHighSpeed(void)
{
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_MAX_PKT_SIZE);
    // USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
	// USBD->USBD_CEPINTEN = BIT(NCT_USBD_CEPINTEN_SETUPPKIEN);
}

/* static */ void InitForFullSpeed(void)
{
	InitForHighSpeed();
}

static void udc_nct_isr(const struct device *dev)
{
	volatile uint32_t IrqStL, IrqSt;
	struct udc_nct_ctrl_evt evt = { 0 };
	struct udc_nct_data *priv = udc_get_private(dev);

	/* get interrupt status */
	IrqStL = USBD->USBD_GINTSTS & USBD->USBD_GINTEN;
	/* Avoid log flood in ISR hot path; enable only for focused debugging. */
	/* LOG_DBG("isr: GINTSTS=0x%08x GINTEN=0x%08x", USBD->USBD_GINTSTS, USBD->USBD_GINTEN); */
	if (!IrqStL) {
		return;
	}

	bool enabled = udc_is_enabled(dev);	

	/* USB interrupt */
	if (IS_BIT_SET(IrqStL, NCT_USBD_GINTSTS_USBIF)) {
		/* Bus Status */
		IrqSt = USBD->USBD_BUSINTSTS & USBD->USBD_BUSINTEN;
#if defined(DBG_USBD_IRQ)		
		LOG_DBG("isr: BUSINTSTS=0x%08x BUSINTEN=0x%08x", IrqSt, USBD->USBD_BUSINTEN);
#endif

		/* SOF */
		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_SOFIF)) {
			/* Clear Bus interrupt flag */
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_SOFIF);
		}

		/* Reset */
		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_RSTIF)) {
#if defined(DBG_USBD_IRQ)					
			LOG_DBG("reset_isr");
#endif
			priv->ctrl_reset_pending = 1U;
			evt.type = CTRL_EVT_RESET;
			udc_nct_ctrl_enqueue_evt(dev, &evt);

			/* Clear Bus interrupt flag */
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_RSTIF);
			return;
		}

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_RESUMEIF)) {
			if (enabled) {
				udc_submit_event(dev, UDC_EVT_RESUME, 0);
			}

			/* After resume, next meaningful transitions are suspend or reset. */
			USBD->USBD_BUSINTEN = BIT(NCT_USBD_BUSINTEN_RSTIEN) |
					      BIT(NCT_USBD_BUSINTEN_SUSPENDIEN);

            // HSUSBD_CLR_BUS_INT_FLAG(HSUSBD_BUSINTSTS_RESUMEIF_Msk);			
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_RESUMEIF);
		}

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_SUSPENDIF)) {
			if (enabled) {
				udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
			}
			/* While suspended, wait for resume or reset. */
			USBD->USBD_BUSINTEN = BIT(NCT_USBD_BUSINTEN_RSTIEN) |
					      BIT(NCT_USBD_BUSINTEN_RESUMEIEN);
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_SUSPENDIF);
		}

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_HISPDIF)) {
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_HISPDIF);
		}

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_DMADONEIF)) {
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_DMADONEIF);
        }

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_PHYCLKVLDIF)) {
			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_PHYCLKVLDIF);
		}

		if (IS_BIT_SET(IrqSt, NCT_USBD_BUSINTSTS_VBUSDETIF)) {
			/* Handle cable plug/unplug transition and clear latch. */
			if (!IS_BIT_SET(USBD->USBD_PHYCTL, NCT_USBD_PHYCTL_VBUSDET)) {
				USBD->USBD_PHYCTL &= ~BIT(NCT_USBD_PHYCTL_DPPUEN);
			} else {
				// USBD->USBD_PHYCTL |= BIT(NCT_USBD_PHYCTL_PHYEN) | BIT(NCT_USBD_PHYCTL_DPPUEN);
			}

			USBD->USBD_BUSINTSTS = BIT(NCT_USBD_BUSINTSTS_VBUSDETIF);
		}
	}

	if (IS_BIT_SET(IrqStL, NCT_USBD_GINTSTS_CEPIF)) {
		udc_nct_cep_isr(dev);
	}

	for (uint8_t ep_hw = NCT_EP_HW_FIRST; ep_hw <= NCT_EP_HW_LAST; ep_hw++) {
		if (!IS_BIT_SET(IrqStL, (NCT_USBD_GINTSTS_EPAIF + ep_hw))) {
			continue;
		}

		volatile uint32_t epintsts_raw = USBD->EP[ep_hw].USBD_EPINTSTS;
		volatile uint32_t irq_ep = epintsts_raw & USBD->EP[ep_hw].USBD_EPINTEN;
		bool in = IS_BIT_SET(USBD->EP[ep_hw].USBD_EPCFG, NCT_USBD_EPCFG_EPDIR);
		uint8_t ep_addr = (ep_hw + 1U) | (in ? USB_EP_DIR_IN : USB_EP_DIR_OUT);
		struct net_buf *buf;

		/*
		 * Always clear ALL EPINTSTS bits (not just the enabled ones).
		 * Clearing only irq_ep leaves TXPKIF/INTKIF bits set which accumulate
		 * and cause spurious ISR re-entry when EPINTEN is re-armed for the
		 * next transfer.
		 */
		USBD->EP[ep_hw].USBD_EPINTSTS = epintsts_raw;
		if (!irq_ep) {
			continue;
		}

#if defined(DBG_USBD_IRQ)		
		LOG_DBG("EP%d%s IRQ: raw=0x%08x en=0x%08x irq=0x%08x", ep_hw, in ? " IN" : " OUT", epintsts_raw, USBD->EP[ep_hw].USBD_EPINTEN, irq_ep);
#endif

		/*
		 * For bulk IN: INTKIF fires once per ACKed packet, not once per
		 * complete transfer. If FIFO still has data, more packets remain.
		 * Re-arm INTKIEN without popping the buffer so the transfer continues.
		 */
		if (in) {
			uint32_t remaining = USBD->EP[ep_hw].USBD_EPDATCNT & 0xFFFFul;

#if defined(DBG_USBD_IRQ)
			uint32_t ctl = USBD->EP[ep_hw].USBD_EPRSPCTL;
			LOG_DBG("EP%d IN: INTKIF with %u bytes remaining, EPRSPCTL=0x%08x",
				ep_hw, remaining, ctl);
#endif


			if (remaining > 0) {
#if defined(DBG_USBD_IRQ)
				LOG_DBG("EP%d IN: partial done, remaining=%u, re-arm INTKIEN",
					ep_hw, remaining);
#endif
				USBD->EP[ep_hw].USBD_EPINTSTS = BIT(NCT_USBD_EPINTSTS_INTKIF) |
							BIT(NCT_USBD_EPINTSTS_TXPKIF) |
							BIT(NCT_USBD_EPINTSTS_SHORTTXIF);
				USBD->EP[ep_hw].USBD_EPINTEN = BIT(NCT_USBD_EPINTEN_INTKIEN);
				continue;
			}

			/*
			 * INTKIF with empty endpoint FIFO means current HW transfer is done.
			 * Stop INTK IRQ here; next kick will re-enable when new TX is queued.
			 */
			USBD->EP[ep_hw].USBD_EPINTEN &= ~BIT(NCT_USBD_EPINTEN_INTKIEN);

			/*
			 * Batch-complete as many queued IN requests as were aggregated into
			 * the just-finished hardware transfer.
			 */
			uint8_t ep_idx = ep_hw + 1U;
			uint16_t done_pkts = priv->ep_data[ep_idx].in_batch_pkts;
			if (done_pkts == 0U) {
				/*
				 * No software request is associated with this INTKIF anymore
				 * (for example queue was purged/cancelled). Treat as spurious
				 * completion and stop IN-token IRQ storm until next kick.
				 */
				LOG_DBG("EP%d IN completion without queued batch, disable INTKIEN",
					ep_hw);
				USBD->EP[ep_hw].USBD_EPINTEN &= ~BIT(NCT_USBD_EPINTEN_INTKIEN);
				continue;
			}

			udc_ep_set_busy(dev, ep_addr, false);
			for (uint16_t i = 0U; i < done_pkts; i++) {
				buf = udc_buf_get(dev, ep_addr);
				if (buf == NULL) {
					LOG_WRN("EP%d IN completion: queue underrun (%u/%u), disable INTKIEN",
						ep_hw, (uint32_t)i + 1U, (uint32_t)done_pkts);
					USBD->EP[ep_hw].USBD_EPINTEN &= ~BIT(NCT_USBD_EPINTEN_INTKIEN);
					break;
				}

				udc_submit_ep_event(dev, buf, 0);
			}

			priv->ep_data[ep_idx].in_batch_pkts = 0U;
			continue;
		}

		buf = udc_buf_get(dev, ep_addr);
		if (buf == NULL) {
LOG_DBG("EP%d%s IRQ: no buffer queued", ep_hw, in ? " IN" : " OUT");			
			/*
			 * No buffer queued for this endpoint yet.
			 * Keep busy state unchanged; software layer (thread queue pump
			 * or next enqueue) will re-kick when buffer becomes available.
			 * For IN endpoints, disable the interrupt to prevent the host's
			 * IN token from re-firing this ISR every poll cycle while there
			 * is no data (would starve the UDC thread).
			 */
			if (in) {
LOG_DBG("EP%d IN IRQ: no buffer, disable INT to avoid ISR flood", ep_hw);
				USBD->EP[ep_hw].USBD_EPINTEN &= ~BIT(NCT_USBD_EPINTEN_INTKIEN);
			}
			continue;
		}

		if (!in) {
			uint32_t len = USBD->EP[ep_hw].USBD_EPDATCNT & 0xFFFFul;
#if defined(DBG_USBD_IRQ)		
			LOG_DBG("EP%d OUT PKT: len=%u buf_len=%u", ep_hw, len, buf->len);
#endif
			usbd_read_ep(ep_hw, buf, len);
		} else {
			/*
			 * FIFO empty: all packets for this transfer have been ACKed by host.
			 * Disable INTKIEN so host IN polling does not re-fire ISR.
			 */
#if defined(DBG_USBD_IRQ)		
			LOG_DBG("EP%d IN: all sent (FIFO empty), disabling INTKIEN, ep_addr=0x%02x",
				ep_hw, ep_addr);
#endif
			USBD->EP[ep_hw].USBD_EPINTEN &= ~BIT(NCT_USBD_EPINTEN_INTKIEN);
		}

		udc_ep_set_busy(dev, ep_addr, false);
		udc_submit_ep_event(dev, buf, 0);
	}
}



static void udc_nct_process_ep_queue(const struct device *dev,
				     struct udc_ep_config *cfg)
{
	struct net_buf *buf;
	int err;

	if (!cfg->stat.enabled) {
		return;
	}

	/*
	 * Do not pop the buffer here. The transfer completion path (ISR) owns
	 * udc_buf_get() when the HW transaction is done.
	 */
	buf = udc_buf_peek(dev, cfg->addr);
	if (buf == NULL) {
		return;
	}

	err = udc_nct_kick_ep(dev, cfg);
	if (err != 0 && err != -EPERM) {
		LOG_WRN("submit ep 0x%02x event failed: %d", cfg->addr, err);
	}
}

/* Helper thread function for driver operations */
static ALWAYS_INLINE void udc_nct_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct udc_nct_config *config = dev->config;
	struct udc_nct_data *priv = udc_get_private(dev);
	struct udc_nct_ctrl_evt evt;

	LOG_INF("UDC NCT driver %p thread started", dev);
	while (true) {
		if (k_msgq_get(&priv->ctrl_evt_msgq, &evt, K_MSEC(1)) == 0) {
			if (priv->ctrl_reset_pending != 0U && evt.type != CTRL_EVT_RESET) {
				LOG_DBG("drop stale ctrl evt while reset pending #%u %s", 
					(uint32_t)evt.evt_seq, udc_nct_evt_name(evt.type));
				if (evt.buf != NULL) {
					net_buf_unref(evt.buf);
				}
				continue;
			}

			udc_nct_ctrl_process_evt(dev, &evt);

			// work only when there are many control events queued up
			while (k_msgq_get(&priv->ctrl_evt_msgq, &evt, K_NO_WAIT) == 0) {
				if (priv->ctrl_reset_pending != 0U && evt.type != CTRL_EVT_RESET) {
					LOG_DBG("drop stale ctrl evt while reset pending #%u %s",
						(uint32_t)evt.evt_seq, udc_nct_evt_name(evt.type));

					if (evt.buf != NULL) {
						net_buf_unref(evt.buf);
					}
					continue;
				}

				udc_nct_ctrl_process_evt(dev, &evt);
			}
		}

		for (int i = 0; i < config->num_bidir_endpoints; i++) {
			udc_nct_process_ep_queue(dev, &config->ep_cfg_out[i]);
			udc_nct_process_ep_queue(dev, &config->ep_cfg_in[i]);
		}
	}
}

/* UDC EP enqueue */
static int udc_nct_ep_enqueue(const struct device *dev,
				struct udc_ep_config *const cfg,
				struct net_buf *buf)
{
	int err;

	udc_buf_put(cfg, buf);

	if (cfg->stat.halted) {
		LOG_DBG("ep 0x%02x halted", cfg->addr);
		return 0;
	}

	/* Try to submit immediately; queue thread will continue pumping if needed. */
	err = udc_nct_kick_ep(dev, cfg);
	if (err != 0 && err != -EPERM) {
		LOG_WRN("submit ep 0x%02x on enqueue failed: %d", cfg->addr, err);
	}

	return 0;
}

/* UDC EP dequeue */
static int udc_nct_ep_dequeue(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;
	struct udc_nct_data *priv = udc_get_private(dev);
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);

	lock_key = irq_lock();

	if (USB_EP_DIR_IS_IN(cfg->addr) && ep_idx < NUM_OF_EP_MAX) {
		priv->ep_data[ep_idx].in_batch_pkts = 0U;
	}

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

/* UDC EP enable */
static int udc_nct_ep_enable(const struct device *dev,
			       struct udc_ep_config *const cfg)
{
	struct udc_nct_data *priv = udc_get_private(dev);
	uint8_t ep = cfg->addr;
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint32_t ep_type;
	uint32_t ep_dir;
	bool is_in;
	uint32_t ram_base;
	uint32_t ram_end;

	LOG_INF("Enable ep 0x%02x", cfg->addr);

	if (ep_idx >= NUM_OF_EP_MAX) {
		return -EINVAL;
	}

	if (ep_idx == 0U) {
		/* Reserve CEP shared RAM [0..63] for control endpoint. */
		if (priv->ram_offset == 0) {
			usbd_set_ep_buf_addr(NCT_EP_CEP, CEP_BUF_BASE, CEP_MAX_PKT_SIZE);	
			priv->ram_offset = CEP_MAX_PKT_SIZE;

			USBD->USBD_GINTEN |= BIT(NCT_USBD_GINTEN_CEPIEN);			
		}

		return 0;
	}

	switch (cfg->attributes) {
	case USB_EP_TYPE_BULK:
		ep_type = USBD_EP_CFG_TYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		ep_type = USBD_EP_CFG_TYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		ep_type = USBD_EP_CFG_TYPE_ISO;
		break;
	default:
		return -EINVAL;
	}

	/* Determine endpoint direction based on address */
	ep_dir = (ep & USB_EP_DIR_MASK) ? USBD_EP_CFG_DIR_IN : USBD_EP_CFG_DIR_OUT;
	is_in = USB_EP_DIR_IS_IN(ep);


	uint32_t buf_size;	
#if (UDC_NCT_EP_RAM_ALLOC_STRATEGY == UDC_NCT_EP_RAM_ALLOC_AVG_BY_COUNT)
	buf_size = (USB_RAM_SIZE - CEP_MAX_PKT_SIZE) / UDC_NCT_EP_RAM_AVG_NON_CTRL_EP_COUNT;
#else
	if (is_in) {
		buf_size = cfg->mps * UDC_NCT_EP_RAM_MULTIPLIER;
	} else {
		buf_size = cfg->mps;
	}
#endif

	if (buf_size < cfg->mps) {
		LOG_ERR("EP 0x%02x: buf_size %u < mps %u", ep, buf_size, cfg->mps);
		return -EINVAL;
	}
	
	if (priv->ram_offset + buf_size > USB_RAM_SIZE) {
		LOG_ERR("EP 0x%02x: buffer overflow (need %u, have %u bytes)",
			ep, buf_size, USB_RAM_SIZE - priv->ram_offset);
		return -ENOMEM;
	}

	ram_base = priv->ram_offset;
	ram_end = ram_base + buf_size - 1U;

	usbd_set_ep_buf_addr(ep_idx, ram_base, buf_size);
	usbd_set_ep_max_payload(ep_idx, cfg->mps);
	usbd_config_ep(ep_idx, ep_type, ep_dir);
	USBD->EP[ep_idx - 1].USBD_EPCFG |= BIT(NCT_USBD_EPCFG_EPEN);

	priv->ep_data[ep_idx].mps = cfg->mps;
	priv->ep_data[ep_idx].in_batch_pkts = 0U;
	priv->ep_data[ep_idx].ram_base = ram_base;
	priv->ep_data[ep_idx].ram_len = buf_size;
	priv->ep_data[ep_idx].type = ep_type;
	priv->ram_offset += buf_size;

	LOG_INF("EP 0x%02x RAM range: [%u..%u] (%u bytes, mps=%u)",
		ep, ram_base, ram_end, buf_size, cfg->mps);

	USBD->USBD_GINTEN |= (BIT(ep_idx) << 1);
	udc_nct_kick_ep(dev, cfg);

	return 0;
}

/* UDC EP disable */
static int udc_nct_ep_disable(const struct device *dev,
				struct udc_ep_config *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	struct udc_nct_data *priv = udc_get_private(dev);

	LOG_DBG("Disable ep 0x%02x", cfg->addr);
	if (ep_idx >= NUM_OF_EP_MAX) {
		return -EINVAL;
	}

	if (ep_idx == 0U) {
		USBD->USBD_CEPINTSTS = 0x1FFF;
		USBD->USBD_CEPINTEN = 0;
		return 0;
	}

	if (USB_EP_DIR_IS_IN(cfg->addr) && ep_idx < NUM_OF_EP_MAX) {
		priv->ep_data[ep_idx].in_batch_pkts = 0U;
	}

	USBD->EP[ep_idx - 1].USBD_EPINTSTS = 0x1FFF;
	USBD->EP[ep_idx - 1].USBD_EPINTEN = 0;
	USBD->EP[ep_idx - 1].USBD_EPCFG &= ~BIT(NCT_USBD_EPCFG_EPEN);

	return 0;
}

/* UDC EP set halt */
static int udc_nct_ep_set_halt(const struct device *dev,
				 struct udc_ep_config *const cfg)
{
	LOG_DBG("Set halt ep 0x%02x", cfg->addr);

	cfg->stat.halted = true;

	return 0;
}

/* UDC EP clear halt */
static int udc_nct_ep_clear_halt(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);

	cfg->stat.halted = false;

	return 0;
}

/* UDC set address */
static int udc_nct_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_nct_data *priv = udc_get_private(dev);

	LOG_DBG("Set new address %u for %p", addr, dev);
	priv->new_addr = addr;
	USBD->USBD_FADDR = priv->new_addr;
	return 0;
}

/* UDC host wakeup */
static int udc_nct_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);

	return 0;
}

/* UDC device speed */
static enum udc_bus_speed udc_nct_device_speed(const struct device *dev)
{
	struct udc_data *data = dev->data;

	return data->caps.hs ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

/* UDC enable device */
static int udc_nct_enable(const struct device *dev)
{
	const struct udc_nct_config *config = dev->config;
	struct udc_nct_data *priv = udc_get_private(dev);
	int err;
	uint64_t st;

	LOG_DBG("Enable device %p", dev);

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err == -ENOENT) {
		LOG_WRN("No pinctrl default state for %s, continue", dev->name);
	} else if (err != 0) {
		LOG_ERR("pinctrl default apply failed: %d", err);
		return err;
	}

	// prepare control OUT queue and buffers before enabling interrupts
	priv->ctrl_out_queued = 0U;
	err = usbd_ctrl_feed_dout(dev, SETUP_PKT_SIZE);
	if (err != 0) {
		LOG_ERR("Failed to pre-feed control OUT buffer: %d", err);
		return err;
	}

	/* Enable USB PHY */
	USBD->USBD_PHYCTL |= BIT(NCT_USBD_PHYCTL_PHYEN);

	/* Wait PHY clock ready */
	/* Don't access other USBD registers */
	st = k_uptime_get();
	while (1) {
		if (k_uptime_get() - st > NCT_USB_PHY_TIMEOUT) {
			LOG_ERR("Timeout: USB PHY!");
			return -ETIMEDOUT;
		}

		USBD->EP[EPA].USBD_EPMPS = 0x20;
		if (USBD->EP[EPA].USBD_EPMPS == 0x20ul) {
			USBD->EP[EPA].USBD_EPMPS = 0x0ul;
			break;
		}
	}

	// PHY ready, enable pull-up to signal presence to host
	USBD->USBD_PHYCTL |= BIT(NCT_USBD_PHYCTL_DPPUEN);

	usbd_reset_dma();
	usbd_flush_all_ep();

	usbd_setup_control_pipe();
	

	USBD->USBD_FADDR = 0;
	USBD->USBD_OPER = (config->speed_idx == 2) ? BIT(NCT_USBD_OPER_HISPDEN) : 0;

	USBD->USBD_BUSINTEN = BIT(NCT_USBD_BUSINTEN_RESUMEIEN) |
						  BIT(NCT_USBD_BUSINTEN_SUSPENDIEN) |
						  BIT(NCT_USBD_BUSINTEN_RSTIEN) |
						  BIT(NCT_USBD_BUSINTEN_VBUSDETIEN);

	USBD->USBD_GINTEN = BIT(NCT_USBD_GINTEN_USBIEN);

	LOG_INF("enable: GINTEN=0x%08x BUSINTEN=0x%08x OPER=0x%08x PHYCTL=0x%08x",
		USBD->USBD_GINTEN, USBD->USBD_BUSINTEN, USBD->USBD_OPER, USBD->USBD_PHYCTL);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

/* UDC disable device */
static int udc_nct_disable(const struct device *dev)
{
	LOG_DBG("Disable device %p", dev);
	irq_disable(DT_INST_IRQN(0));
	
	USBD->USBD_PHYCTL &= ~BIT(NCT_USBD_PHYCTL_PHYEN);
	USBD->USBD_PHYCTL &= ~BIT(NCT_USBD_PHYCTL_DPPUEN);

	return 0;
}

/* UDC init */
static int udc_nct_init(const struct device *dev)
{
	// Init endpoint for Control Pipe
	if (udc_ep_enable_internal(dev, 
			USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, CEP_MAX_PKT_SIZE, 0)) {
		LOG_ERR("Failed to enable control endpoint OUT");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, 
			USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, CEP_MAX_PKT_SIZE, 0)) {
		LOG_ERR("Failed to enable control endpoint IN");
		return -EIO;
	}

	return 0;
}

/* UDC shutdown */
static int udc_nct_shutdown(const struct device *dev)
{
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint OUT");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint IN");
		return -EIO;
	}

	return 0;
}

/* UDC preinit driver */
static int udc_nct_driver_preinit(const struct device *dev)
{
	const struct udc_nct_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct udc_nct_data *priv = udc_get_private(dev);
	int err;

	priv->dev = dev;
	priv->ram_offset = 0;

	k_msgq_init(&priv->ctrl_evt_msgq, priv->ctrl_evt_msgq_buffer, 
		sizeof(struct udc_nct_ctrl_evt), NCT_CTRL_EVT_Q_LEN);

	k_work_init(&priv->ctrl_status_out_work, udc_nct_ctrl_status_out_work_handler);

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.addr_before_status = false;
	data->caps.mps0 = UDC_MPS0_64;	// control pipe max packet size
	
	// FS: mps = 64 / 32 /16 / 8
	// HS: mps = 512
	uint16_t mps = 64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
		mps = 512;
	}

	// Init OUT endpoint configurations and register with UDC core
	int i;
	for (i = 0; i < config->num_bidir_endpoints; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = CEP_MAX_PKT_SIZE;
		} else if ((i & BIT(0))) {
			continue;
		} else {
			/* NCT driver rule: non-control even endpoint numbers are OUT only. */
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = mps;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register OUT endpoint");
			return err;
		}
	}

	// Init IN endpoint configurations and register with UDC core
	for (i = 0; i < config->num_bidir_endpoints; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else if ((i & BIT(0)) == 0) {
			continue;
		} else {
			/* NCT driver rule: non-control odd endpoint numbers are IN only. */
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = mps;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register IN endpoint");
			return err;
		}
	}

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), udc_nct_isr, DEVICE_DT_INST_GET(0), 0);

	/*
	 * Start bottom-half thread: process control events queued by ISR and
	 * keep non-control endpoint queue pumping.
	 */
	config->make_thread(dev);
	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

/* UDC lock */
static int udc_nct_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

/* UDC unlock */
static int udc_nct_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/* UDC API structure */
static const struct udc_api udc_nct_api = {
	.lock = udc_nct_lock,
	.unlock = udc_nct_unlock,
	.device_speed = udc_nct_device_speed,
	.init = udc_nct_init,
	.enable = udc_nct_enable,
	.disable = udc_nct_disable,
	.shutdown = udc_nct_shutdown,
	.set_address = udc_nct_set_address,
	.host_wakeup = udc_nct_host_wakeup,
	.ep_enable = udc_nct_ep_enable,
	.ep_disable = udc_nct_ep_disable,
	.ep_set_halt = udc_nct_ep_set_halt,
	.ep_clear_halt = udc_nct_ep_clear_halt,
	.ep_enqueue = udc_nct_ep_enqueue,
	.ep_dequeue = udc_nct_ep_dequeue,
};

/* Device definition macro */
#define UDC_NCT_DEVICE_DEFINE(n)					\
	PINCTRL_DT_INST_DEFINE(n);					\
									\
	K_THREAD_STACK_DEFINE(udc_nct_stack_##n,			\
			CONFIG_UDC_NCT_THREAD_STACK_SIZE);	\
									\
	static void udc_nct_thread_##n(void *dev, void *arg1,	\
					 void *arg2)			\
	{								\
		udc_nct_thread_handler(dev);			\
	}								\
									\
	static void udc_nct_make_thread_##n(const struct device *dev) \
	{								\
		struct udc_nct_data *priv = udc_get_private(dev);	\
									\
		k_thread_create(&priv->thread_data,			\
				udc_nct_stack_##n,			\
				K_THREAD_STACK_SIZEOF(			\
					udc_nct_stack_##n),		\
				udc_nct_thread_##n,			\
				(void *)dev, NULL, NULL,		\
				K_PRIO_COOP(8),				\
				K_ESSENTIAL,				\
				K_NO_WAIT);				\
		k_thread_name_set(&priv->thread_data, dev->name);	\
	}								\
									\
	static struct udc_ep_config					\
		ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)]; \
	static struct udc_ep_config					\
		ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];  \
									\
	static const struct udc_nct_config udc_nct_config_##n = { \
		.ep_cfg_out = ep_cfg_out_##n,				\
		.ep_cfg_in = ep_cfg_in_##n,				\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.make_thread = udc_nct_make_thread_##n,			\
		.num_bidir_endpoints = DT_INST_PROP(n, 		\
						num_bidir_endpoints), \
		.speed_idx = DT_INST_ENUM_IDX_OR(n, maximum_speed, 1),	\
	};								\
									\
	static struct udc_nct_data udc_priv_##n = {		\
	};								\
									\
	static struct udc_data udc_data_##n = {				\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),	\
		.priv = &udc_priv_##n,					\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, udc_nct_driver_preinit, NULL,	\
			      &udc_data_##n,				\
			      &udc_nct_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			      &udc_nct_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_NCT_DEVICE_DEFINE)
