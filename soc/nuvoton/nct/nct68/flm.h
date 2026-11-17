/*
 * flm.h
 *
 *  Created on: 2026�???1?? *      Author: WSCHUANG
 *
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 */

#ifndef _FLM_H_
#define _FLM_H_

#include <zephyr/types.h>

typedef enum {
    /* BIT0: 1: monitor mode, 0: abort mode
     * BIT1: 1: monitor FLM_CSI# (GPIO54)
     * BIT4~6: determinate SHD, PVT, BAK
     */
	/* input: FLM_CSI# (GPIO54) from PCH
	 * output: FLM_CSIO# / SHD_CS# (GPIO44) to FLASH */
    FLM_ABORT_MODE_FLM          = 0x22,

    /* SAF environment */
    FLM_ABORT_MODE_FIU_SHD      = 0x20,
    FLM_ABORT_MODE_FIU_PVT      = 0x40,
    FLM_ABORT_MODE_FIU_BAK      = 0x80,

    /* SAF environment */
    FLM_MONITOR_MODE_FIU_SHD    = 0x21,
    FLM_MONITOR_MODE_FIU_PVT    = 0x41,
    FLM_MONITOR_MODE_FIU_BAK    = 0x81,
} FLM_EngineMode_Enum;

typedef enum {
	/*
	 * BIT0~1: DUMBPCK (Dummy Bits Per Clock)/DATBPCK (Data Bits Per Clock)
	 * 00: One data/dummy bit per clock checked
	 * 01: Two data/dummy bits per clock checked (Dual mode)
	 * 10: Four data/dummy bits per clock checked (Quad mode)
	 * 11: Reserved
	 *
	 * BIT4~5: ADBPCK (Address Bits Per Clock)
	 * 00: One address bit per clock checked
	 * 01: Two address bits per clock checked (Dual mode)
	 * 10: Four address bits per clock checked (Quad mode)
	 * 11: Reserved
	 *
	 * BIT7: 1: 4-byte address mode, 0: 3-byte address mode
	 */
    FLM_SINGLE_3B_MODE = 0x00,
    FLM_DUAL_3B_MODE = 0x01,
    FLM_QUAD_3B_MODE = 0x02,
    FLM_DUAL_3B_IO_MODE = 0x11,
    FLM_QUAD_3B_IO_MODE = 0x22,
    FLM_SINGLE_4B_MODE = 0x80,
    FLM_DUAL_4B_MODE = 0x81,
    FLM_QUAD_4B_MODE = 0x82,
    FLM_DUAL_4B_IO_MODE = 0x91,
    FLM_QUAD_4B_IO_MODE = 0xA2
} FLM_FlashMode_Enum;

typedef enum {
    FLM_FLASH_SIZE_2MB,
    FLM_FLASH_SIZE_4MB,
    FLM_FLASH_SIZE_8MB,
    FLM_FLASH_SIZE_16MB,
    FLM_FLASH_SIZE_32MB,
    FLM_FLASH_SIZE_64MB,
    FLM_FLASH_SIZE_128MB,
    FLM_FLASH_SIZE_256MB
} FLM_FlashSize_Enum;

typedef enum {
    FLM_RJ_CMD,
    FLM_RJ_ADDR,
    FLM_RJ_RESERVED,
    FLM_RJ_DATA_GO_OUT_OF_RANGE,
    FLM_RJ_DATA_UNSTABLE,
    FLM_RJ_CUALIFIER
} FLM_Rejection_Type_Enum;

typedef void (*ptrFLM_RJ_Callback) (FLM_Rejection_Type_Enum type);  // Reject event call back function
typedef void (*ptrFLM_TCR_Callback) (void);     // CS# pin active low event call back function
typedef void (*ptrFLM_CSI_Callback) (void);     // Transaction counter event call back function

#ifdef __cplusplus
extern "C"
{
#endif


void flm_non_reversible_lock(void);
void flm_reversible_lock(void);
/*
 * return :
 *      -1 : can not unlock
 *       0 : unlock successfully
 */
int flm_unlock(void);
/*
 * return :
 *       1 : FLM is locked
 *       0 : FLM is unlocked
 */
int flm_is_lock(void);
/*
 * return :
 *       1 : FLM module is enabled
 *       0 : FLM module is disabled
 */
int flm_is_enable(void);
/*
 * return :
 *      -1 : FLM is locked, can not enable module
 *       0 : done
 */
int flm_module_enable(void);
/*
 *  return :
 *      -1 : FLM is locked, can not disable module
 *       0 : done
 */
int flm_module_disable(void);
/*
 * return:
 *      -1 : FLM is locked
 *       0 : Success
 */
int flm_init(FLM_EngineMode_Enum mode, FLM_FlashSize_Enum flash_size, ptrFLM_RJ_Callback rjfn);

/* Get flash size set by flm_init */
FLM_FlashSize_Enum flm_get_flash_size(void);

/* idx: 0 ~ 7, size == 0 makes the range empty */
void flm_set_address_range(uint8_t idx, uint32_t start_addr, uint32_t size);

/*
 * return: -1: not success, >0: handler, keep this handler to delete this command from allow list
 * byte_cnt: how many byte count after command (1 ~ 7)
 * value: compared value (0 ~ 0xFF)
 * mask: 1: compared with value, 0 means don't care. (0 ~ 0xFF)
 */
int flm_add_qualifier_to_allow_list(uint8_t cmd,uint8_t byte_cnt,uint8_t value,uint8_t mask);

/* buf: 128 bytes, 32 elements of 4 bytes {cmd, range_mask, flash_mode, reserved},
 * element n at offset 4n, n = HW CMD register index.
 * get: read from HW. Reserved / disabled elements are 0.
 */
void flm_get_allow_list(volatile uint8_t *buf);
/* set: cmd == 0 clears the HW entry. An element with invalid flash_mode is not written
 * (HW keeps its previous value). Invalid/reserved elements are cleared to 0 in buf.
 */
void flm_set_allow_list(volatile uint8_t *buf);

/*
 * Legacy: add one entry into a free slot.
 * addr_range_idx_mask: bit0~7 means idx 0~7
 * dummy_bytes: dummy bytes counts after address, it should be 0~3
 * return: -1: not success, >=0: success (index)
 */
int flm_add_range_to_allow_list(uint8_t cmd, uint8_t addr_range_mask, FLM_FlashMode_Enum flash_mode, uint8_t dummy_bytes);

void flm_del_allow_list(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* _FLM_H_ */

