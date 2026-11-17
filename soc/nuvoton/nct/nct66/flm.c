/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT   nuvoton_nct_flm

#include <errno.h>

#include <zephyr/types.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flm_nct, LOG_LEVEL_DBG);

#include "flm.h"

/* Device configuration */
struct flm_nct_config {
    struct flm_reg *const base;     /* FLM controller base address */
    uint8_t irq;                    /* FLM controller irq */
};

/* Runtime data (set by flm_init) */
struct flm_nct_data {
    FLM_EngineMode_Enum mode;
    FLM_FlashSize_Enum flash_size;
};

static struct flm_nct_data flm_data = {
    .mode = FLM_ABORT_MODE_FLM,
    .flash_size = FLM_FLASH_SIZE_32MB,
};

/* Get default value in dts */
static const struct flm_nct_config flm_cfg = {
    .base = (struct flm_reg *)DT_INST_REG_ADDR(0),
    .irq = DT_INST_IRQN(0),
};

#define FLM             (flm_cfg.base)
#define IRQN            (flm_cfg.irq)
#define FLM_MODE        (flm_data.mode)
#define FLM_FLASH_SIZE  (flm_data.flash_size)

ptrFLM_RJ_Callback RJ_CallBackfn = NULL;
ptrFLM_TCR_Callback TCR_CallBackfn = NULL;
ptrFLM_CSI_Callback CSI_CallBackfn = NULL;

#define FLM_CMD_IDX_RSV_CQ0     7
#define FLM_CMD_IDX_RSV_CQ1     15
#define FLM_CMD_IDX_RSV_CQ2     23
#define FLM_CMD_IDX_RSV_CQ3     31

/* Get RJ_NO */
uint16_t flm_get_rejection_number(void)
{
    return GET_FIELD(FLM->STAT, NCT_FLM_STAT_RJ_NO_FIELD);
}

/* Get RJ_TP1/2
 * idx:
 * 1: first rejected transaction
 * 2: second rejected transaction
 *
 * return:
 * 0h: Command byte not in list
 * 1h: Command address out of range
 * 2h: Reserved
 * 3h: Command data going above the valid address range (in case of burst)
 * 4h: Data lines 1-3 (FLM_DI1-3) are not stable during command byte transmission, when EBCHKDIS bit is cleared
 * 5h: Transaction was disqualified by one of the FLM_CQn registers
 * */
uint8_t flm_get_rejection_type(uint8_t idx)
{
    if (idx == 1) {
        return GET_FIELD(FLM->STAT, NCT_FLM_STAT_RJ_TP1_FIELD);
    } else if (idx == 2) {
        return GET_FIELD(FLM->STAT, NCT_FLM_STAT_RJ_TP2_FIELD);
    }
    return 0;
}

/* Clears RJ_TP1, RJ_TP2, RJ_NO fields and FLM_LOGn */
void flm_clr_rejection_event(void)
{
    FLM->STAT = BIT(NCT_FLM_STAT_RJ_EV);
}

/* idx: 0 ~ 7 */
void flm_set_address_range(uint8_t idx, uint32_t start_addr, uint32_t size)
{
    uint16_t statrang;
    uint16_t lastrang;

    statrang = start_addr / 4096;
    lastrang = (start_addr + (size - 1)) / 4096;

    FLM->RANG[idx] = (lastrang << GET_FIELD_POS(NCT_FLM_RANG_LASTRANG_FIELD)) | statrang;
}

/* return :
 *       1 : FLM is locked
 *       0 : FLM is unlocked
 */
int flm_is_lock(void)
{
    if (FLM->CTL & BIT(NCT_FLM_CTL_LCK)) {
        return 1;
    }
    if (GET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD) != 0xA9) {
        return 1;
    }
    return 0;
}

/* return :
 *      -1 : can not unlock
 *       0 : unlock successfully
 */
int flm_unlock(void)
{
    uint32_t timeout;

    if (FLM->CTL & BIT(NCT_FLM_CTL_LCK)) {
        LOG_ERR("Unlock Failed: non-reversible lock");
        return -1;
    }

    if (GET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD) != 0xA9) {
        SET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD, 0xA9);
    }

    timeout = 1000;
    while (((FLM->CTL & BIT(NCT_FLM_CTL_RDY)) == 0) && --timeout) {
    }

    if (!(FLM->CTL & BIT(NCT_FLM_CTL_RDY))) {
        LOG_ERR("Unlock Failed: RDY timeout");
        return -1;
    }

    return 0;
}

/* return :
 *       1 : FLM module is enabled
 *       0 : FLM module is disabled
 */
int flm_is_enable(void)
{
    return (FLM->CTL & BIT(NCT_FLM_CTL_MEN)) ? 1 : 0;
}

void flm_non_reversible_lock(void)
{
    FLM->CTL |= BIT(NCT_FLM_CTL_LCK);
}

void flm_reversible_lock(void)
{
    SET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD, 0xFF);
}

/* return :
 *      -1 : FLM is locked, can not enable module
 *       0 : done
 */
int flm_module_enable(void)
{
    struct scfg_reg *inst_scfg =
        (struct scfg_reg *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg);

    if (flm_is_lock()) {
        return -1;
    }
    /* clear event */
    FLM->STAT = BIT(NCT_FLM_STAT_RJ_EV) | BIT(NCT_FLM_STAT_SCI_EV) | BIT(NCT_FLM_STAT_TCR_EV);
    /* clear FLM_TCRn, FLM_CMDEV and FLM_CMBEV */
    FLM->TCGC = BIT(NCT_FLM_TCGC_TCCLR);

    /* Disables checking FLM_DI1-3 signals during command byte phase of the command. */
    uint32_t mode = FLM_MODE;

    FLM->CFG = ((mode & BIT(0)) << NCT_FLM_CFG_MON_MD) |
               (FLM_FLASH_SIZE << GET_FIELD_POS(NCT_FLM_CFG_DEVSIZ_FIELD)) |
               BIT(NCT_FLM_CFG_EBCHKDIS);

    inst_scfg->DEVALT0[5] |= BIT(7); /* enable FLM pad (DEVALT5) */

    /* Configure DEVALT2 BIT5 ~ BIT7 */
    inst_scfg->DEVALT0[2] &= 0x1F;
    inst_scfg->DEVALT0[2] |= (mode & 0xE0);

    if (mode & BIT(1)) {
        /* abort mode and monitor FLM_CSI# (GPIO54) */
        inst_scfg->DEVALT0[0xC] &= ~BIT(3); /* DEVALTC */
        inst_scfg->DEVALT0[0xC] &= ~BIT(2); /* Quad mode */
    } else {
        /* monitor SHD_CS# */
        inst_scfg->DEVALT0[0xC] |= BIT(3); /* DEVALTC */
        inst_scfg->DEVALT0[0xC] |= BIT(2); /* Quad mode */
    }

    if (RJ_CallBackfn != NULL) {
        FLM->IE |= BIT(NCT_FLM_IE_RJ_IE);
    }
    if (TCR_CallBackfn != NULL) {
        FLM->IE |= BIT(NCT_FLM_IE_TCR_IE);
    }
    if (CSI_CallBackfn != NULL) {
        FLM->IE |= BIT(NCT_FLM_IE_CSI_IE);
    }

    if ((RJ_CallBackfn != NULL) || (TCR_CallBackfn != NULL) || (CSI_CallBackfn != NULL)) {
        irq_enable(IRQN);
    }

    /* Enable the module last, after CFG / pad / interrupt setup is done */
    FLM->CTL |= BIT(NCT_FLM_CTL_MEN);

    return 0;
}

/* return :
 *      -1 : FLM is locked, can not disable module
 *       0 : done
 */
int flm_module_disable(void)
{
    struct scfg_reg *inst_scfg =
        (struct scfg_reg *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg);

    if (flm_is_lock()) {
        return -1;
    }
    FLM->CTL &= ~BIT(NCT_FLM_CTL_MEN);
    FLM->IE &= ~(BIT(NCT_FLM_IE_RJ_IE) | BIT(NCT_FLM_IE_CSI_IE) | BIT(NCT_FLM_IE_TCR_IE));
    /* Disable FLM function, and these pins will act as GPIOs or FIU_SHD pins */
    inst_scfg->DEVALT0[5] &= ~BIT(7);
    irq_disable(IRQN);
    return 0;
}

/*
 * rjfn: rejection callback. NULL keeps the callback registered before.
 * return:
 *      -1 : FLM is locked
 *       0 : Success
 */
int flm_init(FLM_EngineMode_Enum mode, FLM_FlashSize_Enum flash_size,
        ptrFLM_RJ_Callback rjfn)
{
    if (flm_unlock() < 0) {
        return -1;
    }

    if (rjfn != NULL) {
        RJ_CallBackfn = rjfn;
    }
    FLM_MODE = mode;
    FLM_FLASH_SIZE = flash_size;

    FLM->CTL &= ~BIT(NCT_FLM_CTL_MEN);

    return 0;
}

FLM_FlashSize_Enum flm_get_flash_size(void)
{
    return FLM_FLASH_SIZE;
}

void flm_del_allow_list(uint8_t idx)
{
    if (idx >= 32) {
        return;
    }
    FLM->CMD[idx] = 0;
    FLM->CMDEN &= ~BIT(idx);
}

/* Legacy helper: find a free slot and add one entry.
 * flm_set_allow_list() is preferred (it keeps entries in HW register order).
 * addr_range_idx_mask: bit0~7 means idx 0~7, if range is no need, this value shoule be 0.
 * dummy_bytes: dummy bytes counts after address, it should be 0~3
 * return: -1: not success, >=0: success
 */
int flm_add_range_to_allow_list(
        uint8_t cmd,
        uint8_t addr_range_mask,
        FLM_FlashMode_Enum flash_mode,
        uint8_t dummy_bytes)
{
    int i;

    for (i = 0; i <= 31; i++) {
        if (i == FLM_CMD_IDX_RSV_CQ0 || i == FLM_CMD_IDX_RSV_CQ1 ||
            i == FLM_CMD_IDX_RSV_CQ2 || i == FLM_CMD_IDX_RSV_CQ3) {
            continue;
        }
        if (((FLM->CMDEN & BIT(i)) == 0) && ((FLM->CMD[i] & 0xFF) == 0)) {
            break;
        }
    }
    if (i == 32) {
        return -1;
    }

    FLM->CMD[i] = 0;
    if (addr_range_mask) {
        /* range select */
        FLM->CMD[i] |= (addr_range_mask << GET_FIELD_POS(NCT_FLM_CMD_CARSEL_FIELD)) |
                       BIT(NCT_FLM_CMD_CLAR);
        /* address mode */
        FLM->CMD[i] |= (((flash_mode & 0x30) >> 4) << GET_FIELD_POS(NCT_FLM_CMD_ADBPCK_FIELD));
        /* data mode */
        FLM->CMD[i] |= ((flash_mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DUMBPCK_FIELD));
        FLM->CMD[i] |= ((flash_mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DATBPCK_FIELD));
        /* 3 or 4 bytes mode */
        FLM->CMD[i] |= ((flash_mode & 0x80) >> 7) << NCT_FLM_CMD_ADDSZ;
        FLM->CMD[i] |= (dummy_bytes & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DUMB_FIELD);
    }
    FLM->CMD[i] |= cmd << GET_FIELD_POS(NCT_FLM_CMD_CMD_FIELD);
    FLM->CMDEN |= BIT(i);

    return i;
}

/* buf: 128 bytes, 32 elements of 4 bytes {cmd, range_mask, flash_mode, reserved},
 * element n at offset 4n, n = HW CMD register index.
 * Only entries enabled in FLM->CMDEN are filled; others (and reserved elements) are 0.
 * flash_mode is always decoded from the CMD register, also when range_mask is 0.
 */
void flm_get_allow_list(volatile uint8_t *buf)
{
    for (int i = 0; i < 32; i++) {
        buf[i * 4] = 0;
        buf[i * 4 + 1] = 0;
        buf[i * 4 + 2] = 0;
        buf[i * 4 + 3] = 0;
        if (i == FLM_CMD_IDX_RSV_CQ0 || i == FLM_CMD_IDX_RSV_CQ1 ||
            i == FLM_CMD_IDX_RSV_CQ2 || i == FLM_CMD_IDX_RSV_CQ3) {
            continue;
        }
        if (FLM->CMDEN & BIT(i)) {
            uint32_t v = FLM->CMD[i];

            buf[i * 4] = GET_FIELD(v, NCT_FLM_CMD_CMD_FIELD);
            buf[i * 4 + 1] = GET_FIELD(v, NCT_FLM_CMD_CARSEL_FIELD);
            buf[i * 4 + 2] = (uint8_t)((GET_FIELD(v, NCT_FLM_CMD_ADBPCK_FIELD) << 4) |
                                       GET_FIELD(v, NCT_FLM_CMD_DATBPCK_FIELD) |
                                       (((v >> NCT_FLM_CMD_ADDSZ) & 1) << 7));
        }
    }
}

/* buf: 128 bytes, 32 elements of 4 bytes {cmd, range_mask, flash_mode, reserved},
 * element n at offset 4n, n = HW CMD register index.
 * cmd == 0 clears (disables) the entry, regardless of range_mask.
 * Reserved indexes are skipped (HW never touched) and cleared to 0 in buf.
 * An element with an invalid flash_mode is not written (HW keeps its previous value)
 * and is cleared to 0 in buf.
 * flash_mode is stored for every enabled entry; range_mask == 0 means no address check.
 */
void flm_set_allow_list(volatile uint8_t *buf)
{
    for (int i = 0; i < 32; i++) {
        uint8_t cmd = buf[i * 4];
        uint8_t range_mask = buf[i * 4 + 1];
        uint8_t flash_mode = buf[i * 4 + 2];

        if (i == FLM_CMD_IDX_RSV_CQ0 || i == FLM_CMD_IDX_RSV_CQ1 ||
            i == FLM_CMD_IDX_RSV_CQ2 || i == FLM_CMD_IDX_RSV_CQ3) {
            buf[i * 4] = 0;
            buf[i * 4 + 1] = 0;
            buf[i * 4 + 2] = 0;
            buf[i * 4 + 3] = 0;
            continue;
        }

        if (cmd == 0) {
            /* cmd == 0 disables the entry, range_mask / flash_mode are ignored */
            FLM->CMDEN &= ~BIT(i);
            FLM->CMD[i] = 0;
            buf[i * 4 + 1] = 0;
            buf[i * 4 + 2] = 0;
            buf[i * 4 + 3] = 0;
            continue;
        }

        switch (flash_mode) {
        case FLM_SINGLE_3B_MODE: case FLM_DUAL_3B_MODE: case FLM_QUAD_3B_MODE:
        case FLM_DUAL_3B_IO_MODE: case FLM_QUAD_3B_IO_MODE:
        case FLM_SINGLE_4B_MODE: case FLM_DUAL_4B_MODE: case FLM_QUAD_4B_MODE:
        case FLM_DUAL_4B_IO_MODE: case FLM_QUAD_4B_IO_MODE:
            break;
        default:
            /* invalid element: HW keeps the previous value, clear the buf element */
            buf[i * 4] = 0;
            buf[i * 4 + 1] = 0;
            buf[i * 4 + 2] = 0;
            buf[i * 4 + 3] = 0;
            continue;
        }

        /* Disable the entry first so HW never sees a half-written one */
        FLM->CMDEN &= ~BIT(i);
        FLM->CMD[i] = 0;
        if (range_mask) {
            FLM->CMD[i] |= (range_mask << GET_FIELD_POS(NCT_FLM_CMD_CARSEL_FIELD)) |
                           BIT(NCT_FLM_CMD_CLAR);
        }
        FLM->CMD[i] |= (((flash_mode & 0x30) >> 4) << GET_FIELD_POS(NCT_FLM_CMD_ADBPCK_FIELD));
        FLM->CMD[i] |= ((flash_mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DUMBPCK_FIELD));
        FLM->CMD[i] |= ((flash_mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DATBPCK_FIELD));
        FLM->CMD[i] |= ((flash_mode & 0x80) >> 7) << NCT_FLM_CMD_ADDSZ;
        /* dummy bytes = 0 */
        FLM->CMD[i] |= cmd << GET_FIELD_POS(NCT_FLM_CMD_CMD_FIELD);
        FLM->CMDEN |= BIT(i);
        buf[i * 4 + 3] = 0;
    }
}

/*
 * return: -1: not success, >0: handler, keep this handler to delete this command from allow list
 * byte_cnt: how many byte count after command (1 ~ 7)
 * value: compared value (0 ~ 0xFF)
 * mask: 1: compared with value, 0 means don't care. (0 ~ 0xFF)
 */
int flm_add_qualifier_to_allow_list(
        uint8_t cmd,
        uint8_t byte_cnt,
        uint8_t value,
        uint8_t mask)
{
    int i, idx;

    for (i = FLM_CMD_IDX_RSV_CQ0; i <= FLM_CMD_IDX_RSV_CQ3; i += 8) {
        if (((FLM->CMDEN & BIT(i)) == 0) && ((FLM->CMD[i] & 0xFF) == 0)) {
            break;
        }
    }
    if (i > FLM_CMD_IDX_RSV_CQ3) {
        return -1;
    }

    idx = i / 8;

    FLM->CMD[i] = cmd << GET_FIELD_POS(NCT_FLM_CMD_CMD_FIELD);
    FLM->CMDEN |= BIT(i);

    /* CQ[idx] covers CMD[idx*8 .. idx*8+7]; QEN bit n selects CMD[idx*8+n] */
    FLM->CQ[idx] = (BIT(i % 8) << GET_FIELD_POS(NCT_FLM_CQ_QEN_FIELD)) |
            (byte_cnt << GET_FIELD_POS(NCT_FLM_CQ_QBYTE_FIELD)) |
            (value << GET_FIELD_POS(NCT_FLM_CQ_QVAL_FIELD)) |
            (mask << GET_FIELD_POS(NCT_FLM_CQ_QMASK_FIELD));

    return i;
}

void flm_irq_handler(void)
{
    uint32_t stat;

    stat = FLM->STAT;
//    LOG_DBG("FLM_IRQ (STAT = %x)", stat);
    if (stat & BIT(NCT_FLM_STAT_RJ_EV)) {
//        LOG_DBG("RJ_EV");
        if (RJ_CallBackfn != NULL) {
            RJ_CallBackfn(flm_get_rejection_type(1));
        }
    }
#if 0
    if (stat & BIT(NCT_FLM_STAT_SCI_EV)) {
//        LOG_DBG("CSI_EV");
        if (CSI_CallBackfn != NULL) {
            CSI_CallBackfn();
        }
    }
    if (stat & BIT(NCT_FLM_STAT_TCR_EV)) {
//        LOG_DBG("TCR_EV");
        if (TCR_CallBackfn != NULL) {
            TCR_CallBackfn();
        }
    }
#endif

    FLM->STAT = BIT(NCT_FLM_STAT_RJ_EV) | BIT(NCT_FLM_STAT_SCI_EV) | BIT(NCT_FLM_STAT_TCR_EV);
}

#if 0
void flm_cmb_set_reject_opcode(uint8_t opcode, bool enable)
{
    uint8_t reg_idx = opcode >> 3;       // opcode / 8
    uint8_t bit_pos = opcode & 0x07;     // opcode % 8

    if (enable) {
        FLM->CMB[reg_idx] |= (1 << bit_pos);
    } else {
        FLM->CMB[reg_idx] &= ~(1 << bit_pos);
    }
}

void flm_cmb_enable(bool enable)
{
	if (enable) {
		FLM->CMBEN = 1;
	} else {
		FLM->CMBEN = 0;
	}
	FLM->CTL |= BIT(NCT_FLM_CTL_CHANGE);
	while (!(FLM->CTL & BIT(NCT_FLM_CTL_RDY)));
}
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
static int flm_nct_init(const struct device *dev)
{
    LOG_DBG("Device name: %s", dev->name);

    /* irq */
    IRQ_CONNECT(DT_INST_IRQN(0),
            DT_INST_IRQ(0, priority),
            flm_irq_handler, NULL, 0);

    return 0;
}

#define NCT_FLM_INIT(inst)                          \
                                                    \
    DEVICE_DT_INST_DEFINE(inst,                     \
                  flm_nct_init,                     \
                  NULL,                             \
                  NULL, &flm_cfg,                   \
                  POST_KERNEL,                     \
                  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(NCT_FLM_INIT)
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
