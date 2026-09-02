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

/* Get default value in dts */
static const struct flm_nct_config flm_cfg = {
    .base = (struct flm_reg *)DT_INST_REG_ADDR(0),
    .irq = DT_INST_IRQN(0),
};

#define FLM     (flm_cfg.base)
#define IRQN    (flm_cfg.irq)

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
        LOG_ERR("Unlock Failed");
        return -1;
    }

    if (GET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD) != 0xA9) {
        SET_FIELD(FLM->CTL, NCT_FLM_CTL_RLCK_FIELD, 0xA9);
    }

    timeout = 1000;
    while (((FLM->CTL & BIT(NCT_FLM_CTL_RDY)) == 0) && --timeout) {
    }

    if (!(FLM->CTL & BIT(NCT_FLM_CTL_RDY))) {
		LOG_ERR("Unlock Failed");
        return -1;
    }

    return 0;
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
    if (flm_is_lock()) {
        return -1;
    }
    /* clear event */
    FLM->STAT = BIT(NCT_FLM_STAT_RJ_EV) | BIT(NCT_FLM_STAT_SCI_EV) | BIT(NCT_FLM_STAT_TCR_EV);
    /* clear FLM_TCRn, FLM_CMDEV and FLM_CMBEV */
    FLM->TCGC = BIT(NCT_FLM_TCGC_TCCLR);
    FLM->CTL |= BIT(NCT_FLM_CTL_MEN);
    if ((RJ_CallBackfn != NULL) || (TCR_CallBackfn != NULL) || (CSI_CallBackfn != NULL)) {
        irq_enable(IRQN);
    }

    return 0;
}

/* return :
 *      -1 : FLM is locked, can not disable module
 *       0 : done
 */
int flm_module_disable(void)
{
    if (flm_is_lock()) {
        return -1;
    }
    FLM->CTL &= ~BIT(NCT_FLM_CTL_MEN);
    irq_disable(IRQN);
    return 0;
}

/*
 * return:
 *      -1 : FLM is locked
 *       0 : Success
 */
int flm_init(FLM_EngineMode_Enum mode, FLM_FlashSize_Enum flash_size,
        ptrFLM_RJ_Callback rjfn)
{
    struct scfg_reg *inst_scfg =
        (struct scfg_reg *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(scfg), scfg);

    if (flm_unlock() < 0) {
        return -1;
    }

    RJ_CallBackfn = rjfn;

    FLM->CTL &= ~BIT(NCT_FLM_CTL_MEN);

    /* Disables checking FLM_DI1-3 signals during command byte phase of the command. */
    FLM->CFG = ((mode & BIT(0)) << NCT_FLM_CFG_MON_MD) |
               (flash_size << GET_FIELD_POS(NCT_FLM_CFG_DEVSIZ_FIELD)) |
               BIT(NCT_FLM_CFG_EBCHKDIS);

    inst_scfg->DEVALT0[5] |= BIT(7); /* enable FLM pad (DEVALT5) */

    /* Configure DEVALT2 BIT5 ~ BIT7 */
    inst_scfg->DEVALT0[2] &= 0x1F;
    inst_scfg->DEVALT0[2] |= (mode & 0xE0);

    if (mode & BIT(1)) {
        /* abort mode and monitor FLM_CSI# (GPIO54) */
        inst_scfg->DEVALT0[0xC] &= ~BIT(3); /* DEVALTC */
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

    return 0;
}

void flm_del_allow_list(uint8_t idx)
{
    if (idx >= 32) {
        return;
    }
    FLM->CMD[idx] = 0;
    FLM->CMDEN &= ~(0x01 << idx);
}

/* addr_range_idx_mask: bit0~7 means idx 0~7, if range is no need, this value shoule be 0.
 * dummy_bytes: dummy bytes counts after address, it should be 0~3
 * return: -1: not success, >=0: success
 */
int flm_add_range_to_allow_list(
        uint8_t cmd,
        uint8_t addr_range_mask,
        FLM_FlashMode_Enum mode,
        uint8_t dummy_bytes)
{
    int i;

    for (i = 0; i <= 31; i++) {
        if (i == FLM_CMD_IDX_RSV_CQ0 || i == FLM_CMD_IDX_RSV_CQ1 ||
            i == FLM_CMD_IDX_RSV_CQ2 || i == FLM_CMD_IDX_RSV_CQ3) {
            continue;
        }
        if (((FLM->CMDEN & (1 << i)) == 0) && ((FLM->CMD[i] & 0xFF) == 0)) {
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
        FLM->CMD[i] |= (((mode & 0x30) >> 4) << GET_FIELD_POS(NCT_FLM_CMD_ADBPCK_FIELD));
        /* data mode */
        FLM->CMD[i] |= ((mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DUMBPCK_FIELD));
        FLM->CMD[i] |= ((mode & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DATBPCK_FIELD));
        /* 3 or 4 bytes mode */
        FLM->CMD[i] |= ((mode & 0x80) >> 7) << NCT_FLM_CMD_ADDSZ;
        FLM->CMD[i] |= (dummy_bytes & 0x03) << GET_FIELD_POS(NCT_FLM_CMD_DUMB_FIELD);
    }
    FLM->CMD[i] |= cmd << GET_FIELD_POS(NCT_FLM_CMD_CMD_FIELD);
    FLM->CMDEN |= 0x01 << i;

    return i;
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
        if (((FLM->CMDEN & (1 << i)) == 0) && ((FLM->CMD[i] & 0xFF) == 0)) {
            break;
        }
    }
    if (i > FLM_CMD_IDX_RSV_CQ3) {
        return -1;
    }

    idx = i / 8;

    FLM->CMD[i] = cmd << GET_FIELD_POS(NCT_FLM_CMD_CMD_FIELD);
    FLM->CMDEN |= 0x01 << i;

    FLM->CQ[idx] = (((0x01) << i) << GET_FIELD_POS(NCT_FLM_CQ_QEN_FIELD)) |
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
