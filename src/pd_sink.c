/*
 * USB-PD sink, adapted from WCH's EVT/EXAM/USBPD/USBPD_SNK example.
 *
 * Differences from the example:
 * - no SDK Delay_* (they reprogram SysTick); uses board delay_us/millis.
 * - the board has external 5.1k Rd on CC1/CC2 (R12/R13), so the internal
 *   pull-down stays off unless PD_INTERNAL_RD is set.
 * - PD 3.0 with fixed, PPS and (EPR mode) AVS requests, chosen by voltage
 *   and current via pd_request(). EPR message flow follows Ralim's usb-pd
 *   library (IronOS); the EPR path is untested on hardware.
 * - if the source was already attached before we started (MCU reset), asks
 *   for capabilities and, failing that, sends one hard reset.
 */
#include "pd_sink.h"

#include <ch32x035.h>
#include <string.h>

#include "board.h"
#include "config.h"

void USBPD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

#define SRC_CAP_TIMEOUT_MS 1000
#define GET_CAPS_AFTER_MS 620
#define ACCEPT_TIMEOUT_MS 500
#define DETECT_PERIOD_MS 5
#define VBUS_CHECK_MS 50
#define PPS_REFRESH_MS 5000    /* spec: at least every 10 s */
#define EPR_KEEPALIVE_MS 250   /* spec: tSinkEPRKeepAlive 250-500 ms */
#define MAX_HARD_RESETS 3
#define MAX_PDOS 13            /* 7 SPR + 6 EPR positions */
#ifndef CC_SETTLE_US
#define CC_SETTLE_US 20
#endif

#if PD_INTERNAL_RD
#define CC_SINK_CFG (CC_CMP_66 | CC_PD)
#else
#define CC_SINK_CFG (CC_CMP_66)
#endif

/* Message types not in the SDK header. */
#define MSG_NOT_SUPPORTED 0x10      /* control */
#define MSG_EPR_REQUEST 0x09        /* data */
#define MSG_EPR_MODE 0x0A           /* data */
#define MSG_ALERT 0x06              /* data */
#define EXT_EPR_SRC_CAP 0x11        /* extended */
#define EXT_CONTROL 0x10            /* extended */
#define EXT_CTRL_EPR_KEEPALIVE 3
#define EXT_CTRL_EPR_KEEPALIVE_ACK 4

#define EPR_ACTION_ENTER 1
#define EPR_ACTION_ACK 2
#define EPR_ACTION_SUCCEEDED 3
#define EPR_ACTION_FAILED 4
#define EPR_ACTION_EXIT 5

#define CHUNK_LEN 26

/* PDO / RDO fields (USB PD 3.1) */
#define PDO_TYPE(p) ((p) >> 30)
#define PDO_FIXED 0
#define PDO_AUGMENTED 3
#define APDO_TYPE(p) (((p) >> 28) & 3)
#define APDO_PPS 0
#define APDO_EPR_AVS 1
#define FIXED_MV(p) ((((p) >> 10) & 0x3FF) * 50)
#define FIXED_MA(p) (((p) & 0x3FF) * 10)
#define FIXED_EPR_CAPABLE (1UL << 23)
#define PPS_MAX_MV(p) ((((p) >> 17) & 0xFF) * 100)
#define PPS_MIN_MV(p) ((((p) >> 8) & 0xFF) * 100)
#define PPS_MA(p) (((p) & 0x7F) * 50)
#define AVS_MAX_MV(p) ((((p) >> 17) & 0x1FF) * 100)
#define AVS_MIN_MV(p) ((((p) >> 8) & 0xFF) * 100)
#define AVS_PDP_W(p) ((p) & 0xFF)
#define RDO_POS(i) ((uint32_t)(i) << 28)
#define RDO_NO_USB_SUSPEND (1UL << 24)
#define RDO_EPR_CAPABLE (1UL << 22)

static __attribute__((aligned(4))) uint8_t rx_buf[34];
static __attribute__((aligned(4))) uint8_t tx_buf[34];
static uint8_t ack_buf[2];

static PD_CONTROL ctl;
static uint32_t src_pdo[MAX_PDOS];
static uint8_t src_pdo_count;
static uint8_t spec_rev = 1; /* header encoding: 1 = PD 2.0, 2 = PD 3.0 */

static uint8_t ext_buf[MAX_PDOS * 4];
static uint16_t ext_size;

static uint16_t want_mv = 5000;
static uint16_t want_mohm; /* load resistance, 0 = unconstrained */
static pd_kind_t pending_kind;
static uint16_t pending_mv;
static pd_kind_t contract_kind;
static uint16_t pending_ma, contract_ma;
static uint32_t last_rx;
static uint16_t contract_mv;
static uint32_t ready_since;
static uint32_t state_since;
static uint32_t last_detect;
static uint32_t last_vbus_check;
static uint32_t last_request;
static uint32_t last_epr_msg;
static uint8_t vbus_low_cnt;
static uint8_t vbus_seen; /* V_BUS present at least once since attach */
static volatile uint32_t hard_reset_at; /* V_BUS drops on purpose after a hard reset */
#define HARD_RESET_RECOVER_MS 3000
static uint8_t need_request;
static uint8_t asked_caps;
static uint8_t hard_resets;
static uint8_t sent_hard_reset;
static uint8_t epr_active;
static uint8_t epr_tried;
static uint8_t refreshing; /* re-request of the same level (PPS/AVS keep-alive) */
static uint16_t request_count;

/* Debug event ring, drained by pd_event(). */
static volatile uint16_t evt_ring[32];
static volatile uint8_t evt_head, evt_tail;

static void evt(uint8_t code, uint8_t arg)
{
    uint8_t next = (evt_head + 1) & 31;
    if (next != evt_tail) {
        evt_ring[evt_head] = ((uint16_t)code << 8) | arg;
        evt_head = next;
    }
}

static void rx_mode(void)
{
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONFIG |= IE_RX_ACT | IE_RX_RESET | PD_DMA_EN;
    USBPD->DMA = (uint32_t)rx_buf;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
    NVIC_EnableIRQ(USBPD_IRQn);
}

static void sink_cc_init(void)
{
    ctl.Flag.Bit.PR_Role = 0;
    ctl.Flag.Bit.Auto_Ack_PRRole = 0;
    USBPD->PORT_CC1 = CC_SINK_CFG;
    USBPD->PORT_CC2 = CC_SINK_CFG;
}

static void set_state(CC_STATUS s)
{
    ctl.PD_State = s;
    state_since = millis();
}

/* Forget everything learned from the source (hard reset or detach). */
static void forget_source(void)
{
    ctl.Msg_ID = 0;
    src_pdo_count = 0;
    spec_rev = 1;
    contract_mv = 0;
    contract_kind = PD_KIND_NONE;
    epr_active = 0;
    epr_tried = 0;
    need_request = 0;
    refreshing = 0;
}

static void phy_reset(void)
{
    sink_cc_init();
    ctl.Flag.Bit.Connected = 0;
    ctl.Det_Cnt = 0;
    asked_caps = 0;
    sent_hard_reset = 0;
    forget_source();
    set_state(STA_IDLE);
}

static void send_pack(int wait, uint8_t *buf, uint8_t len, uint8_t sop)
{
    if (USBPD->CONFIG & CC_SEL) {
        USBPD->PORT_CC2 |= CC_LVE;
    } else {
        USBPD->PORT_CC1 |= CC_LVE;
    }
    USBPD->BMC_CLK_CNT = UPD_TMR_TX_48M;
    USBPD->DMA = (uint32_t)buf;
    USBPD->TX_SEL = sop;
    USBPD->BMC_TX_SZ = len;
    USBPD->CONTROL |= PD_TX_EN;
    USBPD->STATUS &= BMC_AUX_INVALID;
    USBPD->CONTROL |= BMC_START;

    if (!wait) {
        return;
    }
    while ((USBPD->STATUS & IF_TX_END) == 0) {
    }
    USBPD->STATUS |= IF_TX_END;
    if (USBPD->CONFIG & CC_SEL) {
        USBPD->PORT_CC2 &= ~CC_LVE;
    } else {
        USBPD->PORT_CC1 &= ~CC_LVE;
    }
    /* Back to receive to catch the GoodCRC. */
    USBPD->CONFIG |= PD_ALL_CLR;
    USBPD->CONFIG &= ~PD_ALL_CLR;
    USBPD->CONTROL &= ~PD_TX_EN;
    USBPD->DMA = (uint32_t)rx_buf;
    USBPD->BMC_CLK_CNT = UPD_TMR_RX_48M;
    USBPD->CONTROL |= BMC_START;
}

void USBPD_IRQHandler(void)
{
    if (USBPD->STATUS & IF_RX_ACT) {
        USBPD->STATUS |= IF_RX_ACT;
        if ((USBPD->STATUS & MASK_PD_STAT) == PD_RX_SOP0 && USBPD->BMC_BYTE_CNT >= 6) {
            /* Answer everything except a GoodCRC with a GoodCRC. */
            if (USBPD->BMC_BYTE_CNT != 6 || (rx_buf[0] & 0x1F) != DEF_TYPE_GOODCRC) {
                uint8_t rev = rx_buf[0] & 0xC0;
                delay_us(30);
                ack_buf[0] = (rev > 0x80 ? 0x80 : rev) | DEF_TYPE_GOODCRC;
                ack_buf[1] = (rx_buf[1] & 0x0E) | ctl.Flag.Bit.Auto_Ack_PRRole;
                USBPD->CONFIG |= IE_TX_END;
                send_pack(0, ack_buf, 2, UPD_SOP0);
            }
        }
    }
    if (USBPD->STATUS & IF_TX_END) {
        /* GoodCRC sent: hand the message to pd_poll(). */
        USBPD->PORT_CC1 &= ~CC_LVE;
        USBPD->PORT_CC2 &= ~CC_LVE;
        NVIC_DisableIRQ(USBPD_IRQn);
        ctl.Flag.Bit.Msg_Recvd = 1;
        USBPD->STATUS |= IF_TX_END;
    }
    if (USBPD->STATUS & IF_RX_RESET) {
        /* Hard reset from the source: the bus returns to 5 V. */
        USBPD->STATUS |= IF_RX_RESET;
        evt(PD_EVT_RX_HARD_RESET, 0);
        hard_reset_at = millis() | 1;
        sink_cc_init();
        forget_source();
        ctl.PD_State = STA_SRC_CONNECT;
        state_since = millis();
    }
}

static void load_header(uint8_t msg_type, int extended)
{
    tx_buf[0] = msg_type | (uint8_t)(spec_rev << 6); /* UFP */
    tx_buf[1] = (ctl.Msg_ID & 0x0E) | (extended ? 0x80 : 0);
}

/* len must be a multiple of 4 (data objects). */
static int send_msg(const void *data, uint8_t len)
{
    int ok = 0;

    tx_buf[1] |= (len >> 2) << 4;
    if (len) {
        memcpy(&tx_buf[2], data, len);
    }
    for (int tries = 0; tries < 3 && !ok; tries++) {
        NVIC_DisableIRQ(USBPD_IRQn);
        send_pack(1, tx_buf, len + 2, UPD_SOP0);
        /* Wait up to ~750 us for GoodCRC. */
        for (int i = 0; i < 250; i++) {
            if (USBPD->STATUS & IF_RX_ACT) {
                USBPD->STATUS |= IF_RX_ACT;
                if (USBPD->BMC_BYTE_CNT == 6 && (rx_buf[0] & 0x1F) == DEF_TYPE_GOODCRC) {
                    ctl.Msg_ID += 2;
                    ok = 1;
                    break;
                }
            }
            delay_us(3);
        }
    }
    rx_mode();
    return ok;
}

static int send_ctrl(uint8_t type)
{
    load_header(type, 0);
    return send_msg(NULL, 0);
}

static void reject_unsupported(void)
{
    send_ctrl(spec_rev >= 2 ? MSG_NOT_SUPPORTED : DEF_TYPE_REJECT);
}

static void store_pdos(const uint8_t *data, uint8_t n)
{
    if (n > MAX_PDOS) {
        n = MAX_PDOS;
    }
    for (uint8_t i = 0; i < n; i++) {
        memcpy(&src_pdo[i], &data[4 * i], 4);
    }
    src_pdo_count = n;
}

typedef struct {
    uint8_t pos; /* 1-based object position, 0 = none */
    pd_kind_t kind;
    uint16_t mv;
    uint16_t ma;
} choice_t;

/* Better = higher voltage (not above the target), then fixed over
 * programmable (no refresh needed). */
static int better(const choice_t *a, const choice_t *b)
{
    if (!b->pos) {
        return 1;
    }
    if (a->mv != b->mv) {
        return a->mv > b->mv;
    }
    return a->kind == PD_KIND_FIXED && b->kind != PD_KIND_FIXED;
}

/* Highest voltage at which a load of mohm draws at most ma, with 10 %
 * margin for heater tolerance (cold heaters have lower resistance). */
static uint32_t load_max_mv(uint16_t ma, uint16_t mohm)
{
    if (!mohm) {
        return 0xFFFF;
    }
    return (uint32_t)ma * mohm / 1100;
}

static choice_t pick(uint16_t mv, uint16_t mohm)
{
    choice_t best = {0};

    if (mv > PD_MAX_MV) {
        mv = PD_MAX_MV;
    }
    for (uint8_t i = 0; i < src_pdo_count; i++) {
        uint32_t p = src_pdo[i];
        choice_t c = {.pos = (uint8_t)(i + 1)};

        if (p == 0) {
            continue; /* unused EPR position */
        }
        if (PDO_TYPE(p) == PDO_FIXED) {
            /* Positions 8+ (EPR fixed) only exist in EPR mode. */
            c.kind = PD_KIND_FIXED;
            c.mv = FIXED_MV(p);
            c.ma = FIXED_MA(p);
            if (c.mv > mv || c.mv > load_max_mv(c.ma, mohm)) {
                continue;
            }
        } else if (PDO_TYPE(p) == PDO_AUGMENTED && APDO_TYPE(p) == APDO_PPS) {
            uint32_t top = mv;
            c.ma = PPS_MA(p);
            if (top > PPS_MAX_MV(p)) {
                top = PPS_MAX_MV(p);
            }
            if (top > load_max_mv(c.ma, mohm)) {
                top = load_max_mv(c.ma, mohm);
            }
            if (!PD_ALLOW_PPS || spec_rev < 2 || top < PPS_MIN_MV(p)) {
                continue;
            }
            c.kind = PD_KIND_PPS;
            c.mv = (uint16_t)(top - top % 20);
        } else if (PDO_TYPE(p) == PDO_AUGMENTED && APDO_TYPE(p) == APDO_EPR_AVS) {
            /* AVS current: PDP / V, at most 5 A. Lower the voltage until the
             * load fits, in 100 mV steps. */
            uint32_t top = mv > AVS_MAX_MV(p) ? AVS_MAX_MV(p) : mv;
            if (!epr_active) {
                continue;
            }
            top -= top % 100;
            while (top >= AVS_MIN_MV(p)) {
                uint32_t ma = (uint32_t)AVS_PDP_W(p) * 1000000UL / top;
                if (ma > 5000) {
                    ma = 5000;
                }
                if (top <= load_max_mv((uint16_t)ma, mohm)) {
                    c.ma = (uint16_t)ma;
                    break;
                }
                top -= 100;
            }
            if (top < AVS_MIN_MV(p)) {
                continue;
            }
            c.kind = PD_KIND_AVS;
            c.mv = (uint16_t)top;
        } else {
            continue;
        }
        if (better(&c, &best)) {
            best = c;
        }
    }
    return best;
}

static void send_request(void)
{
    choice_t c = pick(want_mv, want_mohm);
    uint32_t obj[2];
    uint32_t rdo;

    need_request = 0;
    last_request = millis();
    if (!c.pos) {
        /* Nothing usable: vSafe5V is always position 1. */
        c.pos = 1;
        c.kind = PD_KIND_FIXED;
        c.mv = FIXED_MV(src_pdo[0]);
        c.ma = FIXED_MA(src_pdo[0]);
    }

    rdo = RDO_POS(c.pos) | RDO_NO_USB_SUSPEND;
    if (PD_ALLOW_EPR) {
        rdo |= RDO_EPR_CAPABLE;
    }
    switch (c.kind) {
    case PD_KIND_PPS:
        /* Voltage in 20 mV, operating current in 50 mA units. Ask for the
         * full APDO current so the source never enters current limit. */
        rdo |= ((uint32_t)(c.mv / 20) << 9) | (c.ma / 50);
        break;
    case PD_KIND_AVS:
        /* Voltage in 25 mV units, 100 mV steps. */
        rdo |= ((uint32_t)((c.mv / 25) & ~3U) << 9) | (c.ma / 50);
        break;
    default:
        rdo |= ((uint32_t)(c.ma / 10) << 10) | (c.ma / 10);
        break;
    }
    pending_kind = c.kind;
    pending_mv = c.mv;
    pending_ma = c.ma;
    refreshing = contract_mv && c.kind == contract_kind && c.mv == contract_mv;
    request_count++;
    evt(PD_EVT_TX_REQUEST, c.pos);

    obj[0] = rdo;
    obj[1] = src_pdo[c.pos - 1];
    if (epr_active) {
        load_header(MSG_EPR_REQUEST, 0);
        if (!send_msg(obj, 8)) {
            set_state(STA_TX_SOFTRST);
            return;
        }
    } else {
        load_header(DEF_TYPE_REQUEST, 0);
        if (!send_msg(obj, 4)) {
            set_state(STA_TX_SOFTRST);
            return;
        }
    }
    last_epr_msg = millis();
    set_state(STA_RX_ACCEPT_WAIT);
}

static void send_epr_mode_enter(void)
{
    uint32_t obj = ((uint32_t)EPR_ACTION_ENTER << 24) | ((uint32_t)PD_EPR_WATTS << 16);

    epr_tried = 1;
    evt(PD_EVT_EPR_ENTER, (uint8_t)((src_pdo[0] >> 23) & 1));
    load_header(MSG_EPR_MODE, 0);
    send_msg(&obj, 4);
    last_epr_msg = millis();
}

static void send_epr_keepalive(void)
{
    /* Extended_Control, chunked, data size 2: type + unused byte. */
    uint8_t data[4] = {2, 0x80, EXT_CTRL_EPR_KEEPALIVE, 0};

    load_header(EXT_CONTROL, 1);
    send_msg(data, 4);
    last_epr_msg = millis();
}

static void request_chunk(uint8_t type, uint8_t n)
{
    /* Ext header: chunked, chunk number n, request chunk, size 0. */
    uint16_t eh = (1U << 15) | ((uint16_t)n << 11) | (1U << 10);
    uint8_t data[4] = {(uint8_t)eh, (uint8_t)(eh >> 8), 0, 0};

    load_header(type, 1);
    send_msg(data, 4);
}

/* 1 = source on CC1, 2 = CC2, 0 = nothing attached. */
static uint8_t detect(void)
{
    uint8_t ret = 0;

    USBPD->PORT_CC1 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC1 |= CC_CMP_22;
    delay_us(CC_SETTLE_US);
    if (USBPD->PORT_CC1 & PA_CC_AI) {
        ret = 1;
    }
    USBPD->PORT_CC2 &= ~(CC_CMP_Mask | PA_CC_AI);
    USBPD->PORT_CC2 |= CC_CMP_22;
    delay_us(CC_SETTLE_US);
    if (!ret && (USBPD->PORT_CC2 & PA_CC_AI)) {
        ret = 2;
    }
    sink_cc_init();
    return ret;
}

static void detect_proc(void)
{
    uint8_t cc;

    if (ctl.Flag.Bit.Connected) {
        return; /* detach is detected through V_BUS, see vbus_proc() */
    }
    cc = detect();
    ctl.Det_Cnt = cc ? ctl.Det_Cnt + 1 : 0;
    if (ctl.Det_Cnt >= 5) {
        ctl.Det_Cnt = 0;
        ctl.Flag.Bit.Connected = 1;
        evt(PD_EVT_ATTACH, cc);
        if (cc == 1) {
            USBPD->CONFIG &= ~CC_SEL;
        } else {
            USBPD->CONFIG |= CC_SEL;
        }
        set_state(STA_SRC_CONNECT);
    }
}

/* When powered from elsewhere (debugger), a charger can be unplugged
 * without us losing power: treat V_BUS gone as detach. */
static void vbus_proc(void)
{
    uint32_t mv;

    if (!ctl.Flag.Bit.Connected) {
        vbus_low_cnt = 0;
        vbus_seen = 0;
        return;
    }
    mv = vin_mv();
    if (mv >= VIN_MIN_MV) {
        vbus_seen = 1;
    }
    /* After a hard reset the source turns V_BUS off and back on to 5 V
     * (tSrcRecover + tSrcTurnOn, under 1.5 s): not a detach. */
    if (hard_reset_at && millis() - hard_reset_at < HARD_RESET_RECOVER_MS) {
        vbus_low_cnt = 0;
        return;
    }
    /* Rp without V_BUS (source recovering, or latched off after a hard
     * reset) is not a detach: keep waiting. */
    if (vbus_seen && mv < VIN_MIN_MV) {
        if (++vbus_low_cnt >= 4) {
            vbus_low_cnt = 0;
            evt(PD_EVT_DETACH, 0);
            phy_reset();
        }
    } else {
        vbus_low_cnt = 0;
    }
}

static void handle_extended(uint8_t type)
{
    uint16_t eh = rx_buf[2] | ((uint16_t)rx_buf[3] << 8);
    uint16_t size = eh & 0x1FF;
    uint8_t chunk = (eh >> 11) & 0x0F;
    uint16_t off, len;

    if (type == EXT_CONTROL) {
        if (rx_buf[4] == EXT_CTRL_EPR_KEEPALIVE_ACK) {
            last_epr_msg = millis();
        }
        return;
    }
    if (type != EXT_EPR_SRC_CAP || !(eh & (1U << 15)) || (eh & (1U << 10))) {
        reject_unsupported();
        return;
    }
    if (chunk == 0) {
        ext_size = size > sizeof(ext_buf) ? sizeof(ext_buf) : size;
    }
    off = (uint16_t)chunk * CHUNK_LEN;
    if (off >= ext_size) {
        return;
    }
    len = ext_size - off;
    if (len > CHUNK_LEN) {
        len = CHUNK_LEN;
    }
    memcpy(&ext_buf[off], &rx_buf[4], len);
    if (off + len < ext_size) {
        request_chunk(EXT_EPR_SRC_CAP, chunk + 1);
        return;
    }
    store_pdos(ext_buf, (uint8_t)(ext_size / 4));
    send_request();
}

static void handle_msg(void)
{
    uint8_t type = rx_buf[0] & 0x1F;
    uint8_t ndo = (rx_buf[1] >> 4) & 0x07;

    last_rx = millis();
    evt(PD_EVT_RX, (uint8_t)((rx_buf[1] & 0x80 ? 0x80 : 0) | (ndo << 5) | type));
    if (rx_buf[1] & 0x80) {
        handle_extended(type);
        return;
    }

    if (ndo) {
        switch (type) {
        case DEF_TYPE_SRC_CAP: {
            uint8_t rev = (rx_buf[0] >> 6) & 3;
            uint32_t pdo1;
            spec_rev = rev >= 2 ? 2 : 1;
            store_pdos(&rx_buf[2], ndo);
            memcpy(&pdo1, &rx_buf[2], 4);
            /* An SPR caps message means we are not (or no longer) in EPR. */
            epr_active = 0;
            (void)pdo1;
            send_request();
            break;
        }
        case MSG_EPR_MODE: {
            uint32_t obj;
            memcpy(&obj, &rx_buf[2], 4);
            evt(PD_EVT_EPR_MODE, (uint8_t)(((obj >> 24) << 4) | ((obj >> 16) & 0x0F)));
            switch (obj >> 24) {
            case EPR_ACTION_SUCCEEDED:
                epr_active = 1; /* EPR_Source_Capabilities follows */
                last_epr_msg = millis();
                set_state(STA_SRC_CONNECT);
                break;
            case EPR_ACTION_FAILED:
                epr_active = 0;
                break;
            case EPR_ACTION_EXIT:
                epr_active = 0; /* SPR Source_Capabilities follows */
                set_state(STA_SRC_CONNECT);
                break;
            default:
                break;
            }
            break;
        }
        case MSG_ALERT:
            break;
        case DEF_TYPE_VENDOR_DEFINED:
            if (spec_rev >= 2) {
                reject_unsupported();
            }
            break;
        default:
            reject_unsupported();
            break;
        }
        return;
    }

    switch (type) {
    case DEF_TYPE_ACCEPT:
        if (ctl.PD_State == STA_RX_ACCEPT_WAIT) {
            /* Bus is moving to a new level: no valid contract until PS_RDY.
             * A refresh of the same level keeps the contract. */
            if (!refreshing) {
                contract_mv = 0;
            }
            set_state(STA_RX_PS_RDY_WAIT);
        }
        break;
    case DEF_TYPE_REJECT:
    case DEF_TYPE_WAIT:
        /* Previous contract (if any) stays in place. */
        if (ctl.PD_State == STA_RX_ACCEPT_WAIT) {
            refreshing = 0;
            set_state(contract_mv ? STA_RX_PS_RDY : STA_IDLE);
        }
        break;
    case DEF_TYPE_PS_RDY:
        if (ctl.PD_State == STA_RX_PS_RDY_WAIT) {
            if (!refreshing) {
                ready_since = millis();
            }
            contract_mv = pending_mv;
            contract_ma = pending_ma;
            contract_kind = pending_kind;
            refreshing = 0;
            set_state(STA_RX_PS_RDY);
        }
        break;
    case DEF_TYPE_SOFT_RESET:
        ctl.Msg_ID = 0;
        send_ctrl(DEF_TYPE_ACCEPT);
        set_state(STA_SRC_CONNECT);
        break;
    case DEF_TYPE_GET_SNK_CAP: {
        /* Fixed 5 V / 1 A, higher capability (we want more than 5 V). */
        uint32_t pdo = (1UL << 28) | (100UL << 10) | 100;
        load_header(DEF_TYPE_SNK_CAP, 0);
        send_msg(&pdo, 4);
        break;
    }
    case DEF_TYPE_PING:
    case DEF_TYPE_GOODCRC:
    case MSG_NOT_SUPPORTED:
        break;
    default:
        reject_unsupported();
        break;
    }
}

void pd_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBPD, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &gpio);

    AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    USBPD->CONFIG = PD_DMA_EN;
    USBPD->STATUS = BUF_ERR | IF_RX_BIT | IF_RX_BYTE | IF_RX_ACT | IF_RX_RESET | IF_TX_END;

    memset(&ctl, 0, sizeof(ctl));
    phy_reset();
    NVIC_SetPriority(USBPD_IRQn, 0x00); /* may preempt SysTick, nothing else */
    rx_mode();
}

void pd_poll(void)
{
    uint32_t now = millis();
    uint32_t age = now - state_since;

    if (now - last_detect >= DETECT_PERIOD_MS) {
        last_detect = now;
        detect_proc();
    }
    if (now - last_vbus_check >= VBUS_CHECK_MS) {
        last_vbus_check = now;
        vbus_proc();
    }

    switch (ctl.PD_State) {
    case STA_SRC_CONNECT:
        if (epr_active) {
            /* Waiting for EPR_Source_Capabilities after entering EPR. */
            if (age > SRC_CAP_TIMEOUT_MS) {
                set_state(STA_TX_SOFTRST);
            }
            break;
        }
        /* Source may have stopped sending caps before we were running
         * (e.g. MCU reset while attached). Ask once, then hard reset. */
        if (!asked_caps && age > GET_CAPS_AFTER_MS) {
            asked_caps = 1;
            evt(PD_EVT_GET_CAPS, (uint8_t)send_ctrl(DEF_TYPE_GET_SRC_CAP));
        }
        /* Only with V_BUS present: a source that already turned V_BUS off
         * gains nothing from it, and some wait for V_BUS < 0.8 V before
         * turning back on, which never happens while the 3.3 V rail
         * back-feeds V_IN through the buck (debugger-powered bench). */
        if (asked_caps && !sent_hard_reset && hard_resets < MAX_HARD_RESETS &&
            age > SRC_CAP_TIMEOUT_MS - 50 && vin_mv() >= VIN_MIN_MV) {
            sent_hard_reset = 1;
            hard_resets++;
            evt(PD_EVT_TX_HARD_RESET, hard_resets);
            hard_reset_at = millis() | 1;
            forget_source();
            send_pack(1, NULL, 0, UPD_HARD_RESET);
            rx_mode();
            set_state(STA_SRC_CONNECT);
            break;
        }
        if (age > 5 * SRC_CAP_TIMEOUT_MS) {
            /* Not a PD source: stay on 5 V. */
            set_state(STA_IDLE);
        }
        break;
    case STA_RX_ACCEPT_WAIT:
    case STA_RX_PS_RDY_WAIT:
        if (age > ACCEPT_TIMEOUT_MS) {
            set_state(STA_TX_SOFTRST);
        }
        break;
    case STA_TX_SOFTRST:
        /* No hard reset here: it drops V_BUS and would brown us out. */
        ctl.Msg_ID = 0;
        contract_mv = 0;
        epr_active = 0;
        {
            int ok = send_ctrl(DEF_TYPE_SOFT_RESET);
            evt(PD_EVT_TX_SOFT_RESET, (uint8_t)ok);
            set_state(ok ? STA_SRC_CONNECT : STA_IDLE);
        }
        break;
    case STA_RX_PS_RDY:
        if (contract_kind == PD_KIND_PPS || contract_kind == PD_KIND_AVS) {
            if (now - last_request >= PPS_REFRESH_MS) {
                need_request = 1;
            }
        }
        /* PD_EPR_FORCE_TRY: try even without the source's EPR capable bit,
         * to learn the reason from EPR_Mode (Enter Failed). */
        if (PD_ALLOW_EPR && !epr_active && !epr_tried && spec_rev >= 2 && want_mv > 20000 &&
            src_pdo_count && ((src_pdo[0] & FIXED_EPR_CAPABLE) || PD_EPR_FORCE_TRY)) {
            send_epr_mode_enter();
            break;
        }
        if (epr_active && now - last_epr_msg >= EPR_KEEPALIVE_MS) {
            send_epr_keepalive();
            break;
        }
        /* fall through */
    case STA_IDLE:
        if (need_request && src_pdo_count) {
            send_request();
        }
        break;
    default:
        break;
    }

    if (ctl.Flag.Bit.Msg_Recvd) {
        handle_msg();
        ctl.Flag.Bit.Msg_Recvd = 0;
        rx_mode();
    }
}

void pd_request(uint16_t mv, uint16_t load_mohm)
{
    choice_t c;

    if (mv == want_mv && load_mohm == want_mohm) {
        return;
    }
    want_mv = mv;
    want_mohm = load_mohm;
    if (!src_pdo_count) {
        return;
    }
    /* EPR entry for mv > 20 V is handled in pd_poll(). */
    c = pick(mv, load_mohm);
    if (!c.pos || c.mv != contract_mv || c.kind != contract_kind) {
        need_request = 1;
    }
}

pd_state_t pd_state(void)
{
    switch (ctl.PD_State) {
    case STA_RX_PS_RDY:
        return PD_READY;
    case STA_IDLE:
        return contract_mv ? PD_READY : PD_IDLE;
    case STA_RX_ACCEPT_WAIT:
    case STA_RX_PS_RDY_WAIT:
        return refreshing ? PD_READY : PD_NEGOTIATING;
    default:
        return PD_NEGOTIATING;
    }
}

uint16_t pd_contract_mv(void)
{
    return contract_mv;
}

uint16_t pd_contract_ma(void)
{
    return contract_ma;
}

uint32_t pd_last_rx(void)
{
    return last_rx;
}

pd_kind_t pd_contract_kind(void)
{
    return contract_kind;
}

int pd_epr_active(void)
{
    return epr_active;
}

uint32_t pd_ready_since(void)
{
    return ready_since;
}

uint8_t pd_src_pdos(const uint32_t **pdos)
{
    *pdos = src_pdo;
    return src_pdo_count;
}

int pd_event(uint8_t *code, uint8_t *arg)
{
    uint16_t e;

    if (evt_tail == evt_head) {
        return 0;
    }
    e = evt_ring[evt_tail];
    evt_tail = (evt_tail + 1) & 31;
    *code = e >> 8;
    *arg = e & 0xFF;
    return 1;
}

uint16_t pd_request_count(void)
{
    return request_count;
}

uint8_t pd_spec_rev(void)
{
    return spec_rev;
}
