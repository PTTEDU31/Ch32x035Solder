#ifndef PD_SINK_H
#define PD_SINK_H

#include <stdint.h>

typedef enum {
    PD_IDLE,       /* no source attached, or gave up */
    PD_NEGOTIATING,
    PD_READY,      /* PS_RDY received for the current contract */
} pd_state_t;

typedef enum {
    PD_KIND_NONE,
    PD_KIND_FIXED,
    PD_KIND_PPS,
    PD_KIND_AVS,   /* EPR adjustable voltage */
} pd_kind_t;

void pd_init(void);
/* Call from the main loop as often as possible; no iteration of the main
 * loop may block for more than a few ms (PD response deadlines ~25 ms). */
void pd_poll(void);

/* Ask for up to mv for a resistive load of load_mohm (0 = no load limit).
 * Picks the highest voltage not above mv at which the source object can
 * supply V / R (+10 %), fixed over programmable on a tie. PPS/AVS are
 * lowered as needed to fit the load. Above 20 V, enters EPR mode if the
 * source supports it (PD_ALLOW_EPR). Repeated calls are cheap. */
void pd_request(uint16_t mv, uint16_t load_mohm);

pd_state_t pd_state(void);
/* Voltage of the accepted contract in mV, 0 while none (or while the bus is
 * moving between Accept and PS_RDY). */
uint16_t pd_contract_mv(void);
pd_kind_t pd_contract_kind(void);
/* Current the source agreed to supply, mA. */
uint16_t pd_contract_ma(void);
/* millis() of the last received message (for timing flash writes). */
uint32_t pd_last_rx(void);
int pd_epr_active(void);
/* millis() at which PS_RDY arrived for the current contract. */
uint32_t pd_ready_since(void);
/* Source capabilities; in EPR mode, positions 8+ are the EPR objects. */
uint8_t pd_src_pdos(const uint32_t **pdos);
/* Debug events, see pd_event(). */
enum {
    PD_EVT_ATTACH = 1,      /* arg: CC line */
    PD_EVT_DETACH,
    PD_EVT_RX,              /* arg: bit7 ext, bits 6-5 ndo (low 2 bits), bits 4-0 type */
    PD_EVT_GET_CAPS,        /* arg: GoodCRC received */
    PD_EVT_TX_HARD_RESET,   /* arg: count */
    PD_EVT_RX_HARD_RESET,
    PD_EVT_TX_SOFT_RESET,   /* arg: GoodCRC received */
    PD_EVT_TX_REQUEST,      /* arg: object position */
    PD_EVT_EPR_ENTER,       /* arg: source EPR capable bit */
    PD_EVT_EPR_MODE,        /* arg: action << 4 | data (2 ack, 3 ok, 4 failed, 5 exit) */
};
int pd_event(uint8_t *code, uint8_t *arg);
/* Debug: number of Request messages sent. */
uint16_t pd_request_count(void);
/* 1 = PD 2.0, 2 = PD 3.0 */
uint8_t pd_spec_rev(void);

#endif
