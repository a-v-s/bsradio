/*
 * sxv1.h
 *
 *  Created on: 24 mrt. 2023
 *      Author: andre
 */

#ifndef COMMON_SXV1_H_
#define COMMON_SXV1_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#pragma pack(push, 1)

#define SXV1_READ 0x00
#define SXV1_WRITE 0x80

#define SXV1_REG_FIFO 0x00
#define SXV1_REG_OPMODE 0x01
#define SXV1_REG_DATAMODUL 0x02
#define SXV1_REG_BITRATEMSB 0x03
#define SXV1_REG_BITRATELSB 0x04
#define SXV1_REG_FDEVMSB 0x05
#define SXV1_REG_FDEVLSB 0x06
#define SXV1_REG_FRFMSB 0x07
#define SXV1_REG_FRFMID 0x08
#define SXV1_REG_FRFLSB 0x09
#define SXV1_REG_OSC1 0x0A
#define SXV1_REG_AFCCTRL 0x0B
#define SXV1_REG_LOWBAT 0x0C
#define SXV1_REG_LISTEN1 0x0D
#define SXV1_REG_LISTEN2 0x0E
#define SXV1_REG_LISTEN3 0x0F
#define SXV1_REG_VERSION 0x10
#define SXV1_REG_PALEVEL 0x11
#define SXV1_REG_PARAMP 0x12
#define SXV1_REG_OCP 0x13
#define SXV1_REG_LNA 0x18
#define SXV1_REG_RXBW 0x19
#define SXV1_REG_AFCBW 0x1A
#define SXV1_REG_OOKPEAK 0x1B
#define SXV1_REG_OOKAVG 0x1C
#define SXV1_REG_OOKFIX 0x1D
#define SXV1_REG_AFCFEI 0x1E
#define SXV1_REG_AFCMSB 0x1F
#define SXV1_REG_AFCLSB 0x20
#define SXV1_REG_FEIMSB 0x21
#define SXV1_REG_FEILSB 0x22
#define SXV1_REG_RSSICONFIG 0x23
#define SXV1_REG_RSSIVALUE 0x24
#define SXV1_REG_DIOMAPPING1 0x25
#define SXV1_REG_DIOMAPPING2 0x26
#define SXV1_REG_IRQFLAGS1 0x27
#define SXV1_REG_IRQFLAGS2 0x28
#define SXV1_REG_RSSITHRESH 0x29
#define SXV1_REG_RXTIMEOUT1 0x2A
#define SXV1_REG_RXTIMEOUT2 0x2B
#define SXV1_REG_PREAMBLEMSB 0x2C
#define SXV1_REG_PREAMBLELSB 0x2D
#define SXV1_REG_SYNCCONFIG 0x2E
#define SXV1_REG_SYNCVALUE1 0x2F
#define SXV1_REG_SYNCVALUE2 0x30
#define SXV1_REG_SYNCVALUE3 0x31
#define SXV1_REG_SYNCVALUE4 0x32
#define SXV1_REG_SYNCVALUE5 0x33
#define SXV1_REG_SYNCVALUE6 0x34
#define SXV1_REG_SYNCVALUE7 0x35
#define SXV1_REG_SYNCVALUE8 0x36
#define SXV1_REG_PACKETCONFIG1 0x37
#define SXV1_REG_PAYLOADLENGTH 0x38
#define SXV1_REG_NODEADRS 0x39
#define SXV1_REG_BROADCASTADRS 0x3A
#define SXV1_REG_AUTOMODES 0x3B
#define SXV1_REG_FIFOTHRESH 0x3C
#define SXV1_REG_PACKETCONFIG2 0x3D
#define SXV1_REG_AESKEY1 0x3E
#define SXV1_REG_AESKEY2 0x3F
#define SXV1_REG_AESKEY3 0x40
#define SXV1_REG_AESKEY4 0x41
#define SXV1_REG_AESKEY5 0x42
#define SXV1_REG_AESKEY6 0x43
#define SXV1_REG_AESKEY7 0x44
#define SXV1_REG_AESKEY8 0x45
#define SXV1_REG_AESKEY9 0x46
#define SXV1_REG_AESKEY10 0x47
#define SXV1_REG_AESKEY11 0x48
#define SXV1_REG_AESKEY12 0x49
#define SXV1_REG_AESKEY13 0x4A
#define SXV1_REG_AESKEY14 0x4B
#define SXV1_REG_AESKEY15 0x4C
#define SXV1_REG_AESKEY16 0x4D
#define SXV1_REG_TEMP1 0x4E
#define SXV1_REG_TEMP2 0x4F
#define SXV1_REG_TESTLNA 0x58
#define SXV1_REG_TESTPA1 0x5A
#define SXV1_REG_TESTPA2 0x5C
#define SXV1_REG_TESTDAGC 0x6F
#define SXV1_REG_TESTAFC 0x71

#define SXV1_XTAL_FREQ 32000000
// 32 Mhz / 2^19
//#define SXV1_FSTEP				(61)
#define SXV1_FSTEP_HZ (61.03515625f)
#define SXV1_FSTEP_KHZ (0.06103515625f)
#define SXV1_TXRX_TIMEOUT_MS (100)

// Switching from TX to RX may take up to 2200 µS
// With some margin, wait 3000 µS
#define SXV1_MODESWITCH_TIMEOUT_US (3000)

typedef union {
    struct {
        unsigned int : 2;
        unsigned int mode : 3;
        unsigned int listen_abort : 1;
        unsigned int listen_on : 1;
        unsigned int sequencer_off : 1;
    };
    uint8_t as_uint8;
} sxv1_val_opmode_t;

typedef union {
    struct {
        unsigned int output_power : 5;
        unsigned int pa2_on : 1;
        unsigned int pa1_on : 1;
        unsigned int pa0_on : 1;
    };
    uint8_t as_uint8;
} sxv1_val_palevel_t;

typedef enum {
    sxv1_mode_sleep = 0b000,
    sxv1_mode_standby = 0b001,
    sxv1_mode_fs = 0b010,
    sxv1_mode_tx = 0b011,
    sxv1_mode_rx = 0b100,
} sxv1_mode_t;

typedef union {
    struct {
        unsigned int sync_tol : 3;
        unsigned int sync_size : 3;
        unsigned int fifo_fill_condition : 1;
        unsigned int sync_on : 1;
    };
    uint8_t as_uint8;
} sxv1_sync_config_t;

typedef union {
    struct {
        unsigned int : 1;
        unsigned int address_filtering : 2;
        unsigned int crc_auto_clear_off : 1;
        unsigned int crc_on : 1;
        unsigned int dc_free : 2;
        unsigned int packet_format : 1;
    };
    uint8_t as_uint8;
} sxv1_packet_config1_t;

typedef union {
    struct {
        unsigned int aes_on : 1;
        unsigned int auto_rx_restart_on : 1;
        unsigned int restart_rx : 1;
        unsigned int : 1;
        unsigned int inter_packet_rx_delay : 4;
    };
    uint8_t as_uint8;
} sxv1_packet_config2_t;

typedef union {
    struct {
        unsigned int sync_address_match : 1;
        unsigned int auto_mode : 1;
        unsigned int timeout : 1;
        unsigned int rssi : 1;
        unsigned int pll_lock : 1;
        unsigned int tx_ready : 1;
        unsigned int rx_ready : 1;
        unsigned int mode_ready : 1;
    };
    uint8_t as_uint8;
} sxv1_irq_flags_1_t;

typedef union {
    struct {
        unsigned int : 1;
        unsigned int crc_ok : 1;
        unsigned int payload_ready : 1;
        unsigned int packet_send : 1;
        unsigned int fifo_overrun : 1;
        unsigned int fifo_level : 1;
        unsigned int fifo_not_empty : 1;
        unsigned int fifo_full : 1;
    };
    uint8_t as_uint8;
} sxv1_irq_flags_2_t;

typedef union {
	struct {
		unsigned int modulation_shaping : 2;
		unsigned int : 1;
		unsigned int modulation_type : 2;
		unsigned int data_mode : 2;
		unsigned int : 1;
	};
	uint8_t as_uint8;
} sxv1_data_modul_t;;


typedef union {
	struct {
		unsigned int lna_gain_select : 3;
		unsigned int lna_current_gain : 3;
		unsigned int : 1;
		unsigned int lna_zin : 1;
	};
	uint8_t as_uint8;
} sxv1_lna_t;


typedef union{
	 struct{
		unsigned int rx_bw_exp : 3; // default 101
		unsigned int rx_bw_mant : 2; // default 10
		unsigned int dcc_freq : 3;  // default 010
	};
	uint8_t as_uint8;
} sxv1_rxbw_t;

typedef struct {
	uint32_t bandwidth;
	sxv1_rxbw_t rxbw;
} sxv1_rxbw_entry_t;


////-------------------
// Attempt to be on-air compatible with LowPowerLab's SXV1 library
typedef struct {
    struct {
        uint8_t size;
        uint8_t target;
        uint8_t sender;
        uint8_t control;
    } header;
    uint8_t data[64];
} sxv1_air_packet_t;





#define SXV1_ACK_REQUEST 0x40
#define SXV1_ACK_RESPONSE 0x80

#pragma pack(pop)
#endif /* COMMON_SXV1_H_ */
