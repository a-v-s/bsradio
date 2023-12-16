/*
 * si4x6x.h
 *
 *  Created on: 16 mei 2023
 *      Author: andre
 */

#ifndef SI4X6X_H_
#define SI4X6X_H_

#include <stdint.h>
#include "radio.h"



// BOOT_COMMANDS
#define SI4X6X_CMD_POWER_UP					0x02

// COMMON_COMMANDS
#define SI4X6X_CMD_NOP 						0x00
#define SI4X6X_CMD_PART_INFO 				0x01
#define SI4X6X_CMD_FUNC_INFO 				0x10
#define SI4X6X_CMD_SET_PROPERTY 			0x11
#define SI4X6X_CMD_GET_PROPERTY 			0x12
#define SI4X6X_CMD_GPIO_PIN_CFG 			0x13
#define SI4X6X_CMD_FIFO_INFO 				0x15
#define SI4X6X_CMD_GET_INT_STATUS 			0x20
#define SI4X6X_CMD_REQUEST_DEVICE_STATE 	0x33
#define SI4X6X_CMD_CHANGE_STATE 			0x34
#define SI4X6X_CMD_READ_CMD_BUFF 			0x44
#define SI4X6X_CMD_FRR_A_READ 				0x50
#define SI4X6X_CMD_FRR_B_READ 				0x51
#define SI4X6X_CMD_FRR_C_READ 				0x53
#define SI4X6X_CMD_FRR_D_READ 				0x57

// IR_CAL_COMMANDS
#define SI4X6X_CMD_IRCAL 					0x17
#define SI4X6X_CMD_IRCAL_MANUAL 			0x1a

// TX_COMMANDS
#define SI4X6X_CMD_START_TX 				0x31
#define SI4X6X_CMD_TX_HOP 					0x37
#define SI4X6X_CMD_WRITE_TX_FIFO 			0x66

//	RX_COMMANDS
#define SI4X6X_CMD_PACKET_INFO 				0x16
#define SI4X6X_CMD_GET_MODEM_STATUS 		0x22
#define SI4X6X_CMD_START_RX 				0x32
#define SI4X6X_CMD_RX_HOP 					0x36
#define SI4X6X_CMD_READ_RX_FIFO 			0x77

//	ADVANCED_COMMANDS
#define SI4X6X_CMD_GET_ADC_READING 			0x14
#define SI4X6X_CMD_GET_PH_STATUS 			0x21
#define SI4X6X_CMD_GET_CHIP_STATUS 			0x23

#pragma pack (push,1)
typedef struct {
	unsigned int chiprev :8;
	unsigned int part_be :16;
	unsigned int pbuild :8;
	unsigned int id_be :16;
	unsigned int customer :8;
	unsigned int romid :8;
} si4x6x_part_info_t;

typedef struct {
	union {
		struct {
			unsigned int func :6;
			unsigned int :1;
			unsigned int patch :1;
		};
		uint8_t as_uint8;
	} boot_options;
	union {
		struct {
			unsigned int txco :6;
		};
		uint8_t as_uint8;
	} xtal_options;
	unsigned int xo_freq_be :32;
} si4x6x_cmd_power_up_t;

typedef struct {
	unsigned int channel :8;
	union {
		struct {
			unsigned int start :2;
			unsigned int retransmit :1;
			unsigned int update :1;
			unsigned int tx_complete_state :4;
		};
		unsigned int condition :8;
	};
	unsigned int tx_len_12_8 :5;
	unsigned int :3;
	unsigned int tx_len_7_0 :8;
	unsigned int tx_delay :8;
	unsigned int num_repeat :8;
} si4x6x_cmd_start_tx_t;

typedef struct {
	uint8_t rx_fifo_count;
	uint8_t tx_fifo_space;
} si4x6x_cmd_fifo_count_response_t;

typedef struct {
	unsigned int channel :8;
	union {
		struct {
			unsigned int start :2;
			unsigned int :1;
			unsigned int update :1;
		};
		uint8_t condition;
	};
	unsigned int rx_len_12_8 :5;
	unsigned int :3;
	unsigned int rx_len_7_0 :8;
	union {
		unsigned int rxtimeout_state :4;
		uint8_t next_state1;
	};
	union {
		unsigned int rxvalid_state :4;
		uint8_t next_state2;
	};
	union {
		unsigned int rxinvalid_state :4;
		uint8_t next_state3;
	};

} si4x6x_cmd_start_rx_t;

typedef struct {

} si4x6x_cmd_get_int_status_t;

typedef struct {
	unsigned int revext :8;
	unsigned int revbranch :8;
	unsigned int revint :8;
	unsigned int patch_be :16;
	unsigned int func :8;
} si4x6x_func_info_t;

typedef struct {
	unsigned int inte :7;
	unsigned int :1;
	unsigned int frac_19_16 :4;
	unsigned int :4; // padding
	unsigned int frac_15_8 :8;
	unsigned int frac_7_0 :8;
} si4x6x_prop_freq_control_intefraq_t;

typedef union {
	struct {
		unsigned int band :3;
		unsigned int sy_sel :1;
		unsigned int force_sy_recall :1;
		unsigned int :3;
	};
	uint8_t as_uint8;
} si4x6x_prop_modem_clkgen_band_t;

#pragma pack(pop)

int si4x6x_command(bsradio_instance_t *bsradio,uint8_t cmd, void *request, uint8_t request_size,
		void *response, uint8_t response_size);
int si4x6x_set_frequency(bsradio_instance_t *bsradio, int kHz);
int si4x6x_set_bitrate(bsradio_instance_t *bsradio, int bps);
int si4x6x_set_fdev(bsradio_instance_t *bsradio, int hz);

#endif /* SI4X6X_H_ */
