/*
 * bsradio.h
 *
 *  Created on: 9 jun. 2023
 *      Author: andre
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <protocol.h>
#include <bshal_spim.h>

#ifndef BSRADIO_MAX_PACKET_LEN
#define BSRADIO_MAX_PACKET_LEN (60)
#endif

#ifndef BSRADIO_SEND_QUEUE_LEN
#define BSRADIO_SEND_QUEUE_LEN (4)
#endif

#pragma pack (push, 1)

typedef enum {
	chip_brand_st = 0x01,
	chip_brand_semtech = 0x02,
	chip_brand_silabs = 0x03,
	chip_brand_ti = 0x04,
	chip_brand_nordic = 0x05,
	chip_brand_amiccom = 0x06,
	chip_brand_onsemi = 0x07,
} bsradio_chip_brand_t;

typedef enum {
	module_brand_generic = 0x01,
	module_brand_radiocontrolli = 0x02,
	module_brand_hoperf = 0x03,
	module_brand_gnicerf = 0x04,
	module_brand_dreamlnk = 0x05,
	module_brand_aithinker = 0x06,
} bsradio_module_brand_t;

typedef enum {
	module_variant_xl4432_smt = 0x01, module_variant_rf4432pro = 0x02,

	module_variant_rf4463pro = 0x01,

} bsradio_module_variant_t;

typedef enum {
	antenna_type_trace = 0x01,
	antenna_type_chip = 0x02,
	antenna_type_spring = 0x03,

	antenna_type_undefined = 0xFF,
} bsradio_antenna_type_t;

typedef struct {
	bsradio_chip_brand_t chip_brand :8;
	unsigned int chip_type :8;
	unsigned int chip_variant :16;
	bsradio_module_brand_t module_brand :8;
	unsigned int module_variant :8;
	unsigned int frequency_band :16;
	int tune :8;
	unsigned int pa_config :8;
	bsradio_antenna_type_t antenna_type :8;
	unsigned int :8;
	unsigned int xtal_freq :32;
} bsradio_hwconfig_t;

typedef enum {
	// TODO
	modulation_none = 0x00,
	modulation_ook = 0x10,
	modulation_2fsk = 0x20,
	modulation_4fsk = 0x21,
	modulation_lora = 0x80,

} bsradio_modulation_t;

typedef enum {
	// CRC modes supported by Si4x6x
	// SX1231 CRC Polynomial =X16 + X12 + X5 + 1. This would be CCIT_16, mode 5
	// TODO formatting
	crc_disable = 0,
	ITU_T_CRC8 = 1,		// ITU-T CRC8: X8+X2+X+1.
	IEC_16 = 2,			//IEC-16: X16+X14+X12+X11+X9+X8+X7+X4+X+1.
	BAICHEVA_16 = 3,	// Baicheva-16: X16+X15+X12+X7+X6+X4+X3+1.
	CRC_16_IBM = 4,		//CRC-16 (IBM): X16+X15+X2+1.
	CCITT_16 = 5,		// CCIT-16: X16+X12+X5+1.
	KOOPMAN = 6,// Koopman: X32+X30+X29+X28+X26+X20+X19+X17+X16+X15+X11+X10+X7+X6+X4+X2+X+1.
	IEEE_802_3 = 7,	//IEEE 802.3: X32+X26+X23+X22+X16+X12+X11+X10+X8+X7+X5+X4+X2+X+1.
	CASTAGNOLI = 8,	// Castagnoli: X32+X28+X27+X26+X25+X23+X22+X20+X19+X18+X14+X13+X11+X10+X9+X8+X6+1.
	CRC_16_DNP = 9,	//  	CRC-16-DNP: X16+X13+X12+X11+X10+X8+X6+X5+X2+1.
} bsradio_crc_t;

typedef struct {
	uint32_t frequency_kHz;
	uint32_t freq_dev_hz;
	uint32_t bandwidth_hz;
	uint32_t birrate_bps;
	bsradio_modulation_t modulation :8;
	uint8_t modulation_shaping; // gaussian filter
	bool node_id_enable :1;
	bool broadcast_id_enable :1;
	bsradio_crc_t crc :6;
	uint8_t network_id[8];
	uint8_t network_id_size;
	uint8_t node_id;
	uint8_t broadcast_id;
	int8_t tx_power_dBm;
} bsradio_rfconfig_t;



typedef struct {
	uint8_t length;
	uint8_t to;
	uint8_t from;
	unsigned int seq_nr :3;
	unsigned int retry_cnt :3;
	unsigned int ack_request :1;
	unsigned int ack_response :1;
	uint8_t payload[BSRADIO_MAX_PACKET_LEN];
	int8_t rssi;
} bsradio_packet_t;


// Attempt at compatibility with
// https://github.com/LowPowerLab/RFM69/tree/master
// Replace sequence numbre and retry counter with extended
// address bits. This gives more addresses but less robust
// packet retrying. Note, this is not implemented yet.
typedef struct {
	uint8_t length;
	uint8_t to;
	uint8_t from;
	unsigned int  to_hi:3; 		// TODO order to/from not
	unsigned int  from_hi:3;	 // verifieds
	unsigned int ack_request :1;
	unsigned int ack_response :1;
	uint8_t payload[BSRADIO_MAX_PACKET_LEN];
	int8_t rssi;
} bsradio_lowpowerlab_compat_packet_t;

typedef struct {
	bsradio_packet_t queue[BSRADIO_SEND_QUEUE_LEN];
} bsradio_send_queue_t;

typedef enum {
	// Initially, we support off, receive and transmit
	// Each with the upper nibble inxcreased, to allow for
	// future variants, eg. various states of standdby in the 0x0?
	// and the sw1231 LISTEN mode as for example 0x11
	mode_off = 0x00,
	mode_receive = 0x10,
	mode_tranmit = 0x20,
} bsradio_mode_t;

struct bsradio_instance_t;

typedef struct {
	int (*set_frequency)( struct bsradio_instance_t *bsradio,int kHz) ;
	int (*set_tx_power)(struct bsradio_instance_t *bsradio,int tx_power);
	int (*set_bitrate)(struct bsradio_instance_t *bsradio,int bps) ;
	int (*set_fdev)(struct bsradio_instance_t *bsradio,int hz) ;
	int (*set_bandwidth)(struct bsradio_instance_t *bsradio,int hz) ;
	int (*init)(struct bsradio_instance_t *bsradio) ;
	int (*set_network_id)(struct bsradio_instance_t *bsradio,char *sync_word, size_t size) ;
	int (*set_mode)(struct bsradio_instance_t *bsradio,bsradio_mode_t mode) ;
	int (*recv_packet)(struct bsradio_instance_t *bsradio ,bsradio_packet_t *p_packet) ;
	int (*send_packet)(struct bsradio_instance_t *bsradio, bsradio_packet_t *p_packet) ;
} bsradio_driver_t;

typedef struct bsradio_instance_t{
	bshal_spim_instance_t spim;
	bsradio_hwconfig_t hwconfig;
	bsradio_rfconfig_t rfconfig;
	bsradio_send_queue_t send_queue;
	bsradio_driver_t driver;
} bsradio_instance_t;



////-------------------
// Old Attempt to be on-air compatible with LowPowerLab's RFM69 library
// Note: this is in here to compile old demos, has to be removed!
typedef struct {
    struct {
        uint8_t size;
        uint8_t target;
        uint8_t sender;
        uint8_t control;
    } header;
    uint8_t data[64];
} sxv1_air_packet_t;
#pragma pack(pop)


int bsradio_set_frequency( struct bsradio_instance_t *bsradio,int kHz) ;
int bsradio_set_tx_power(struct bsradio_instance_t *bsradio,int tx_power);
int bsradio_set_bitrate(struct bsradio_instance_t *bsradio,int bps) ;
int bsradio_set_fdev(struct bsradio_instance_t *bsradio,int hz) ;
int bsradio_set_bandwidth(struct bsradio_instance_t *bsradio,int hz) ;
int bsradio_init(struct bsradio_instance_t *bsradio) ;
int bsradio_set_network_id(struct bsradio_instance_t *bsradio,char *sync_word, size_t size) ;
int bsradio_set_node_id(struct bsradio_instance_t *bsradio, char node_id);
int bsradio_set_mode(struct bsradio_instance_t *bsradio,bsradio_mode_t mode) ;
int bsradio_recv_packet(struct bsradio_instance_t *bsradio ,bsradio_packet_t *p_packet) ;
int bsradio_send_packet(struct bsradio_instance_t *bsradio, bsradio_packet_t *p_packet) ;

int bsradio_send_request(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_request, bsradio_packet_t *p_response) ;

#endif /* RADIO_H_ */

