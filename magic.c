/*
 * magic.c
 *
 *  Created on: 24 mei 2023
 *      Author: andre
 */

#include "12500_10.h"

#include <bshal_spim.h>
#include <stdbool.h>

#include "radio.h"

void si4x6x_load_magic_values(bsradio_instance_t *bsradio) {

	/*
	 *


	 The magic values needed to make the Si4x6x work are
	 RF_MODEM_TX_RAMP_DELAY_12 and RF_MODEM_RAW_CONTROL_10

	 These are the magic values, compared to their default values, to determine
	 which of them have changed:

	 RF_MODEM_TX_RAMP_DELAY_12 0x11, 0x20, 0x0C, 0x18, 0x01, 0x80, 0x08, 0x03, 0xC0, 0x00, 0x20, 0x20, 0x00, 0xE8, 0x01, 0x2C
	 0x01, 0x00, 0x08, 0x03, 0xC0, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x4B,
	 xxxx              xxxxx xxxxxxxxxxx
	 MODEM_DECIMATION_CFG1
	 MODEM_IFPKD_THRESHOLDS
	 MODEM_BCR_OSR


	 #define RF_MODEM_RAW_CONTROL_10 0x11, 0x20, 0x0A, 0x45, 0x83, 0x00, 0xD5, 0x01, 0x00, 0xFF, 0x06, 0x00, 0x18, 0x40
	 0x02, 0x00, 0xA3, 0x02, 0x80, 0xFF, 0x0C, 0x01, 0x00, 0x40,
	 xxxx   xxxxxxxxx  xxxx  xxxxx       xxxx  xxxxxxxxxxxx
	 MODEM_RAW_CONTROL
	 MODEM_RAW_EYE
	 MODEM_ANT_DIV_MODE
	 MODEM_ANT_DIV_CONTROL
	 MODEM_RSSI_JUMP_THRESH
	 MODEM_RSSI_CONTROL


	 *	MODEM_DECIMATION_CFG1
	 *	MODEM_IFPKD_THRESHOLDS
	 *	MODEM_BCR_OSR

	 *	MODEM_RAW_CONTROL
	 *	MODEM_RAW_EYE
	 *	MODEM_ANT_DIV_MODE
	 *	MODEM_ANT_DIV_CONTROL
	 *	MODEM_RSSI_JUMP_THRESH
	 *	MODEM_RSSI_CONTROL


	 MODEM_DECIMATION_CFG1
	 0x10 -> 0xC0
	 00 01 000 0
	 11 00 000 0
	 NDEC0 --> Decimate by 1 --> 1.
	 NDEC1 --> Decimate by 2 --> 1.
	 NDEC2 --> Decimate by 1 --> 8.

	 The decimation ratio of each decimator circuit block is 2NDEC.
	 In (G)FSK mode the RX data samples pass through two cascaded
	 decimator blocks, and thus the decimation ratio is 2^(NDEC1+NDEC2).
	 In OOK mode, the post-demodulated RX data samples pass through an
	 additional decimator block with decimation ratio 2^NDEC0.
	 If not in OOK mode, this decimation coefficient should be set NDEC0 = 0.

	 So, the value goes from 2*1 = 2 to 1*8 = 8.
	 The MODEM_BCR_OSR goes from 75 to 300
	 So both values are multiplied by 4

	 Anyhow, these values are related to the bandwidth filter, so I should
	 compare the output for different filter values to derive their relation.



	 MODEM_RAW_CONTROL  2 -> 4
	 MODEM_RAW_EYE Eye-open detector threshold.

	 *
	 */
	uint8_t magic_values[] = //RADIO_CONFIGURATION_DATA_ARRAY;

			{
			//     0x07, RF_POWER_UP,
			//     0x08, RF_GPIO_PIN_CFG,
			//     0x06, RF_GLOBAL_XO_TUNE_2,
			//     0x05, RF_GLOBAL_CONFIG_1,
			//     0x05, RF_INT_CTL_ENABLE_1,
			//     0x08, RF_FRR_CTL_A_MODE_4,

			//     0x0D, RF_PREAMBLE_TX_LENGTH_9,
			//     0x0E, RF_SYNC_CONFIG_10,

			//     0x10, RF_PKT_CRC_CONFIG_12,
			//     0x10, RF_PKT_RX_THRESHOLD_12,
			//     0x10, RF_PKT_FIELD_3_CRC_CONFIG_12,
			//     0x10, RF_PKT_RX_FIELD_1_CRC_CONFIG_12,
			//     0x09, RF_PKT_RX_FIELD_4_CRC_CONFIG_5,
			//     0x08, RF_PKT_CRC_SEED_31_24_4,

			//     0x10, RF_MODEM_MOD_TYPE_12,
			//     0x05, RF_MODEM_FREQ_DEV_0_1,

					0x10, RF_MODEM_TX_RAMP_DELAY_12,
					//0x10, RF_MODEM_BCR_NCO_OFFSET_2_12,
//	        0x07, RF_MODEM_AFC_LIMITER_1_3,
//	        0x05, RF_MODEM_AGC_CONTROL_1,
//	        0x10, RF_MODEM_AGC_WINDOW_SIZE_12,
					0x0E, RF_MODEM_RAW_CONTROL_10,
//	        0x06, RF_MODEM_RAW_SEARCH2_2,
//	        0x06, RF_MODEM_SPIKE_DET_2,
//	        0x05, RF_MODEM_RSSI_MUTE_1,
//	        0x09, RF_MODEM_DSA_CTRL1_5,
//	        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12,
//	        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12,
//	        0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12,

//	        0x08, RF_PA_MODE_4,
//	        0x0B, RF_SYNTH_PFDCP_CPFF_7,
//	        0x10, RF_MATCH_VALUE_1_12,

					//       0x0C, RF_FREQ_CONTROL_INTE_8,
					0x00 };

	uint8_t *magic_pointer = magic_values;
	while (*magic_pointer) {
		bshal_spim_transmit(&bsradio->spim, magic_pointer + 1, *magic_pointer,
				false);
		magic_pointer += 1 + (*magic_pointer);
		bshal_delay_ms(10);
	}

}

