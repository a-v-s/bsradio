#include "si4x3x.h"

#include <endian.h>

#include "sxv1.h"



const static si4x3x_rxbw_entry_t m_rxbw_entries[] = { { 2600, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 1 } }, { 2800, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 2 } }, { 3100, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 3 } }, { 3200, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 4 } }, { 3700, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 5 } }, { 4200, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 6 } }, { 4500, { .ndec_exp = 5,
		.dwn3_bypass = 0, .filset = 7 } },

{ 4900, { .ndec_exp = 4, .dwn3_bypass = 0, .filset = 1 } }, { 5400, {
		.ndec_exp = 4, .dwn3_bypass = 0, .filset = 2 } }, { 5900, { .ndec_exp =
		4, .dwn3_bypass = 0, .filset = 3 } }, { 6100, { .ndec_exp = 4,
		.dwn3_bypass = 0, .filset = 4 } }, { 7200, { .ndec_exp = 4,
		.dwn3_bypass = 0, .filset = 5 } }, { 8200, { .ndec_exp = 4,
		.dwn3_bypass = 0, .filset = 6 } }, { 8800, { .ndec_exp = 4,
		.dwn3_bypass = 0, .filset = 7 } },

{ 9500, { .ndec_exp = 3, .dwn3_bypass = 0, .filset = 1 } }, { 10600, {
		.ndec_exp = 3, .dwn3_bypass = 0, .filset = 2 } }, { 11500, { .ndec_exp =
		3, .dwn3_bypass = 0, .filset = 3 } }, { 12100, { .ndec_exp = 3,
		.dwn3_bypass = 0, .filset = 4 } }, { 14200, { .ndec_exp = 3,
		.dwn3_bypass = 0, .filset = 5 } }, { 16200, { .ndec_exp = 3,
		.dwn3_bypass = 0, .filset = 6 } }, { 17500, { .ndec_exp = 3,
		.dwn3_bypass = 0, .filset = 7 } },

{ 18900, { .ndec_exp = 2, .dwn3_bypass = 0, .filset = 1 } }, { 21000, {
		.ndec_exp = 2, .dwn3_bypass = 0, .filset = 2 } }, { 22700, { .ndec_exp =
		2, .dwn3_bypass = 0, .filset = 3 } }, { 24000, { .ndec_exp = 2,
		.dwn3_bypass = 0, .filset = 4 } }, { 28200, { .ndec_exp = 2,
		.dwn3_bypass = 0, .filset = 5 } }, { 32200, { .ndec_exp = 2,
		.dwn3_bypass = 0, .filset = 6 } }, { 34700, { .ndec_exp = 2,
		.dwn3_bypass = 0, .filset = 7 } },

{ 37700, { .ndec_exp = 1, .dwn3_bypass = 0, .filset = 1 } }, { 41700, {
		.ndec_exp = 1, .dwn3_bypass = 0, .filset = 2 } }, { 45200, { .ndec_exp =
		1, .dwn3_bypass = 0, .filset = 3 } }, { 47900, { .ndec_exp = 1,
		.dwn3_bypass = 0, .filset = 4 } }, { 56200, { .ndec_exp = 1,
		.dwn3_bypass = 0, .filset = 5 } }, { 64100, { .ndec_exp = 1,
		.dwn3_bypass = 0, .filset = 6 } }, { 69200, { .ndec_exp = 1,
		.dwn3_bypass = 0, .filset = 7 } },

{ 75200, { .ndec_exp = 0, .dwn3_bypass = 0, .filset = 1 } }, { 83200, {
		.ndec_exp = 0, .dwn3_bypass = 0, .filset = 2 } }, { 90000, { .ndec_exp =
		0, .dwn3_bypass = 0, .filset = 3 } }, { 95300, { .ndec_exp = 0,
		.dwn3_bypass = 0, .filset = 4 } }, { 112100, { .ndec_exp = 0,
		.dwn3_bypass = 0, .filset = 5 } }, { 127900, { .ndec_exp = 0,
		.dwn3_bypass = 0, .filset = 6 } }, { 137900, { .ndec_exp = 0,
		.dwn3_bypass = 0, .filset = 7 } },

// There are some more entries... but we are not working
// that wide banded yet... also... the above follows a nice
// pattern so we can reduce it to a formula.
// The wide band entries do not follow the pattern above.
// Good enough for now.
// Termination
		{ 0, { 0, 0, 0 } }, };

#include "bshal_spim.h"

//bshal_spim_instance_t spi_radio_config;

#define SI4X3X_WRITE 0x80
#define SI4X3X_READ  0x00

int si4x3x_write_reg8(bsradio_instance_t *bsradio, uint8_t reg, uint8_t val) {
	uint8_t buff[2] = { reg | SI4X3X_WRITE, val };
	return bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), false);
}

int si4x3x_read_reg8(bsradio_instance_t *bsradio, uint8_t reg, uint8_t *val) {
	uint8_t buff[2] = { reg | SI4X3X_READ, 0xFF };
	int result = bshal_spim_transceive(&bsradio->spim, buff, sizeof(buff),
			false);
	*val = buff[1];
	return result;
}

int si4x3x_write_reg16(bsradio_instance_t *bsradio, uint16_t reg, uint16_t val) {
	si4x3x_spi_16bit_t buff;
	buff.reg = reg | SI4X3X_WRITE;
	buff.value = htobe16(val);
	return bshal_spim_transmit(&bsradio->spim, &buff, sizeof(buff), false);
}

int si4x3x_read_reg16(bsradio_instance_t *bsradio, uint16_t reg, uint16_t *val) {
	si4x3x_spi_16bit_t buff;
	buff.reg = reg | SI4X3X_READ;
	int result = bshal_spim_transceive(&bsradio->spim, &buff, sizeof(buff),
			false);
	*val = be16toh(buff.value);
	return result;
}

int si4x3x_clear_tx_fifo(bsradio_instance_t *bsradio) {
	si4x3x_reg_08_t r08;
	si4x3x_read_reg8(bsradio, 0x08, &r08);
	r08.ffclrtx = 1;
	si4x3x_write_reg8(bsradio, 0x08, r08.as_uint8);
	r08.ffclrtx = 0;
	si4x3x_write_reg8(bsradio, 0x08, r08.as_uint8);
	return 0;
}

int si4x3x_clear_rx_fifo(bsradio_instance_t *bsradio) {
	si4x3x_reg_08_t r08;
	si4x3x_read_reg8(bsradio, 0x08, &r08);
	r08.ffclrrx = 1;
	si4x3x_write_reg8(bsradio, 0x08, r08.as_uint8);
	r08.ffclrrx = 0;
	si4x3x_write_reg8(bsradio, 0x08, r08.as_uint8);
	return 0;
}

int si4x3x_write_fifo(bsradio_instance_t *bsradio, void *data, uint8_t size) {
	si4x3x_reg_3e_t r3e = { };
	r3e.pklen = size;
	si4x3x_clear_tx_fifo(bsradio);
	si4x3x_write_reg8(bsradio, 0x3e, r3e.as_uint8);
	uint8_t fifo_access = 0x7F | SI4X3X_WRITE;
	bshal_spim_transmit(&bsradio->spim, &fifo_access, 1, true);
	bshal_spim_transmit(&bsradio->spim, data, size, false);
	return 0;
}

int si4x3x_read_fifo(bsradio_instance_t *bsradio, void *data, uint8_t *size) {
	si4x3x_reg_4b_t r4b;
	int result = 0;
	si4x3x_read_reg8(bsradio, 0x4B, &r4b);
	if (*size < r4b.rxplen) {
		result = -1;
	} else {
		*size = r4b.rxplen;
	}
	uint8_t fifo_access = 0x7F | SI4X3X_READ;
	bshal_spim_transmit(&bsradio->spim, &fifo_access, 1, true);
	bshal_spim_receive( &bsradio->spim, data, *size, false);
	si4x3x_clear_rx_fifo(bsradio);
	return 0;
}
int si4x3x_set_frequency(bsradio_instance_t *bsradio, int kHz) {
	int fb, fc, hbsel;

	if (kHz < 240000) {
		// Out of range
		return -1;
	}
	if (kHz > 960000) {
		// Out of range
		return -1;
	}

//	if (kHz < 480000) {
//		// Low Band
//		fb = (kHz - 240000) / 10000;
//		fc = (float) (kHz % 10000) / 0.15625f;
//		hbsel = 0;
//	} else {
//		// High Band
//		fb = (kHz - 480000) / 20000;
//		fc = (float) (kHz % 20000) / 0.3125f;
//		hbsel = 1;
//	}

// Get the same without using floats
	int Hz = kHz * 1000;
	hbsel = Hz >= 480000000;
	fb = ((Hz >> hbsel) - 240000000) / 10000000;
	fc = (((Hz >> hbsel) % 10000000) * 20) / 3125;

	si4x3x_reg_75_t r75 = { 0 };
	r75.fb = fb;
	r75.hbsel = hbsel;
	r75.sbsel = 1; // AN440 says sbsel = 1 is recommended.
	//r75.sbsel = 0;
	si4x3x_write_reg8(bsradio, 0x75, r75.as_uint8);

	si4x3x_write_reg16(bsradio, 0x76, fc);

	return 0;
}

int si4x3x_set_sync_word32(bsradio_instance_t *bsradio, uint32_t sync_word) {
	uint8_t buffer[5] = { SI4X3X_WRITE | 0x36, sync_word, sync_word >> 8,
			sync_word >> 16, sync_word >> 24 };
	bshal_spim_transmit(&bsradio->spim, buffer, sizeof(buffer), false);
	si4x3x_reg_33_t r33;
	si4x3x_read_reg8(bsradio, 0x33, &r33.as_uint8);
	r33.synclen = 0b11;
	si4x3x_write_reg8(bsradio, 0x33, r33.as_uint8);
	return 0;
}

int si4x3x_set_bitrate(bsradio_instance_t *bsradio, int bps) {

	/*
	 The data rate can be calculated as:
	 TX_DR = 10^6 x txdr[15:0]/2^16 [bps] (if address 70[5] = 0)
	 TX_DR = 10^6 x txdr[15:0]/2^21 [bps] (if address 70[5] = 1)
	 */

	si4x3x_reg_70_t r70;
	si4x3x_read_reg8(bsradio, 0x70, &r70);

	uint16_t txdr;
	if (bps < 30000) {
		r70.txdtrtscale = 1;
		txdr = ((uint64_t)(bps) << 21) / 1000000;
	} else {
		r70.txdtrtscale = 0;
		txdr = ((uint64_t)(bps) << 16) / 1000000;
	}
	si4x3x_write_reg8(bsradio, 0x70, r70.as_uint8);

	si4x3x_write_reg16(bsradio, 0x6e, txdr);

	return 0;

}

int si4x3x_set_fdev(bsradio_instance_t *bsradio, int hz) {
	// 	Fd = 625 Hz x fd[8:0].

	int fd = hz / 625;

	si4x3x_reg_71_t r71;
	si4x3x_read_reg8(bsradio, 0x71, &r71);
	r71.fd8 = fd >> 8;
	si4x3x_write_reg8(bsradio, 0x71, r71.as_uint8);
	si4x3x_write_reg8(bsradio, 0x72, fd);

	return 0;

}

int si4x3x_set_bandwidth(bsradio_instance_t *bsradio, int hz) {
	si4x3x_reg_1c_t r1c;
	int i;
	for (i = 0; m_rxbw_entries[i].bandwidth; i++)
		if (m_rxbw_entries[i].bandwidth > hz)
			break;
	if (m_rxbw_entries[i].bandwidth) {
		r1c = m_rxbw_entries[i].rxbw;
		si4x3x_write_reg8(bsradio, 0x1c, r1c.as_uint8);
		return 0;
	}
	return -1;
}

int si4x3x_update_clock_recovery(bsradio_instance_t *bsradio) {
	// Clock Recovery registers 20 t/m 25 need to be
	// updated when the bandwidth filter, frequency deviation
	// or bitrate change to make reception work.

	si4x3x_reg_70_t r70;
	si4x3x_read_reg8(bsradio, 0x70, &r70);
	uint16_t r6e;
	si4x3x_read_reg16(bsradio, 0x6e, &r6e);

	uint64_t Rb = ((uint64_t)(1000000 * (uint64_t) r6e))
			>> (uint64_t)(r70.txdtrtscale ? 21 : 16);

	si4x3x_reg_72_t r72;
	si4x3x_read_reg8(bsradio, 0x72, &r72);
	int Fd = 625 * r72.fd;

	si4x3x_reg_1c_t r1c;
	si4x3x_read_reg8(bsradio, 0x1c, &r1c);

	uint64_t rxosr_val;
	if (r1c.ndec_exp < 3) {
		rxosr_val = (500000 * (1 + (2 * r1c.dwn3_bypass)))
				/ ((Rb * (1 + r70.enmanch)) >> (3 - r1c.ndec_exp));
	} else {
		rxosr_val = (500000 * (1 + (2 * r1c.dwn3_bypass)))
				/ ((Rb * (1 + r70.enmanch)) << (r1c.ndec_exp - 3));
	}

	uint64_t ncoff_val = ((uint64_t)(Rb) << (20 + r1c.ndec_exp + r70.enmanch))
			/ (500000 * (1 + 2 * r1c.dwn3_bypass));

	// Default value is off? Formula seems to match WDS output
	uint64_t crgain_val = 2
			+ (((uint64_t)(Rb) << (16 + r70.enmanch)) / (rxosr_val * Fd));

	// Default value appears to be
	//uint64_t crgain_val =  (((uint64_t)(Rb) << (15 + r70.enmanch)) / (rxosr_val * Fd));

	si4x3x_reg_20_t r20 = { };
	si4x3x_reg_21_t r21;
	si4x3x_read_reg8(bsradio, 0x21, &r21);
	r20.rxosr_7_0 = rxosr_val;
	r21.rxosr_10_8 = rxosr_val >> 8;
	si4x3x_write_reg8(bsradio, 0x20, r20.as_uint8);
	si4x3x_write_reg8(bsradio, 0x21, r21.as_uint8);

	si4x3x_write_reg16(bsradio, 0x22, ncoff_val);

	si4x3x_reg_24_t r24 = { };
	si4x3x_reg_25_t r25 = { };

	r24.crgain_10_8 = crgain_val >> 8;
	r25.crgain_7_0 = crgain_val;
	si4x3x_write_reg8(bsradio, 0x24, r24.as_uint8);
	si4x3x_write_reg8(bsradio, 0x25, r25.as_uint8);

	return 0;

}

int si4x3x_set_tx_power(bsradio_instance_t *bsradio, int tx_power) {
	/*
	 The output power is configurable
	 from +13 dBm to –8 dBm (Si4430/31), and
	 from +20 dBM to –1 dBM (Si4432) in ~3 dB steps.

	 txpow[2:0]=000 corresponds to min output power, while
	 txpow[2:0]=111 corresponds to max output power.

	 As I have not found a register that tells the 30/31 from the 32
	 I will assume we are on a 32, as these seems to be the commonly
	 sold module
	 */

	if (tx_power > 20)
		return -1;
	if (tx_power < -1)
		return -1;

	si4x3x_reg_6d_t r6d;
	si4x3x_read_reg8(bsradio, 0x6D, &r6d);

	if (bsradio->hwconfig.chip_variant == 2)
		r6d.txpow = (tx_power + 1) / 3;
	else
		r6d.txpow = (tx_power + 8) / 3;

	r6d.lna_sw = 1; // try
	return si4x3x_write_reg8(bsradio, 0x6D, r6d.as_uint8);
}

void si4x3x_configure_packet(bsradio_instance_t *bsradio) {
	// Now.... let's configure packet mode and such
	si4x3x_write_reg8(bsradio, 0x71, 0b00100011); //GFSK, FIFO
	si4x3x_write_reg8(bsradio, 0x05, 0xFF); //Enable Interrupts
	si4x3x_write_reg8(bsradio, 0x06, 0xFF); //Enable Interrupts

	si4x3x_write_reg8(bsradio, 0x30, 0b10001001); // disable crc for first tst

	// Set CRC to CCIT, as this is what SX123x uses. still nothing
	//si4x3x_write_reg8(0x30, 0b10001100);

	si4x3x_write_reg8(bsradio, 0x32, 0x00); // No Header
	si4x3x_write_reg8(bsradio, 0x33, 0x00); // No Header

	si4x3x_write_reg8(bsradio, 0x35, 0b00010010); // Preamble detection

	si4x3x_set_sync_word32(bsradio, 0xdeadbeef);

}

int si4x3x_set_mode(bsradio_instance_t *bsradio,si4x3x_mode_t mode) {
	//si4x3x_write_reg8(bsradio, 0x07, 0x04);
	return si4x3x_write_reg8(bsradio, 0x07, mode);
}

int si4x3x_receive_request(bsradio_instance_t *bsradio, sxv1_air_packet_t *p_request) {
	uint8_t reg, val;
	si4x3x_reg_03_t r03 = { };
	si4x3x_reg_04_t r04 = { };

	si4x3x_read_reg8(bsradio, 0x03, &r03);
			si4x3x_read_reg8(bsradio, 0x04, &r04);

	uint8_t rssi_raw;
	if (r04.irssi) {
			si4x3x_read_reg8(bsradio,0x26, &rssi_raw);
		}
	if (r03.ipkvalid) {
//		bshal_delay_ms(1);


		uint8_t size = sizeof(sxv1_air_packet_t);
			si4x3x_read_fifo(bsradio, p_request, &size);



			int8_t rssi = (-rssi_raw) / 2;
			printf("RSSI val %d\n", rssi);
			char buff[32] = { 0 };
			sprintf(buff, "RSSI %d\n", rssi);
			print(buff, 0);

			si4x3x_clear_rx_fifo(bsradio);
			si4x3x_set_mode(bsradio, si4x3x_mode_reveive);
			return 0;



	}

	return -1;


}

int si4x3x_send_request(bsradio_instance_t *bsradio, sxv1_air_packet_t *p_request,
		sxv1_air_packet_t *p_response) {
	uint8_t reg;
	uint8_t val;

//	// Clear fifo
	si4x3x_clear_tx_fifo(bsradio);

	si4x3x_write_fifo(bsradio, p_request, p_request->header.size);

	si4x3x_write_reg8(bsradio, 0x07, 0x08);



	si4x3x_reg_03_t r03 = { };
	si4x3x_reg_03_t r04 = { };
	while (!r03.ipksent) {
		si4x3x_read_reg8(bsradio, 0x03, &r03);
		si4x3x_read_reg8(bsradio, 0x04, &r04);
	}
	bshal_delay_ms(1);
	return 0;
}
