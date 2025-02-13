#include "sxv1.h"
#include "bsradio.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <bshal_spim.h>
#include <bshal_gpio.h>
#include <bshal_delay.h>

bool g_sxv1_interrupt_flag;
extern bshal_spim_instance_t spi_radio_config;

// Looks like a pettern, we can optimise this
// so we don't need to keep a table
const static sxv1_rxbw_entry_t m_rxbw_entries[] = { { 2600,
		{ { 7, 0b10, 0b010 } } }, { 3100, { { 7, 0b01, 0b010 } } }, { 3900, { {
		7, 0b00, 0b010 } } },

{ 5200, { { 6, 0b10, 0b010 } } }, { 6300, { { 6, 0b01, 0b010 } } }, { 7800, { {
		6, 0b00, 0b010 } } },

{ 10400, { { 5, 0b10, 0b010 } } }, { 12500, { { 5, 0b01, 0b010 } } }, { 15600, {
		{ 5, 0b00, 0b010 } } },

{ 20800, { { 4, 0b10, 0b010 } } }, { 25000, { { 4, 0b01, 0b010 } } }, { 31300, {
		{ 4, 0b00, 0b010 } } },

{ 41700, { { 3, 0b10, 0b010 } } }, { 50000, { { 3, 0b01, 0b010 } } }, { 62500, {
		{ 3, 0b00, 0b010 } } },

{ 83300, { { 2, 0b10, 0b010 } } }, { 100000, { { 2, 0b01, 0b010 } } }, { 125000,
		{ { 2, 0b00, 0b010 } } },

{ 116700, {{ 1, 0b10, 0b010 }} }, { 200000, {{ 1, 0b01, 0b010 }} }, { 250000, { { 1,
		0b00, 0b010 } } },

{ 333300, { { 0, 0b10, 0b010 } } }, { 400000, {{ 0, 0b01, 0b010 }} }, { 500000,
		{{ 0, 0b00, 0b010 } } },

// Termination
		{ 0, { { 0, 0, 0 } } },

};

int sxv1_write_reg(bsradio_instance_t *bsradio, uint8_t reg, uint8_t val) {
	uint8_t buff[2] = { reg | SXV1_WRITE, val };
	return bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), false);
}

int sxv1_read_reg(bsradio_instance_t *bsradio, uint8_t reg, uint8_t *val) {
	uint8_t buff[2] = { reg | SXV1_READ, 0xFF };
	int result = bshal_spim_transceive(&bsradio->spim, buff, sizeof(buff),
			false);
	*val = buff[1];
	return result;
}

int sxv1_write_fifo_raw(bsradio_instance_t *bsradio, void *data, uint8_t size) {
	int result;
	uint8_t buff[1] = { SXV1_REG_FIFO | SXV1_WRITE };
	result = bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), true);
	if (result)
		return result;
	result = bshal_spim_transmit(&bsradio->spim, &size, sizeof(size), true);
	if (result)
		return result;

	return bshal_spim_transmit(&bsradio->spim, data, size, false);

}

int sxv1_write_fifo(bsradio_instance_t *bsradio, bsradio_packet_t *packet) {
	int result;
	uint8_t buff[1] = { SXV1_REG_FIFO | SXV1_WRITE };
	result = bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), true);
	if (result)
		return result;
	return bshal_spim_transmit(&bsradio->spim, packet, packet->length, false);

}

int sxv1_read_fifo_raw(bsradio_instance_t *bsradio, void *data, uint8_t *size) {
	int result;
	uint8_t buff[1] = { SXV1_REG_FIFO | SXV1_READ };
	result = bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), true);
	if (result)
		return result;
	uint8_t recv_size = 0;

	// temporary disabled for testing
	// using fixed size packets for debugging purposes
	if (1) {
		result = bshal_spim_receive(&bsradio->spim, &recv_size, 1, true);
		if (result)
			return result;
		if (recv_size > *size)
			return -1;
		*size = recv_size;
	} else {
		recv_size = *size;
	}
	return bshal_spim_receive(&bsradio->spim, data, recv_size, false);
}

int sxv1_read_fifo(bsradio_instance_t *bsradio, bsradio_packet_t *packet) {
	int result;
	uint8_t buff[1] = { SXV1_REG_FIFO | SXV1_READ };
	uint8_t *packet_data = (uint8_t*) packet;
	memset(packet, 0, sizeof(*packet));
	result = bshal_spim_transmit(&bsradio->spim, buff, sizeof(buff), true);
	if (result)
		return result;

	result = bshal_spim_receive(&bsradio->spim, packet, 1, true);

	if (packet->length == 0 || (packet->length - 1) > sizeof (bsradio_packet_t) ) {
		// Read FIFO anyway when bad data is detected. In a hope to fix the issue
		// we are getting gatbage data after a while. (It might return unread data??)
		puts("Bad size, discarding data, reading fifo anyway");
		bshal_spim_receive(&bsradio->spim, packet_data + 1, sizeof (bsradio_packet_t) +1,
					false);
		return -1;
	}

	result = bshal_spim_receive(&bsradio->spim, packet_data + 1, packet->length,
			false);

	return result;
}

int sxv1_set_frequency(struct bsradio_instance_t *bsradio, int kHz) {

	kHz += bsradio->hwconfig.tune;

	// Without the need of float

	int regval = ((uint64_t)(1000 * kHz) << 19) / bsradio->hwconfig.xtal_freq;

	int status;

	/*
	 > The Frf setting is split across three bytes. A change in the center frequency is only
	 > taken into account when the Least Significant Byte FrfLsb in RegFrfLsb is written.

	 This suggest LSB should be written last
	 */

	status = sxv1_write_reg(bsradio, SXV1_REG_FRFMSB,
			(regval & 0xFF0000) >> 16);
	if (status)
		return status;
	status = sxv1_write_reg(bsradio, SXV1_REG_FRFMID, (regval & 0xFF00) >> 8);
	if (status)
		return status;
	status = sxv1_write_reg(bsradio, SXV1_REG_FRFLSB, regval & 0xFF);
	if (status)
		return status;
	return 0;
}

int sxv1_set_mode_internal(bsradio_instance_t *bsradio, sxv1_mode_t mode) {
	sxv1_val_opmode_t val;
	int status = sxv1_read_reg(bsradio, SXV1_REG_OPMODE,
			&val.as_uint8);
	if (status)
		return status;
	if (val.mode == mode)
		return 0;

	// Before writing the actual mode, as 01 in rx mode is Payload Ready
	// However in tx mode it is tx ready, generating an interrupt on entering
	switch (mode) {
	// TODO: make this neat
	case sxv1_mode_rx:
		// Enable Payload Ready on DIO0
		sxv1_write_reg(bsradio,SXV1_REG_DIOMAPPING1, 0b01000000);

		// RSSI
		sxv1_rssiconfig_t rssiconfig = {};
		rssiconfig.start = 1;
		sxv1_write_reg(bsradio, SXV1_REG_RSSICONFIG, rssiconfig.as_uint8);

		break;
	case sxv1_mode_tx:
		// Disable IRQ on DIO0 in TX mode
		sxv1_write_reg(bsradio,SXV1_REG_DIOMAPPING1, 0b10000000);
		break;
	}


	val.mode = mode;
	status = sxv1_write_reg(bsradio, SXV1_REG_OPMODE, val.as_uint8);
	if (status)
		return status;
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };

	// TODO
	extern uint32_t get_time_us(void);

	int timeout_us = get_time_us() + SXV1_MODESWITCH_TIMEOUT_US;
	while (!irq_flags_1.mode_ready) {
		status = sxv1_read_reg(bsradio, SXV1_REG_IRQFLAGS1,
				&irq_flags_1.as_uint8);
		if (timeout_us < get_time_us()) {
			// timeout
			return -1;
		}
	}





	return 0;
}

int sxv1_set_mode(struct bsradio_instance_t *bsradio, bsradio_mode_t mode) {
	int status;
	switch (mode) {
	case mode_off:
		return sxv1_set_mode_internal(bsradio, sxv1_mode_standby);
	case mode_receive:
		status = sxv1_set_mode_internal(bsradio, sxv1_mode_rx);
		if (status)
			return status;
		return sxv1_rx_restart(bsradio);
	case mode_tranmit:
		return sxv1_set_mode_internal(bsradio, sxv1_mode_tx);


	default:
		return -1;
	}
}

int sxv1_calibarte_rc(bsradio_instance_t *bsradio) {
	int result;
	result = sxv1_set_mode_internal(bsradio, sxv1_mode_standby);
	if (result)
		return result;
	result = sxv1_write_reg(bsradio, SXV1_REG_OSC1, 0x80);
	if (result)
		return result;
	uint8_t regval = 0x00;
	while (!regval) {
		// TODO Add Timeout
		result = sxv1_read_reg(bsradio, SXV1_REG_OSC1, &regval);
		if (result)
			return result;
	}
	return 0;
}

int sxv1_set_network_id(struct bsradio_instance_t *bsradio, char *sync_word,
		size_t size) {
	if (size < 1)
		return -1;
	if (size > 8)
		return -1;

	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = size - 1;
	config.sync_tol = 0;
	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);

	int result;
	for (int i = 0; i < size; i++) {
		result = sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE1 + i, sync_word[i]);
		if (result)
			return result;
	}
	return 0;
}

int sxv1_set_sync_word32(bsradio_instance_t *bsradio, uint32_t sync_word) {
	//debug_println("Setting SYNC word to %08X", sync_word);
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 3; // size = sync_size + 1, thus 4
	config.sync_tol = 0;
	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);

	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE3, (sync_word & 0xFF0000) >> 16);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE4,
			(sync_word & 0xFF000000) >> 24);
	return 0;
}

int sxv1_set_sync_word_24(bsradio_instance_t *bsradio, uint32_t sync_word) {
	//debug_println("Setting SYNC word to %08X", sync_word);
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 2; // size = sync_size + 1, thus 3
	config.sync_tol = 0;
	sxv1_write_reg(bsradio, SXV1_REG_SYNCCONFIG, config.as_uint8);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE1, sync_word >> 24);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE2, sync_word >> 16);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE3, sync_word >> 8);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE4, sync_word );

	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE3, (sync_word & 0xFF0000) >> 16);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE4, (sync_word & 0xFF000000) >> 24);
	return 0;
}

int sxv1_set_sync_word_16(bsradio_instance_t *bsradio, uint16_t sync_word) {
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 1; // size = sync_size + 1, thus 2
	config.sync_tol = 0;
	sxv1_write_reg(bsradio, SXV1_REG_SYNCCONFIG, config.as_uint8);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	return 0;
}

int sxv1_set_sync_word_8(bsradio_instance_t *bsradio, uint8_t sync_word) {
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 0; // size = sync_size + 1, thus 1
	config.sync_tol = 0;
	sxv1_write_reg(bsradio, SXV1_REG_SYNCCONFIG, config.as_uint8);
	sxv1_write_reg(bsradio, SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	return 0;
}

int sxv1_rx_restart(bsradio_instance_t *bsradio) {
	// Restarting RX before changing to TX mode is something done
	// in the LowPowerLab library, with a comment it prevents a deadlock

	sxv1_packet_config2_t config2;
	sxv1_read_reg(bsradio, SXV1_REG_PACKETCONFIG2, &config2.as_uint8);
	config2.restart_rx = 1;
	sxv1_write_reg(bsradio, SXV1_REG_PACKETCONFIG2, config2.as_uint8);
	return 0;
}

int sxv1_set_tx_power(struct bsradio_instance_t *bsradio, int tx_power) {
	// For SXV1HW Module
	// As the module states up to 20 dBm
	// it should output at PA_BOOST

	sxv1_val_palevel_t pa_level;
	if (bsradio->hwconfig.pa_config) {

		if (tx_power < -3)
			tx_power = -3;
		if (tx_power > 17)
			tx_power = 17;

		if ((tx_power + 18) <= 0b11111) {
			// PA1 only

			pa_level.pa0_on = 0;
			pa_level.pa1_on = 1;
			pa_level.pa2_on = 0;
			pa_level.output_power = tx_power + 18;
		} else if ((tx_power + 14) <= 0b11111) {
			// PA1 + PA2

			pa_level.pa0_on = 0;
			pa_level.pa1_on = 1;
			pa_level.pa2_on = 1;
			pa_level.output_power = tx_power + 14;
		}
	} else {
		if (tx_power < -18)
			tx_power = -18;
		if (tx_power > 13)
			tx_power = 13;
		pa_level.pa0_on = 1;
		pa_level.pa1_on = 0;
		pa_level.pa2_on = 0;
		pa_level.output_power = tx_power + 18;
	}
	sxv1_write_reg(bsradio, SXV1_REG_PALEVEL, pa_level.as_uint8);
	return 0;
}

void sxv1_irq_handler(void) {
	g_sxv1_interrupt_flag = true;
}

int sxv1_init(bsradio_instance_t *bsradio) {
	uint8_t chip_version = -1;
	sxv1_read_reg(bsradio, SXV1_REG_VERSION, &chip_version);

	switch (chip_version) {
	case 0x23:
		// SX123x,
		break;
	case 0x24:
		// SX1231H, RFM69
		break;
	default:
		// Known chips have either value 0x23 or 0x24
		// If another value is returned, it is an unknown chip
		// Or more likely a communication error.
		return -1;
	}


	sxv1_calibarte_rc(bsradio);
	sxv1_write_reg(bsradio, SXV1_REG_RSSITHRESH, 0xC4);
	//sxv1_write_reg(bsradio, SXV1_REG_RSSITHRESH, 0xE4);
//	sxv1_write_reg(bsradio, SXV1_REG_RSSITHRESH, 0xFF);

//	sxv1_write_reg(bsradio, SXV1_REG_RSSITHRESH, 0xE4);


//	/*
//	 The DAGC is enabled by setting RegTestDagc to 0x20 for low modulation index systems
//	 (i.e. when AfcLowBetaOn=1, refer to section 3.4.16), and 0x30 for other systems.
//	 It is recommended to always enable the DAGC.
//	 */

	// Try enabling it again. Is this the reason for the corrupted packets?
//	sxv1_write_reg(bsradio, SXV1_REG_AFCCTRL, 0x00);
	sxv1_write_reg(bsradio, SXV1_REG_TESTDAGC, 0x30);

	// Disable DAGC to enable RSSI again
//	sxv1_write_reg(bsradio, SXV1_REG_TESTDAGC, 0x00);

	sxv1_packet_config1_t config1;

	//0b00 = off 0b01 = nodeaddress, 0b10 = node or broadcast
	if (bsradio->rfconfig.node_id_enable && bsradio->rfconfig.broadcast_id_enable)
		config1.address_filtering = 0b10;
	else if (bsradio->rfconfig.node_id_enable && !bsradio->rfconfig.broadcast_id_enable)
		config1.address_filtering = 0b01;
	else
		config1.address_filtering = 0b00;


	config1.crc_auto_clear_off = 0;
	config1.dc_free = 0b00; // TODO might want to make this a confiugation option

	config1.packet_format = 1; // Variable Length

	switch (bsradio->rfconfig.crc) {
	case crc_disable:
		config1.crc_on = 0;
		break;
	case CCITT_16:
		config1.crc_on = 1;
		break;
	default:
		// not supported by hardware
		// if we were to add software support for it
		// we should put the chip to fixed length mode
		// such it doesn't strip off the crc
		config1.packet_format = 0; // fixed length
		return -1;
	}

	sxv1_write_reg(bsradio, SXV1_REG_PACKETCONFIG1, config1.as_uint8);

	sxv1_write_reg(bsradio, SXV1_REG_NODEADRS, bsradio->rfconfig.node_id);
	sxv1_write_reg(bsradio, SXV1_REG_BROADCASTADRS, bsradio->rfconfig.broadcast_id);


	sxv1_packet_config2_t config2;
	config2.aes_on = 0;
	config2.auto_rx_restart_on = 1;
//	config2.auto_rx_restart_on = 0; // manually restart rx
	// config2.inter_packet_rx_delay=0; //?? Verify this value
	config2.inter_packet_rx_delay = 1; //?? Verify this value
	config2.restart_rx = 0;
	sxv1_write_reg(bsradio, SXV1_REG_PACKETCONFIG2, config2.as_uint8);

	sxv1_data_modul_t data_modul;
	data_modul.data_mode = 0b00; // Packet mode

	switch (bsradio->rfconfig.modulation) {

	case modulation_ook:
		data_modul.modulation_type = 0b01; // OOK
		break;
	case modulation_2fsk:
		data_modul.modulation_type = 0b00; // FSK
		break;
	case modulation_none:
	case modulation_4fsk:
	case modulation_lora:
		return -1;
	}

	if (bsradio->rfconfig.modulation_shaping < 3)
		data_modul.modulation_shaping = 0b00; // Gaussian filter off
	else if (bsradio->rfconfig.modulation_shaping < 5)
		data_modul.modulation_shaping = 0b11; // Gaussian filter 0.3
	else if (bsradio->rfconfig.modulation_shaping < 10)
		data_modul.modulation_shaping = 0b10; // Gaussian filter 0.5
	else
		data_modul.modulation_shaping = 0b01; // Gaussian filter 1.0

	sxv1_write_reg(bsradio, SXV1_REG_DATAMODUL, data_modul.as_uint8);

	sxv1_write_reg(bsradio, SXV1_REG_PAYLOADLENGTH, 0x40); // Max size when receiving
	sxv1_write_reg(bsradio, SXV1_REG_FIFOTHRESH, 0x80); // Start sending when 1 byte is in fifo

	// Two SXV1 can communicate with RSSI timeout set to 0x10
	// But receiving an Si4432 requires a higher value.
	// I put it to 0x40 but it can probably be smaller, 0x20 works
	//sxv1_write_reg(SXV1_REG_RXTIMEOUT2, 0x10); // RSSI Timeout
	sxv1_write_reg(bsradio, SXV1_REG_RXTIMEOUT2, 0x40); // RSSI Timeout
//	sxv1_write_reg(bsradio, SXV1_REG_RXTIMEOUT2, 0x20); // RSSI Timeout

	sxv1_set_bitrate(bsradio, bsradio->rfconfig.birrate_bps);
	sxv1_set_fdev(bsradio, bsradio->rfconfig.freq_dev_hz);
	sxv1_set_bandwidth(bsradio, bsradio->rfconfig.bandwidth_hz);
	sxv1_set_frequency(bsradio, bsradio->rfconfig.frequency_kHz);
	sxv1_set_tx_power(bsradio, bsradio->rfconfig.tx_power_dBm);

	bsradio_set_network_id(bsradio, bsradio->rfconfig.network_id,
			bsradio->rfconfig.network_id_size);
	bsradio_set_node_id(bsradio, bsradio->rfconfig.node_id);

	return 0;
}

int sxv1_set_bitrate(struct bsradio_instance_t *bsradio, int bps) {

	int bitratereg = bsradio->hwconfig.xtal_freq / bps;
	sxv1_write_reg(bsradio, SXV1_REG_BITRATEMSB, (bitratereg & 0xFF00) >> 8);
	sxv1_write_reg(bsradio, SXV1_REG_BITRATELSB, bitratereg & 0xFF);
	return 0;
}

int sxv1_set_fdev(struct bsradio_instance_t *bsradio, int hz) {
	//int fdevregreg = hz / SXV1_FSTEP_HZ;
	int fdevregreg = (int) ((float) hz
			/ ((float) bsradio->hwconfig.xtal_freq / (float) (1 << 19)));
	sxv1_write_reg(bsradio, SXV1_REG_FDEVMSB, (fdevregreg & 0xFF00) >> 8);
	sxv1_write_reg(bsradio, SXV1_REG_FDEVLSB, fdevregreg & 0xFF);
	return 0;
}

int sxv1_set_bandwidth(struct bsradio_instance_t *bsradio, int hz) {
	// For OOK, the bandwidth is half of the FSK bandwidth, therefore
	// we double the requested bandwidth if the modulation is set to OOK
	if (bsradio->rfconfig.modulation == modulation_ook)
		hz *= 2;

	sxv1_rxbw_t rxbw;
	int i;
	for (i = 0; m_rxbw_entries[i].bandwidth; i++)
		if (m_rxbw_entries[i].bandwidth > hz)
			break;
	if (m_rxbw_entries[i].bandwidth) {
		rxbw = m_rxbw_entries[i].rxbw;
		sxv1_write_reg(bsradio, SXV1_REG_RXBW, rxbw.as_uint8);
		return 0;
	}
	return -1;
}

int sxv1_send_packet(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_packet) {
	int status;
	status = sxv1_rx_restart(bsradio);
	if (status) {
		puts("[" "]\t" "sxv1_rx_restart failed");
		return status;
	}
	status = sxv1_set_mode_internal(bsradio, sxv1_mode_standby);
	if (status) {
		puts("[" "]\t" "sxv1_set_mode_internal failed");
		return status;
	}
	status = sxv1_write_fifo(bsradio, p_packet);
	if (status) {
		puts("[" "]\t" "sxv1_write_fifo failed");
		return status;
	}
	status = sxv1_set_mode_internal(bsradio, sxv1_mode_tx);
	if (status) {
		puts("[" "]\t" "sxv1_set_mode_internal failed");
		return status;
	}

	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	sxv1_irq_flags_2_t irq_flags_2 = { 0 };


	while (!irq_flags_2.packet_send) {
		// TODO: ADD TIMEOUT
		status = sxv1_read_reg(bsradio, SXV1_REG_IRQFLAGS2,
				&irq_flags_2.as_uint8);
		if (status)
			return status;
		status = sxv1_read_reg(bsradio, SXV1_REG_IRQFLAGS1,
				&irq_flags_1.as_uint8);
		if (status)
			return status;
	}
	status = sxv1_set_mode_internal(bsradio, sxv1_mode_standby);
	if (status)
		return status;
	return 0;
}

int sxv1_recv_packet(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_packet) {
	int status;
	sxv1_set_mode_internal(bsradio, sxv1_mode_rx);
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	sxv1_irq_flags_2_t irq_flags_2 = { 0 };
	status = sxv1_read_reg(bsradio, SXV1_REG_IRQFLAGS2, &irq_flags_2.as_uint8);
	status = sxv1_read_reg(bsradio, SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
	p_packet->length = 0;

	if (status)
		return status;

	static uint8_t rssi_raw;
	sxv1_rssiconfig_t rssiconfig = {};
	static bool rssistarted = false;


//	if (!rssistarted) {
//		rssiconfig.start = 1;
//		sxv1_write_reg(bsradio, SXV1_REG_RSSICONFIG, rssiconfig.as_uint8);
		rssistarted = true;
//	}

	sxv1_read_reg(bsradio, SXV1_REG_RSSICONFIG, &rssiconfig.as_uint8);
	if (rssiconfig.done) {
		sxv1_read_reg(bsradio, SXV1_REG_RSSIVALUE, &rssi_raw);
//		puts("RSSI ready");
		rssistarted = false;
	}


	if (irq_flags_2.payload_ready) {
		puts("Payload Ready");



		// there is data, but how much to read
		// Is there a FIFO LEVEL register???
		uint8_t size = BSRADIO_MAX_PACKET_LEN;
		sxv1_read_fifo(bsradio, p_packet);
		if (size < 0) {
			status = -1;
		} else {
			status = 0;
		}

		int8_t rssi = (-rssi_raw) / 2;
		printf("RSSI val %d\n", rssi);
		p_packet->rssi = (-rssi_raw) / 2;

		rssi_raw = 0;

		// only restart after receiving packet
//		sxv1_rx_restart(bsradio);
		// restart not reliable? switch to standy instead
		// so next interation we go back to rx mode?
		sxv1_set_mode_internal(bsradio, sxv1_mode_standby);



	} else {
		status = -1;
	}

//	if (irq_flags_1.timeout) {
//		// What does timeout mean?
//		sxv1_restart();
//	}

	bshal_delay_ms(1);
	return status;
}

