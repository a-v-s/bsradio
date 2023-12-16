#include "sxv1.h"
#include "radio.h"

#include <stdbool.h>

#include <bshal_spim.h>
#include <bshal_gpio.h>

bool g_sxv1_interrupt_flag;
extern bshal_spim_instance_t spi_radio_config;

// Looks like a pettern, we can optimise this
// so we don't need to keep a table
const static sxv1_rxbw_entry_t m_rxbw_entries[] = {
		{ 2600, { 7, 0b10, 0b010 } }, { 3100, { 7, 0b01, 0b010 } }, { 3900, { 7,
				0b00, 0b010 } },

		{ 5200, { 6, 0b10, 0b010 } }, { 6300, { 6, 0b01, 0b010 } }, { 7800, { 6,
				0b00, 0b010 } },

		{ 10400, { 5, 0b10, 0b010 } }, { 12500, { 5, 0b01, 0b010 } }, { 15600, {
				5, 0b00, 0b010 } },

		{ 20800, { 4, 0b10, 0b010 } }, { 25000, { 4, 0b01, 0b010 } }, { 31300, {
				4, 0b00, 0b010 } },

		{ 41700, { 3, 0b10, 0b010 } }, { 50000, { 3, 0b01, 0b010 } }, { 62500, {
				3, 0b00, 0b010 } },

		{ 83300, { 2, 0b10, 0b010 } }, { 100000, { 2, 0b01, 0b010 } }, { 125000,
				{ 2, 0b00, 0b010 } },

		{ 116700, { 1, 0b10, 0b010 } }, { 200000, { 1, 0b01, 0b010 } }, { 250000,
				{ 1, 0b00, 0b010 } },

		{ 333300, { 0, 0b10, 0b010 } }, { 400000, { 0, 0b01, 0b010 } }, {
				500000, { 0, 0b00, 0b010 } },

		// Termination
		{ 0, { 0, 0, 0 } },

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

int sxv1_write_fifo(bsradio_instance_t *bsradio,void *data, uint8_t size) {
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

int sxv1_read_fifo(bsradio_instance_t *bsradio,void *data, uint8_t *size) {
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

int sxv1_set_frequency(bsradio_instance_t *bsradio,int kHz) {

	//int regval = (float) (1000*kHz) / (float) SXV1_FSTEP_HZ;

	// Without the need of float
	kHz += bsradio->hwconfig.tune;
	int regval = ((uint64_t)(1000 * kHz) << 19) / SXV1_XTAL_FREQ;

	int status;

	/*
	 > The Frf setting is split across three bytes. A change in the center frequency is only
	 > taken into account when the Least Significant Byte FrfLsb in RegFrfLsb is written.

	 This suggest LSB should be written last
	 */

	status = sxv1_write_reg(bsradio,SXV1_REG_FRFMSB, (regval & 0xFF0000) >> 16);
	if (status)
		return status;
	status = sxv1_write_reg(bsradio,SXV1_REG_FRFMID, (regval & 0xFF00) >> 8);
	if (status)
		return status;
	status = sxv1_write_reg(bsradio,SXV1_REG_FRFLSB, regval & 0xFF);
	if (status)
		return status;
	return 0;
}

int sxv1_set_mode(bsradio_instance_t *bsradio,sxv1_mode_t mode) {
	sxv1_val_opmode_t val;
	int status = status = sxv1_read_reg(bsradio,SXV1_REG_OPMODE, &val.as_uint8);
	if (status)
		return status;
	if (val.mode == mode)
		return 0;
	val.mode = mode;
	status = sxv1_write_reg(bsradio,SXV1_REG_OPMODE, val.as_uint8);
	if (status)
		return status;
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	int timeout_us = get_time_us() + SXV1_MODESWITCH_TIMEOUT_US;
	while (!irq_flags_1.mode_ready) {
		status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
		if (timeout_us < get_time_us()) {
			// timeout
			return -1;
		}
	}

	return 0;
}

int sxv1_calibarte_rc(bsradio_instance_t *bsradio) {
	int result;
	result = sxv1_set_mode(bsradio, sxv1_mode_standby);
	if (result)
		return result;
	result = sxv1_write_reg(bsradio,SXV1_REG_OSC1, 0x80);
	if (result)
		return result;
	uint8_t regval = 0x00;
	while (!regval) {
		// TODO Add Timeout
		result = sxv1_read_reg(bsradio,SXV1_REG_OSC1, &regval);
		if (result)
			return result;
	}
	return 0;
}

int sxv1_set_sync_word32(bsradio_instance_t *bsradio,uint32_t sync_word) {
	//debug_println("Setting SYNC word to %08X", sync_word);
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 3; // size = sync_size + 1, thus 4
	config.sync_tol = 0;
//	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);
//	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE1, sync_word >> 24);
//	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE2, sync_word >> 16);
//	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE3, sync_word >> 8);
//	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE4, sync_word );

	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE3, (sync_word & 0xFF0000) >> 16);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE4, (sync_word & 0xFF000000) >> 24);
	return 0;
}

int sxv1_set_sync_word_24(bsradio_instance_t *bsradio,uint32_t sync_word) {
	//debug_println("Setting SYNC word to %08X", sync_word);
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 2; // size = sync_size + 1, thus 3
	config.sync_tol = 0;
	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE1, sync_word >> 24);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE2, sync_word >> 16);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE3, sync_word >> 8);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE4, sync_word );

	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE3, (sync_word & 0xFF0000) >> 16);
//	sxv1_write_reg(SXV1_REG_SYNCVALUE4, (sync_word & 0xFF000000) >> 24);
	return 0;
}

int sxv1_set_sync_word_16(bsradio_instance_t *bsradio,uint16_t sync_word) {
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 1; // size = sync_size + 1, thus 2
	config.sync_tol = 0;
	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE2, (sync_word & 0xFF00) >> 8);
	return 0;
}

int sxv1_set_sync_word_8(bsradio_instance_t *bsradio,uint8_t sync_word) {
	sxv1_sync_config_t config;
	config.fifo_fill_condition = 0;
	config.sync_on = 1;
	config.sync_size = 0; // size = sync_size + 1, thus 1
	config.sync_tol = 0;
	sxv1_write_reg(bsradio,SXV1_REG_SYNCCONFIG, config.as_uint8);
	sxv1_write_reg(bsradio,SXV1_REG_SYNCVALUE1, sync_word & 0xFF);
	return 0;
}

int sxv1_rx_restart(bsradio_instance_t *bsradio) {
	// Restarting RX before changing to TX mode is something done
	// in the LowPowerLab library, with a comment it prevents a deadlock

	sxv1_packet_config2_t config2;
	sxv1_read_reg(bsradio,SXV1_REG_PACKETCONFIG2, &config2.as_uint8);
	config2.restart_rx = 1;
	sxv1_write_reg(bsradio,SXV1_REG_PACKETCONFIG2, config2.as_uint8);
	return 0;
}

int sxv1_set_tx_power(bsradio_instance_t *bsradio,int tx_power) {
	// For SXV1HW Module
	// As the module states up to 20 dBm
	// it should output at PA_BOOST

	sxv1_val_palevel_t pa_level;
	if (tx_power <= -3) {

		// out of range
		return -1;
	} else if ((tx_power + 18) <= 0b11111) {
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
	} else if ((tx_power + 11) <= 0b11111) {
		// PA1 + PA2 + High Power

		pa_level.pa0_on = 0;
		pa_level.pa1_on = 1;
		pa_level.pa2_on = 1;
		pa_level.output_power = tx_power + 11;
	} else {
		// out of range
		return -1;
	}
	sxv1_write_reg(bsradio,SXV1_REG_PALEVEL, pa_level.as_uint8);
	return 0;
}

int sxv1_send_response(bsradio_instance_t *bsradio,sxv1_air_packet_t *p_packet) {
	// Should we care about the data format by the library?
	int status;

	sxv1_rx_restart(bsradio);

	sxv1_set_mode(bsradio,sxv1_mode_standby);
	sxv1_write_fifo(bsradio,p_packet, p_packet->header.size);
	sxv1_set_mode(bsradio,sxv1_mode_tx);
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	sxv1_irq_flags_2_t irq_flags_2 = { 0 };

	int begin = get_time_us();
	while (!irq_flags_2.packet_send) {
		// TODO: ADD TIMEOUT
		status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS2, &irq_flags_2.as_uint8);
		status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
	}
	// Determining what the timeout should be
	// This will depend on packet size...
	debug_printf("Sending response took %d us\n", get_time_us() - begin);

	// Switch to RX mode, wait for answer
	sxv1_set_mode(bsradio,sxv1_mode_rx);
	sxv1_rx_restart(bsradio);

	return 0;
}
int sxv1_send_request(bsradio_instance_t *bsradio,sxv1_air_packet_t *p_request,
		sxv1_air_packet_t *p_response) {
	// Should we care about the data format by the library?
	int status;

	sxv1_rx_restart(bsradio);

	sxv1_set_mode(bsradio,sxv1_mode_standby);
	sxv1_write_fifo(bsradio,p_request, p_request->header.size);
	sxv1_set_mode(bsradio,sxv1_mode_tx);
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	sxv1_irq_flags_2_t irq_flags_2 = { 0 };

	int begin = get_time_us();
	while (!irq_flags_2.packet_send) {
		// TODO: ADD TIMEOUT
		status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS2, &irq_flags_2.as_uint8);
		status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
	}
	sxv1_set_mode(bsradio,sxv1_mode_standby);
	// Determining what the timeout should be
	// This will depend on packet size...

//	begin = get_time_us();
//
//	// Switch to RX mode, wait for answer
//	sxv1_set_mode(sxv1_mode_rx);
//	sxv1_restart();
//	irq_flags_2.as_uint8 = 0;
//	uint32_t response_timeout_ms = get_time_ms() + SXV1_TXRX_TIMEOUT_MS;
//	while (!irq_flags_2.payload_ready) {
//		if (get_time_ms() > response_timeout_ms) {
//			// timeout;
//
//			sxv1_restart();
//			return -1;
//		}
//		status = sxv1_read_reg(SXV1_REG_IRQFLAGS2, &irq_flags_2.as_uint8);
//		status = sxv1_read_reg(SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
//	}
//
//	sxv1_read_fifo(p_response, sizeof(sxv1_air_packet_t));
//
//	uint8_t rssi_raw;
//	sxv1_read_reg(SXV1_REG_RSSIVALUE, rssi_raw);
//	int8_t rssi = (-rssi_raw) / 2;
//	printf("RSSI val %d\n", rssi);

//	sxv1_restart();
	return 0;
}

int sxv1_receive_request(bsradio_instance_t *bsradio,sxv1_air_packet_t *p_packet) {
	int status;
	sxv1_irq_flags_1_t irq_flags_1 = { 0 };
	sxv1_irq_flags_2_t irq_flags_2 = { 0 };
	status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS2, &irq_flags_2.as_uint8);
	status = sxv1_read_reg(bsradio,SXV1_REG_IRQFLAGS1, &irq_flags_1.as_uint8);
	p_packet->header.size = 0;

	if (status)
		return status;

	static uint8_t rssi_raw;
	if (irq_flags_1.sync_address_match) {
		sxv1_read_reg(bsradio,SXV1_REG_RSSIVALUE, &rssi_raw);
	}

	if (irq_flags_2.payload_ready) {
		// there is data, but how much to read
		// Is there a FIFO LEVEL register???
		uint8_t size = sizeof(sxv1_air_packet_t);
		sxv1_read_fifo(bsradio,p_packet, &size);




		int8_t rssi = (-rssi_raw) / 2;
		printf("RSSI val %d\n", rssi);

		char buff[32] = { 0 };
		sprintf(buff, "RSSI %d\n", rssi);
		print(buff, 0);

		// only restart after receiving packet
		sxv1_rx_restart(bsradio);

		status = 0;
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

void sxv1_irq_handler(void) {
	g_sxv1_interrupt_flag = true;
}

void sxv1_configure_packet(bsradio_instance_t *bsradio) {
	sxv1_packet_config1_t config1;
	config1.address_filtering = 0b00;
	config1.crc_auto_clear_off = 0;

	config1.dc_free = 0b00;


//	config1.crc_on = 1;
	config1.packet_format = 1; // Variable Length


	config1.crc_on = 0; // for  inter-module testing
//	config1.packet_format = 0; // Fixed Length for intermodule testing



	sxv1_write_reg(bsradio,SXV1_REG_PACKETCONFIG1, config1.as_uint8);

	sxv1_packet_config2_t config2;
	config2.aes_on = 0;
	//config2.auto_rx_restart_on = 1;
	config2.auto_rx_restart_on = 0; // manually restart rx
	// config2.inter_packet_rx_delay=0; //?? Verify this value
	config2.inter_packet_rx_delay = 1; //?? Verify this value
	config2.restart_rx = 0;
	sxv1_write_reg(bsradio,SXV1_REG_PACKETCONFIG2, config2.as_uint8);

	sxv1_data_modul_t data_modul;
	data_modul.data_mode = 0b00; // Packet mode
	data_modul.modulation_type = 0b00; // FSK

	//data_modul.modulation_shaping = 0b00; // Gaussian filter off
	//data_modul.modulation_shaping = 0b01; // Gaussian filter 1.0
	data_modul.modulation_shaping = 0b10; // Gaussian filter 0.5
	//data_modul.modulation_shaping = 0b11; // Gaussian filter 0.3

	sxv1_write_reg(bsradio,SXV1_REG_DATAMODUL, data_modul.as_uint8);

	sxv1_write_reg(bsradio,SXV1_REG_PAYLOADLENGTH, 0x40); // Max size when receiving
	sxv1_write_reg(bsradio,SXV1_REG_FIFOTHRESH, 0x80); // Start sending when 1 byte is in fifo

	// Two SXV1 can communicate with RSSI timeout set to 0x10
	// But receiving an Si4432 requires a higher value.
	// I put it to 0x40 but it can probably be smaller, 0x20 works
	//sxv1_write_reg(SXV1_REG_RXTIMEOUT2, 0x10); // RSSI Timeout
//	sxv1_write_reg(bsradio,SXV1_REG_RXTIMEOUT2, 0x40); // RSSI Timeout
	sxv1_write_reg(bsradio,SXV1_REG_RXTIMEOUT2, 0x20); // RSSI Timeout

//	sxv1_write_reg(bsradio,SXV1_REG_TESTDAGC, 0x00); // Disable DAGC



}

int sxv1_set_bitrate(bsradio_instance_t *bsradio,int bps) {
	int bitratereg = SXV1_XTAL_FREQ / bps;
	sxv1_write_reg(bsradio,SXV1_REG_BITRATEMSB, (bitratereg & 0xFF00) >> 8);
	sxv1_write_reg(bsradio,SXV1_REG_BITRATELSB, bitratereg & 0xFF);
	return 0;
}

int sxv1_set_fdev(bsradio_instance_t *bsradio,int hz) {
	int fdevregreg = hz / SXV1_FSTEP_HZ;
	sxv1_write_reg(bsradio,SXV1_REG_FDEVMSB, (fdevregreg & 0xFF00) >> 8);
	sxv1_write_reg(bsradio,SXV1_REG_FDEVLSB, fdevregreg & 0xFF);
	return 0;
}

int sxv1_set_bandwidth(bsradio_instance_t *bsradio,int hz) {
	sxv1_rxbw_t rxbw;
	int i;
	for (i = 0; m_rxbw_entries[i].bandwidth; i++)
		if (m_rxbw_entries[i].bandwidth > hz)
			break;
	if (m_rxbw_entries[i].bandwidth) {
		rxbw = m_rxbw_entries[i].rxbw;
		sxv1_write_reg(bsradio,SXV1_REG_RXBW, rxbw.as_uint8);
		return 0;
	}
	return -1;
}

