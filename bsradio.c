/*
 * bsradio.c
 *
 *  Created on: 16 dec. 2023
 *      Author: andre
 */

#include "bsradio.h"

#include "sxv1.h"

int bsradio_set_frequency(struct bsradio_instance_t *bsradio, int kHz) {
	return bsradio->driver.set_frequency(bsradio, kHz);
}
int bsradio_set_tx_power(struct bsradio_instance_t *bsradio, int tx_power_dBm) {
	return bsradio->driver.set_tx_power(bsradio, tx_power_dBm);
}
int bsradio_set_bitrate(struct bsradio_instance_t *bsradio, int bps) {
	return bsradio->driver.set_bitrate(bsradio, bps);
}
int bsradio_set_fdev(struct bsradio_instance_t *bsradio, int hz) {
	return bsradio->driver.set_fdev(bsradio, hz);
}
int bsradio_set_bandwidth(struct bsradio_instance_t *bsradio, int hz) {
	return bsradio->driver.set_bandwidth(bsradio, hz);
}
int bsradio_init(struct bsradio_instance_t *bsradio) {
	return bsradio->driver.init(bsradio);
}
int bsradio_set_network_id(struct bsradio_instance_t *bsradio, char *sync_word,
		size_t size) {
	return bsradio->driver.set_network_id(bsradio, sync_word, size);
}
int bsradio_set_mode(struct bsradio_instance_t *bsradio, bsradio_mode_t mode) {
	return bsradio->driver.set_mode(bsradio, mode);
}
int bsradio_recv_packet(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_packet) {
	return bsradio->driver.recv_packet(bsradio, p_packet);
}
int bsradio_send_packet(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_packet) {
	return bsradio->driver.send_packet(bsradio, p_packet);
}

int bsradio_send_request(struct bsradio_instance_t *bsradio,
		bsradio_packet_t *p_request, bsradio_packet_t *p_response) {
	static uint8_t seq_counters[255]={};
	int result;
	p_request->ack_request = 1;
	p_request->ack_response = 0;
	p_request->retry_cnt=0;
	p_request->seq_nr = seq_counters[p_request->to]++;
	while ( p_request->retry_cnt < 3) {
		printf("Sending request to %02X, seq %3d, retry %d\n",
				p_request->to,
				p_request->seq_nr,
				p_request->retry_cnt);
		uint32_t begin = get_time_ms();
		result = bsradio_send_packet(bsradio, p_request);
		if (result) {
			printf("Sending failed with error %d\n",result);
			return result;
		}
		printf("Transmission took %d ms\n", get_time_ms() - begin);
		result = 1;
		uint32_t timeout = get_time_ms() + 100;
		while (result) {

			result = bsradio_recv_packet(bsradio, p_response);
			if (get_time_ms() > timeout) {
				puts("Timeout");
				break;
			}

		}
		if (result) {
			p_request->retry_cnt++;
			continue;
		}
		printf("Received response from %02X, seq %3d, retry\n",
				p_response->from,
				p_response->seq_nr,
				p_response->retry_cnt);

	}
	return result;
}
