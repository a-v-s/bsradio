/*
 * bsradio.c
 *
 *  Created on: 16 dec. 2023
 *      Author: andre
 */


#include "bsradio.h"

#include "sxv1.h"


int test(void) {
	bsradio_instance_t radio;
	/*
	int (*set_frequency)(bsradio_instance_t *bsradio,int kHz) ;
	int (*set_tx_power)(bsradio_instance_t *bsradio,int tx_power);
	int (*set_bitrate)(bsradio_instance_t *bsradio,int bps) ;
	int (*set_fdev)(bsradio_instance_t *bsradio,int hz) ;
	int (*set_bandwidth)(bsradio_instance_t *bsradio,int hz) ;
	int (*configure_packet)(bsradio_instance_t *bsradio) ;
	int (*set_network_id)(bsradio_instance_t *bsradio,char *sync_word, size_t size) ;
	int (*set_mode)(bsradio_instance_t *bsradio,bsradio_mode_t mode) ;
	int (*recv_packet)(bsradio_instance_t *bsradio ,bsradio_packet_t *p_packet) ;
	int (*send_packet)(bsradio_instance_t *bsradio, bsradio_packet_t *p_packet) ;
	 */
	radio.driver.set_frequency = sxv1_set_frequency;
	radio.driver.set_tx_power = sxv1_set_tx_power;
	radio.driver.set_bitrate = sxv1_set_bitrate;
	radio.driver.set_fdev = sxv1_set_fdev;
	radio.driver.set_bandwidth = sxv1_set_bandwidth;
	radio.driver.init = sxv1_init;
	radio.driver.set_network_id = sxv1_set_network_id;
	radio.driver.set_mode = sxv1_set_mode;
	radio.driver.recv_packet = sxv1_recv_packet;
	radio.driver.send_packet = sxv1_send_packet;
}

