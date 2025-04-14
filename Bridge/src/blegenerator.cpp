/*
 *  Copyright 2017 by Francesco Gringoli <francesco.gringoli@unibs.it>
 *  Copyright 2017 by Jiang Wei <jiangwei@jiangwei.org>
 *  Copyright 2015 by Xianjun Jiao (putaoshu@gmail.com)
 *  Copyright 2013 Florian Echtler <floe@butterbrot.org>
 *
 * This file is part of some open source application.
 *
 * Some open source application is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * Some open source application is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the application. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <iostream>
#include <complex>
#define _USE_MATH_DEFINES
#include <math.h>
#include <inttypes.h>
#include <stdint.h>
#include <vector>
#include <functional>
#include <arpa/inet.h>


#define MAX_NUM_PHY_SAMPLE 3040 //1520
#define MAX_NUM_CHAR_CMD (256)
#define MAX_NUM_PHY_BYTE (94) // 47
#define SAMPLE_PER_SYMBOL 4 // 
#define LEN_GAUSS_FILTER (4) // pre 2, post 2
#define MAX_LE_SYMBOLS 64

#define LE_ADV_AA 0x8E89BED6

class BLESDR {
public:
        BLESDR();
        ~BLESDR();
        std::vector<float> sample_for_raw_packet(size_t chan, int counter, uint8_t packet[],int len_packet,int flag);

private:
        std::vector<float> iqsamples;
        size_t byte_to_bits(uint8_t* byte, size_t len, char* bits);
        float* generate_gaussian_taps(unsigned samples_per_sym, unsigned L, double bt);
        void btle_calc_crc(void* src, uint8_t len, uint8_t* dst,int flag);
        void btle_whiten(uint8_t chan, uint8_t* buf, uint8_t len);

        int gen_sample_from_phy_bit(char *bit, float *sample, int num_bit);
        float tmp_phy_bit_over_sampling[MAX_NUM_PHY_SAMPLE + 2 * LEN_GAUSS_FILTER*SAMPLE_PER_SYMBOL];
        float tmp_phy_bit_over_sampling1[MAX_NUM_PHY_SAMPLE];
        float * gauss_coef;

        uint8_t chan;
        int srate;

	void dump_btle_packet(uint8_t *packet, int length);

};

void BLESDR::dump_btle_packet(uint8_t *pp, int length)
{
        for(int kk = 0; kk < length; kk ++) {
		printf("%02X ", pp[kk]);
	}
	printf("\n");
}


BLESDR::BLESDR() :
	chan(37),
	srate(2)
{
	gauss_coef = generate_gaussian_taps(SAMPLE_PER_SYMBOL, LEN_GAUSS_FILTER, 0.5);

}

BLESDR::~BLESDR() {
	delete gauss_coef;
}

size_t BLESDR::byte_to_bits(uint8_t* byte, size_t len, char* bits) {

	for (int j = 0; j < len; j++) {
		for (int i = 0; i < 8; i++) {
			// Mask each bit in the byte and store it
			bits[j * 8 + i] = (byte[j] >> i) & 1;
		}
	}
	return len * 8;
}

void BLESDR::btle_calc_crc(void* src, uint8_t len, uint8_t* dst,int flag) {

	uint8_t* buf = (uint8_t*)src;
	dst[0] = 0xAA;
	dst[1] = 0xAA;
	dst[2] = 0xAA;
	if(flag==1){
	dst[0] = 0x03;
	dst[1] = 0xEB;
	dst[2] = 0x68;

	}

	while (len--) {

		uint8_t d = *(buf++);

		for (uint8_t i = 1; i; i <<= 1, d >>= 1) {
			uint8_t t = dst[0] & 0x01;         dst[0] >>= 1;
			if (dst[1] & 0x01) dst[0] |= 0x80; dst[1] >>= 1;
			if (dst[2] & 0x01) dst[1] |= 0x80; dst[2] >>= 1;

			if (t != (d & 1)) {
				dst[2] ^= 0xDA;
				dst[1] ^= 0x60;
			}
		}
	}
}

void BLESDR::btle_whiten(uint8_t chan, uint8_t* buf, uint8_t len)
{
	uint8_t lfsr = chan | 0x40;
	printf("channel: %d, length:%d\n",chan,len);
	for(int i=0;i<len;i++){
		printf("%02X",buf[i]);
	}
	printf("\nafter whitening:\n");
	while (len--) {
		uint8_t res = 0;
		for (uint8_t i = 1; i; i <<= 1) {
			if (lfsr & 0x01) {
				lfsr ^= 0x88;
				res |= i;
			}
			lfsr >>= 1;
		}
		*(buf++) ^= res;
		printf("%02X",*(buf-1));
	}
	printf("\n= = = = = = = = = =\n");
}

std::vector<float> BLESDR::sample_for_raw_packet(size_t chan, int counter,uint8_t new_packet[],int len_packet,int flag)
{
	uint8_t packet[len_packet];
	for(int i=0;i<len_packet;++i){
		packet[i]=new_packet[i];
	}
	uint8_t preamble = 0xAA;
	uint32_t access_address =  0x8E89BED6;//1
	if(flag==1){
		access_address=0x21C7CFCD; 
	}

	btle_calc_crc(packet, sizeof(packet) - 3, packet + sizeof(packet) - 3,flag);

	btle_whiten(chan, packet, sizeof(packet));

	size_t numbits = (sizeof(packet) + 5) * 8;
	int offset = 0;
	char *bits = new char[numbits];
	iqsamples.resize((numbits * SAMPLE_PER_SYMBOL + (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL)) * 2);
	offset = byte_to_bits(&preamble, 1, bits);
	offset += byte_to_bits((uint8_t*) &access_address, 4, bits + offset);
	offset += byte_to_bits(packet, sizeof(packet), bits + offset);

	if(flag==4){
		    int  cte_tail_len=0;
			uint8_t cte_tail[cte_tail_len];
			for(int i=0;i<cte_tail_len;++i){
				cte_tail[i]=0xFF;
			}
			offset += byte_to_bits(cte_tail, cte_tail_len, bits + offset);
			numbits=numbits+8*cte_tail_len;
			iqsamples.resize((numbits * SAMPLE_PER_SYMBOL + (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL)) * 2);
	}

	int num_phy_sample = gen_sample_from_phy_bit(bits, iqsamples.data(), numbits);
	delete bits;
	return iqsamples;
}

int BLESDR::gen_sample_from_phy_bit(char *bit, float *sample, int num_bit)
{
	int num_sample = (num_bit * SAMPLE_PER_SYMBOL) + (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL);
	int i, j;
	for (i = 0; i < (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL - 1); i++) {
		tmp_phy_bit_over_sampling[i] = 0.0;
	}
	for (i = (1 * LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL - 1 + num_bit * SAMPLE_PER_SYMBOL);
	     i < (2 * LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL - 2 + num_bit * SAMPLE_PER_SYMBOL);
	     i ++) {
		tmp_phy_bit_over_sampling[i] = 0.0;
	}
	for (i = 0; i < (num_bit * SAMPLE_PER_SYMBOL); i++) {
		if (i % SAMPLE_PER_SYMBOL == 0) {
			tmp_phy_bit_over_sampling[i + (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL - 1)] =
				(float)(bit[i / SAMPLE_PER_SYMBOL]) * 2.0 - 1.0;
		}
		else {
			tmp_phy_bit_over_sampling[i + (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL - 1)] = 0.0;
		}
	}
	int len_conv_result = num_sample - 1;
	for (i = 0; i < len_conv_result; i++) {
		float acc = 0;
		for (j = 0; j < (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL); j++) {
			acc = acc + gauss_coef[(LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL) - j - 1] *
				    tmp_phy_bit_over_sampling[i + j];
		}
		tmp_phy_bit_over_sampling1[i] = acc;
	}
	float tmp = 0;
	sample[0] = cosf(tmp);
	sample[1] = sinf(tmp);
	for (i = 1; i < num_sample; i++) {
		tmp = tmp + (M_PI * 0.5) * tmp_phy_bit_over_sampling1[i - 1] / ((float)SAMPLE_PER_SYMBOL);
		sample[i * 2 + 0] = cos(tmp);
		sample[i * 2 + 1] = sin(tmp);
	}
	return(num_sample);
}

float* BLESDR::generate_gaussian_taps(unsigned samples_per_sym, unsigned L, double bt)
{
	float* taps = new float[L*samples_per_sym];
	double scale = 0;
	double dt = 1.0 / samples_per_sym;
	double s = 1.0 / (sqrt(log(2.0)) / (2 * M_PI*bt));
	double t0 = -0.5 * L*samples_per_sym;
	double ts;
	for (unsigned i = 0; i < L*samples_per_sym; i++) {
		t0++;
		ts = s*dt*t0;
		taps[i] = exp(-0.5*ts*ts);
		scale += taps[i];
	}
	for (unsigned i = 0; i < L*samples_per_sym; i++){
		taps[i] = taps[i] / scale;
	}


	float temp[16]={7.561773e-09, 1.197935e-06, 8.050684e-05, 2.326833e-03, 2.959908e-02, 1.727474e-01, 4.999195e-01, 8.249246e-01, 9.408018e-01, 8.249246e-01, 4.999195e-01, 1.727474e-01, 2.959908e-02, 2.326833e-03, 8.050684e-05, 1.197935e-06};

	for(int i=0;i<16;i++){
		taps[i]=temp[i];
	}

	return taps;
}

std::vector<int8_t> generatesamples(int tx_chan, int counter, int padbytes, uint8_t new_packet[],int len_packet,int flag)
{
	// printf("sizeof(new_packet)->%d",len_packet);
	BLESDR ble;
	std::vector<float> samples = ble.sample_for_raw_packet(tx_chan, counter, new_packet,len_packet,flag);
	std::vector<int8_t> samples8bit;
        int samplen = samples.size();
	samples8bit.resize(samplen + padbytes);

	for(int kk = 0; kk < samples8bit.size(); kk ++) {
		if(kk < samplen) {
                	float sample = *(samples.data() + kk);
                	sample = sample * 128;
                	if(sample > 127) sample = 127;
                	if(sample < -128) sample = -128;
                	int8_t val = (int8_t) sample;
			samples8bit.data()[kk] = val;
		} else {
			samples8bit.data()[kk] = 0;
		}
	}
	return samples8bit;
}
