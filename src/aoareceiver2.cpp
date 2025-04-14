/* 
 * Copyright 2019 Marco Cominelli
 * Copyright 2019 Francesco Gringoli
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <csignal>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <chrono>
#include <complex>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include "blegenerator2.hpp"

#define XOR(x,y) ((x && !y) || (!x && y))

namespace po = boost::program_options;

volatile static bool stop_signal = false;
void sigint_handler(int) {stop_signal = true;}

volatile static int counter = 0;

pthread_spinlock_t lock[2];
volatile size_t bankA_nsamps, bankB_nsamps;

struct proc_pars_t {
    std::vector<std::complex<float> *> bankA_ptrs;
    std::vector<std::complex<float> *> bankB_ptrs;
    size_t samps_per_buff;
    int channel_idx;
};

typedef struct MyData {
    size_t sn;
    float aoa;
    float amplitude[2];
} mydata_t;

typedef struct Command {
    int type;
    int value;
} cmd_t;

std::chrono::high_resolution_clock::time_point startglobal;
std::chrono::high_resolution_clock::time_point endglobal;
int globalVariable=1;
extern std::mutex mtx;
volatile int do_print = 0;
bool do_hop = false;
bool change_gain = false;
double gain, freq;
unsigned int freq_inc = 0;

void* keyboard_routine(void* pars)
{
    while (stop_signal == false) {
        int c = fgetc(stdin);
        if (c == 'p') {
            do_print = 1;
        } else if (c == 'g') {
            change_gain = true;
        } else if (c == 'h') {
            do_hop = true;
        } else if (c == 'r') {
            sleep(4);
            printf("\a");
            for (int kk = 0; kk < 40; kk++) {
                do_hop = true;
                sleep(1);
                do_print = 30;
                while(do_print > 0) usleep(5000);
            }
            printf("\a");
        }
        usleep(1000);
    }

    return NULL;
}


inline void dewhiten(uint8_t *pdu, unsigned int length,int channel_idx)
{
    uint8_t lfsr = 0x40 | (0x3f & channel_idx);
    for (unsigned int byte = 0; byte < length; byte++) {
        for (int b = 0x01; b <= 0x80; b <<=1) {
            if (lfsr & 0x01) {
                pdu[byte] ^= b;
                lfsr ^= 0x88;
            }
            lfsr >>= 1;
        }
    }
}


uint32_t compute_crc(uint8_t *pdu, int len, uint32_t init_val)
{
    uint32_t crc = init_val;
    while (len--) {
        for (int i = 0x01; i <= 0x80; i <<= 1) {
            uint32_t crc_bit_is_one = crc & 0x00000001;
            uint32_t pdu_bit_is_one = *pdu & i;
            if ( XOR(crc_bit_is_one,pdu_bit_is_one) ) {
                crc ^= 0x01B4C000;
            }
            crc >>= 1;
        }
        pdu++;
    }
    return crc;
}

int check_init_address(char init_address[],uint8_t target_address[]){
    uint8_t tmp[6];
    int flag=1;
    for(int i=0;i<6;i++){
        sscanf(init_address+(i*2),"%2hhX",tmp+i);
        if(target_address[i+2]!=tmp[i]){
            flag=0;
            break;
        }
    }

    return flag;
}

void *process_routine(void *pars)
{
    struct proc_pars_t *procpars = (struct proc_pars_t *) pars;
    int channel_idx=procpars->channel_idx;
    size_t kk = 0, select_bank;
    size_t samps_per_buff = procpars->samps_per_buff;
    size_t circbuf_size = 2 * samps_per_buff;
    std::vector<std::complex<float> *> curr_bank;
    float *phasebuf0 = (float *) malloc(circbuf_size * sizeof(float));
    float *phasebuf1 = (float *) malloc(circbuf_size * sizeof(float));
    uint8_t *binbuf0 = (uint8_t *) malloc(circbuf_size * sizeof(uint8_t));
    float *amplbuf0 = (float *) malloc(circbuf_size * sizeof(float));
    size_t opposite_bank;
    const unsigned int srate = 2;
    std::vector<unsigned int> snarray;
    unsigned int sn;
    float dphase;
    size_t idx;

    float *pktphase0 = (float *) malloc(128 * sizeof(float));
    float *pktphase1 = (float *) malloc(128 * sizeof(float));
    
    mydata_t mydata;
    float aoa_ma = 0;

    FILE * fptr = fopen("angles.dat", "a");
    int c=0;
    int c_true=0;

    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    std::chrono::high_resolution_clock::time_point start0;
    std::chrono::high_resolution_clock::time_point end0;
    std::chrono::microseconds dur;
    std::chrono::microseconds dur2;
    int flag=0;
    int  flag0=0;
    int flag_2=1;
    char init_address[]="========="; // "6f61f414430c" for Tag (0c4314f4616f)

    while (stop_signal == false) {

        c++;
        select_bank = kk++ % 2;
        if (select_bank == 0) curr_bank = procpars->bankA_ptrs;
        else curr_bank = procpars->bankB_ptrs;
        pthread_spin_lock(&lock[select_bank]);

        std::complex<float> *buff0_ptr = curr_bank[0];
        std::complex<float> *buff1_ptr = curr_bank[1];

        for (int i = 0; i < samps_per_buff; i++) {
            idx = i + samps_per_buff * select_bank;

            phasebuf0[idx] = atan2f(buff0_ptr[i].imag(), buff0_ptr[i].real());
            phasebuf1[idx] = atan2f(buff1_ptr[i].imag(), buff1_ptr[i].real());
            amplbuf0[idx] = buff0_ptr[i].real() * buff0_ptr[i].real() +
                            buff0_ptr[i].imag() * buff0_ptr[i].imag();
            dphase = (idx != 0) ? phasebuf0[idx] - phasebuf0[idx-1]
                                : phasebuf0[0] - phasebuf0[circbuf_size-1];

            if (dphase > M_PI) dphase -= 2 * M_PI;
            else if (dphase < -M_PI) dphase += 2 * M_PI;
            binbuf0[idx] = (dphase > 0) ? 1 : 0;
        }
        
        opposite_bank = (select_bank + 1) % 2;

        sn = 0;

        for (int i = 0; i < samps_per_buff; i++) {

            idx = i + samps_per_buff * opposite_bank;
            uint8_t transitions = 0;
            for (int c = 0; c < 8 * srate; c += srate) {
                if (binbuf0[(idx+c+srate)%circbuf_size] > binbuf0[(idx+c)%circbuf_size]) {
                    transitions++;
                }
            }
            if (transitions != 4) continue;

            uint32_t aa = 0x00;
            uint32_t offset = idx + 8 * srate;
            for (unsigned int c = 0; c < 32 * srate; c += srate) {
                aa |= binbuf0[(offset + c) % circbuf_size] << (c >> 1);
            }
            if (aa != 0x8e89bed6) continue;

            offset = idx + (8+32) * srate;

            uint8_t   pdu_len[128]={0x00};
            for (int byte = 0; byte < 2; byte++)
                for (int b = 0; b < 8; b++)
                    pdu_len[byte] |= binbuf0[(offset+(byte*8+b)*srate)%circbuf_size] << b;
            dewhiten(pdu_len, 2,channel_idx );
            uint8_t length = pdu_len[1]+2+3;
            uint8_t pdu[128] = {0x00};
            for (int byte = 0; byte < length; byte++){
                for (int b = 0; b < 8; b++)
                    pdu[byte] |= binbuf0[(offset+(byte*8+b)*srate)%circbuf_size] << b;
            }
            dewhiten(pdu, length,channel_idx);
            
            if(check_init_address(init_address,pdu)&&globalVariable==1){
                printf("send a CONNECT_REQ \n");
                // flag_2=0;
                mtx.lock();
                globalVariable=0;
                mtx.unlock();
                startglobal=std::chrono::high_resolution_clock::now();
            }else{
                continue;
            }


            uint32_t crc_packet = 0, crc_computed = 0;

            for (int byte=0; byte<3; byte++) {
                int bitpos = 0;

                for (int b = 0x01; b <= 0x80; b <<= 1) {
                    if (pdu[pdu_len[1]+2+byte] & b) {
                        crc_packet |= 0x01 << (byte*8+bitpos);
                    }
                    bitpos++;
                }
            }
                offset = idx + (8+32) * srate;

            crc_computed = compute_crc(pdu, pdu_len[1]+2, 0x00AAAAAA);
            
            flag0=1;

           if (crc_computed != crc_packet) continue;

            c_true++;

            sn = pdu[15] | (pdu[14] << 8) | (pdu[13] << 16) | (pdu[12] << 24);

            for (int n=0; n<128 ; n++) {
                pktphase0[n] = phasebuf0[(idx+(8+32+16+8)*2+n)%circbuf_size];
                pktphase1[n] = phasebuf1[(idx+(8+32+16+8)*2+n)%circbuf_size];
            }
            break;
        }

            start0=std::chrono::high_resolution_clock::now();

        if (select_bank == 0) bankA_nsamps = 0;
        else bankB_nsamps = 0;
        
        pthread_spin_unlock(&lock[select_bank]);
         end0=std::chrono::high_resolution_clock::now();
                              dur2=std::chrono::duration_cast<std::chrono::microseconds>(end0-start0);
    }
    
    fclose(fptr);

    free(amplbuf0);
    free(pktphase0);
    free(pktphase1);
    free(binbuf0);
    free(phasebuf0);
    free(phasebuf1);

    return NULL;
}

void REC_BLE_ADV()
{   printf("- - - - -\n");
    uhd::set_thread_priority_safe();

    std::string args, channel_list, subdev;
    double rate=2e6;
    size_t total_num_samps=10e3;
    int channel_idx=37;
    channel_list="0,1";
    std::cout << "Creating USRP device..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << std::endl;

    subdev="A:0 A:0";
    usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Setting RX rate to %f MS/s\n")
                % (rate/1e6);
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX rate: %f MS/s.\n\n")
                % (usrp->get_rx_rate()/1e6);

    usrp->set_clock_source("internal");
    usrp->set_time_now(uhd::time_spec_t(0.0));
    std::cout << "Device timestamp set to 0.0 s." << std::endl;

    usrp->set_rx_freq(2402e6, 0);
    std::cout << boost::format("Center frequency set to %f MHz.\n")
                % (usrp->get_rx_freq()/1e6);

    usrp->set_rx_gain(30, 0);
    std::cout << boost::format("Gain set to %.1f dB.\n\n")
                 % (usrp->get_rx_gain());

    usrp->set_rx_antenna("RX2", 0);

    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t c = 0; c < channel_strings.size(); c++) {
        size_t chan = boost::lexical_cast<int>(channel_strings[c]);
        if (chan < usrp->get_rx_num_channels()) {
            channel_nums.push_back(boost::lexical_cast<int>(channel_strings[c]));
        } else {
            throw std::runtime_error("Invalid channel(s) specified.");
        }
    }

    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    std::cout << "Begin streaming samples... ";
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(2.0);
    rx_stream->issue_stream_cmd(stream_cmd);

    const size_t samps_per_buff = rx_stream->get_max_num_samps();
    std::vector<std::vector<std::complex<float>>> bankA_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );
    std::vector<std::vector<std::complex<float>>> bankB_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );

    std::vector<std::complex<float> *> bankA_ptrs;
    std::vector<std::complex<float> *> bankB_ptrs;
    for (size_t i = 0; i < bankA_buffs.size(); i++) {
        bankA_ptrs.push_back(&bankA_buffs[i].front());
        bankB_ptrs.push_back(&bankB_buffs[i].front());
    }
    
    uhd::rx_metadata_t md;
    double timeout = 10.0;
    size_t num_acc_samps = 0;

    size_t kk = 0;
    size_t select_bank;
    size_t num_rx_samps;
    std::vector<std::complex<float> *> curr_bank;

    pthread_spin_init(&lock[0], PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&lock[1], PTHREAD_PROCESS_PRIVATE);

    pthread_t proc_thread;
    pthread_t keyboard_thread;
    struct proc_pars_t proc_pars = {bankA_ptrs, bankB_ptrs, samps_per_buff,channel_idx};
    
    pthread_create(&proc_thread, NULL, process_routine, &proc_pars);
    pthread_create(&keyboard_thread, NULL, keyboard_routine, NULL);

    std::signal(SIGINT, &sigint_handler);
    std::cout << "Press Ctrl + C to stop streaming." << std::endl;

    int txsock = 0;
    struct sockaddr_in txaddr;
    if ((txsock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&txaddr, '0', sizeof(txaddr));
    txaddr.sin_family = AF_INET;
    txaddr.sin_port = htons(8082);

    int slavesock = 0;
    struct sockaddr_in slaveaddr;
    if ((slavesock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&slaveaddr, '0', sizeof(slaveaddr));
    slaveaddr.sin_family = AF_INET;
    slaveaddr.sin_port = htons(8081);

    if (inet_pton(AF_INET, "192.168.0.3", &txaddr.sin_addr) <= 0) stop_signal = true;
    if (inet_pton(AF_INET, "192.168.0.2", &slaveaddr.sin_addr) <= 0) stop_signal = true;

    while (stop_signal == false) {
        select_bank = kk++ % 2;

        if (select_bank == 0) curr_bank = bankA_ptrs;
        else curr_bank = bankB_ptrs;
        pthread_spin_lock(&lock[select_bank]);

        num_rx_samps = rx_stream->recv(
            curr_bank, samps_per_buff, md, timeout, false
        );

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        }
        else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
            throw std::runtime_error(md.strerror());
        }

        num_acc_samps += num_rx_samps;
        
        if (select_bank == 0) bankA_nsamps = num_rx_samps;
        else bankB_nsamps = num_rx_samps;
        pthread_spin_unlock(&lock[select_bank]);

        if (change_gain) {
            gain = fmodf(gain+10,60);
            usrp->set_rx_gain(gain, 0);
            usrp->set_rx_gain(gain, 1);
            std::cout << boost::format("Gain set to %.1f dB.\n") % (usrp->get_rx_gain());
            change_gain = false;

            cmd_t command = {1, (int) gain};
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }

        if (do_hop) {
            freq = (float) ((freq_inc) % 80000000) + 2402e6;
            freq_inc += 2000000;
            usrp->set_rx_freq(freq, 0);
            usrp->set_rx_freq(freq, 1);
            std::cout << boost::format("Frequency set to %f MHz.\n") % (usrp->get_rx_freq()/1e6);
            do_hop = false;

            int freq_integer = 2400 + freq_inc/1000000;

            cmd_t command = {2, freq_integer};
            sendto(txsock, &freq_integer, sizeof(freq_integer), 0, (struct sockaddr*)&txaddr, sizeof(txaddr)); 
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }
    }

    pthread_join(proc_thread, NULL);
    pthread_cancel(keyboard_thread);

    pthread_spin_destroy(&lock[0]);
    pthread_spin_destroy(&lock[1]);
    std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
    std::cout << "Done." << std::endl;
}


void *REC_BLE_ADV(void *pars)
{   printf("- - - - -\n");
    uhd::set_thread_priority_safe();

    std::string args, channel_list, subdev;
    double rate=2e6;
    size_t total_num_samps=10e3;
    int channel_idx=37;
    channel_list="0,1";
    std::cout << "Creating USRP device..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << std::endl;

    subdev="A:0 A:0";
    usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Setting RX rate to %f MS/s\n")
                % (rate/1e6);
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX rate: %f MS/s.\n\n")
                % (usrp->get_rx_rate()/1e6);

    usrp->set_clock_source("internal");
    usrp->set_time_now(uhd::time_spec_t(0.0));
    std::cout << "Device timestamp set to 0.0 s." << std::endl;

    usrp->set_rx_freq(2402e6, 0);
    std::cout << boost::format("Center frequency set to %f MHz.\n")
                % (usrp->get_rx_freq()/1e6);

    usrp->set_rx_gain(30, 0);
    std::cout << boost::format("Gain set to %.1f dB.\n\n")
                 % (usrp->get_rx_gain());

    usrp->set_rx_antenna("RX2", 0);
    // usrp->set_rx_antenna("TX/RX", 1);

    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for (size_t c = 0; c < channel_strings.size(); c++) {
        size_t chan = boost::lexical_cast<int>(channel_strings[c]);
        if (chan < usrp->get_rx_num_channels()) {
            channel_nums.push_back(boost::lexical_cast<int>(channel_strings[c]));
        } else {
            throw std::runtime_error("Invalid channel(s) specified.");
        }
    }

    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    std::cout << "Begin streaming samples... ";
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(2.0);
    rx_stream->issue_stream_cmd(stream_cmd);

    const size_t samps_per_buff = rx_stream->get_max_num_samps();
    std::vector<std::vector<std::complex<float>>> bankA_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );
    std::vector<std::vector<std::complex<float>>> bankB_buffs(
        usrp->get_rx_num_channels(),
        std::vector<std::complex<float>> (samps_per_buff)
    );

    std::vector<std::complex<float> *> bankA_ptrs;
    std::vector<std::complex<float> *> bankB_ptrs;
    for (size_t i = 0; i < bankA_buffs.size(); i++) {
        bankA_ptrs.push_back(&bankA_buffs[i].front());
        bankB_ptrs.push_back(&bankB_buffs[i].front());
    }
    
    uhd::rx_metadata_t md;
    double timeout = 10.0;
    size_t num_acc_samps = 0;

    size_t kk = 0;
    size_t select_bank;
    size_t num_rx_samps;
    std::vector<std::complex<float> *> curr_bank;

    pthread_spin_init(&lock[0], PTHREAD_PROCESS_PRIVATE);
    pthread_spin_init(&lock[1], PTHREAD_PROCESS_PRIVATE);

    pthread_t proc_thread;
    pthread_t keyboard_thread;
    struct proc_pars_t proc_pars = {bankA_ptrs, bankB_ptrs, samps_per_buff,channel_idx};
    
    pthread_create(&proc_thread, NULL, process_routine, &proc_pars);
    pthread_create(&keyboard_thread, NULL, keyboard_routine, NULL);

    std::signal(SIGINT, &sigint_handler);
    std::cout << "Press Ctrl + C to stop streaming." << std::endl;

    int txsock = 0;
    struct sockaddr_in txaddr;
    if ((txsock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&txaddr, '0', sizeof(txaddr));
    txaddr.sin_family = AF_INET;
    txaddr.sin_port = htons(8082);

    int slavesock = 0;
    struct sockaddr_in slaveaddr;
    if ((slavesock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) stop_signal = true;
    memset (&slaveaddr, '0', sizeof(slaveaddr));
    slaveaddr.sin_family = AF_INET;
    slaveaddr.sin_port = htons(8081);

    if (inet_pton(AF_INET, "192.168.0.3", &txaddr.sin_addr) <= 0) stop_signal = true;
    if (inet_pton(AF_INET, "192.168.0.2", &slaveaddr.sin_addr) <= 0) stop_signal = true;

    while (stop_signal == false) {
        select_bank = kk++ % 2;

        if (select_bank == 0) curr_bank = bankA_ptrs;
        else curr_bank = bankB_ptrs;
        pthread_spin_lock(&lock[select_bank]);

        num_rx_samps = rx_stream->recv(
            curr_bank, samps_per_buff, md, timeout, false
        );

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        }
        else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
            throw std::runtime_error(md.strerror());
        }

        num_acc_samps += num_rx_samps;
        
        if (select_bank == 0) bankA_nsamps = num_rx_samps;
        else bankB_nsamps = num_rx_samps;
        pthread_spin_unlock(&lock[select_bank]);

        if (change_gain) {
            gain = fmodf(gain+10,60);
            usrp->set_rx_gain(gain, 0);
            usrp->set_rx_gain(gain, 1);
            std::cout << boost::format("Gain set to %.1f dB.\n") % (usrp->get_rx_gain());
            change_gain = false;

            cmd_t command = {1, (int) gain};
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }

        if (do_hop) {
            freq = (float) ((freq_inc) % 80000000) + 2402e6;
            freq_inc += 2000000;
            usrp->set_rx_freq(freq, 0);
            usrp->set_rx_freq(freq, 1);
            std::cout << boost::format("Frequency set to %f MHz.\n") % (usrp->get_rx_freq()/1e6);
            do_hop = false;

            int freq_integer = 2400 + freq_inc/1000000;

            cmd_t command = {2, freq_integer};
            sendto(txsock, &freq_integer, sizeof(freq_integer), 0, (struct sockaddr*)&txaddr, sizeof(txaddr)); 
            sendto(slavesock, &command, sizeof(command), 0, (struct sockaddr *)&slaveaddr, sizeof(slaveaddr));
        }
    }

    pthread_join(proc_thread, NULL);
    pthread_cancel(keyboard_thread);

    pthread_spin_destroy(&lock[0]);
    pthread_spin_destroy(&lock[1]);
    std::cout << boost::format("\nReceived %u samples.\n") % num_acc_samps;
    std::cout << "Done." << std::endl;
}

