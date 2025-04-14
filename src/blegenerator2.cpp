/*
 * Copyright 2017 Francesco Gringoli <francesco.gringoli@unibs.it>
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
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <pthread.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include "blegenerator.hpp"

namespace po = boost::program_options;
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}

struct txthreadpars {
    uhd::usrp::multi_usrp::sptr *usrp;
    size_t samps_per_buff;
    double msdelay;
    int channel;
    std::string data;
    int rounds;
};

int global_frequency = 0;
uint8_t* data2Char(std::string rawdata){
    int str_len=rawdata.length();
    uint8_t data[str_len];
    for(int i=0;i<str_len;++i){
        data[i]=(uint8_t)rawdata[i];
    }
    return data;
}

void *tx_thread_routine(void *pars)
{
    struct txthreadpars *txpar = (struct txthreadpars *) pars;
    uhd::usrp::multi_usrp::sptr usrp = *(txpar->usrp);
    const std::string cpu_format = "sc8";
    size_t samps_per_buff = txpar->samps_per_buff;
    int channel = txpar->channel;
    int rounds=txpar->rounds;
    int length=txpar->data.length();
    char data[length];
    for(int i=0;i<length;i++){
        data[i]=txpar->data[i];
    }
    uint8_t data_uint8[64];
    for (int i = 0;i<sizeof(data)/2;i++){
        sscanf(data+(i*2),"%2hhx",data_uint8+i);
    }
    for (int i = 0; i < sizeof(data)/2; i++) {
        printf("0x%02X ", data_uint8[i]);
    }
    uhd::stream_args_t stream_args(cpu_format, cpu_format);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    int counter = 0;
#define ZERO_PAD_BYTES 100
    std::cout << "tx start" << std::endl;

    int old_global_frequency = global_frequency;
    int flag=0;
    do {
        int len_packet=sizeof(data_uint8);
        std::vector<int8_t> buff = generatesamples(channel, counter, ZERO_PAD_BYTES, data_uint8,len_packet,flag);
        size_t num_tx_samps = buff.size() / 2;
        md.end_of_burst = true;
        int ssent = (int) tx_stream->send(&buff.front(), num_tx_samps, md);
	counter ++;
        if (global_frequency != old_global_frequency) {
            double txfreq = double(global_frequency) * 1000000;
            uhd::tune_request_t tune_request = uhd::tune_request_t(txfreq);
            usrp->set_tx_freq(tune_request);
            old_global_frequency = global_frequency;
        }
        else {
	    usleep(10000);
        } 
      rounds--;
      printf("rounds-> %d",rounds);
    } while( rounds>0);

    std::cout << std::endl << "tx stream done!" << std::endl << std::endl;
    return NULL;

}

bool check_locked_tx(uhd::usrp::multi_usrp::sptr usrp)
{
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked",0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    return true;
}

double ble_chan2freq(int channel)
{
	double freq = 2402;
        if(channel < 11)
                freq = 2404 + channel * 2;
        else if(channel < 37)
                freq = 2428 + (channel - 11) * 2;
        else if(channel == 37)
                freq = 2402;
        else if(channel == 38)
                freq = 2426;
        else if(channel == 39)
                freq = 2480;
        else {
                fprintf(stderr, "Invalid channel, defaulting to 37\n");
                freq = 2402;
        }
	return freq * 1e6;
}

uhd::usrp::multi_usrp::sptr usrp_setup(std::string chainname,
				       std::string args,
				       double txgain,
				       int blechannel)
{
    double txfreq = ble_chan2freq(blechannel);

    std::cout << boost::format("Setting up chain %s ") % chainname << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    std::string ref = "internal";
    usrp->set_clock_source(ref);

    double txrate = 2000000;
    std::cout <<
	boost::format("Setting TX Rate for %s: %f Msps...") % chainname % (txrate / 1e6) <<
	std::endl;
    usrp->set_tx_rate(txrate);
    std::cout <<
	boost::format("Actual TX Rate for %s: %f Msps...") % chainname % (usrp->get_tx_rate() / 1e6) <<
	std::endl;

    std::cout <<
	boost::format("Setting TX Freq for %s: %f MHz...") % chainname % (txfreq / 1e6) <<
	std::endl;
    uhd::tune_request_t tune_request = uhd::tune_request_t(txfreq);

    usrp->set_tx_freq(tune_request);
    std::cout <<
	boost::format("Actual TX Freq for %s: %f MHz...") % chainname % (usrp->get_tx_freq() / 1e6) <<
	std::endl;

    std::cout <<
	boost::format("Setting TX Gain for %s: %f dB...") % chainname % txgain <<
	std::endl;
    usrp->set_tx_gain(txgain);
    std::cout << boost::format("Actual TX Gain for %s: %f dB...") % chainname % usrp->get_tx_gain() <<
	std::endl;

    std::string txant = "TX/RX";
    usrp->set_tx_antenna(txant);

    return usrp;
}

