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
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <ctime>
#include <chrono>
#include <complex>
#include <pthread.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <mutex>
#include "aoareceiver2.hpp"
#include "blegenerator.hpp"

namespace po = boost::program_options;
static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
extern int globalVariable; 
extern std::chrono::high_resolution_clock::time_point startglobal;
extern std::chrono::high_resolution_clock::time_point endglobal;

std::mutex mtx;

struct txthreadpars {
    uhd::usrp::multi_usrp::sptr *usrp;
    size_t samps_per_buff;
    double msdelay;
    int channel;
    std::string data;
    int rounds;
    std::string initaddr;
};

int global_frequency = 0;

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
void *tx_thread_routine(void *pars)
{
    struct txthreadpars *txpar = (struct txthreadpars *) pars;
    uhd::usrp::multi_usrp::sptr usrp = *(txpar->usrp);
    const std::string cpu_format = "sc8";
    size_t samps_per_buff = txpar->samps_per_buff;
    int channel = txpar->channel;
    int rounds=txpar->rounds;
    std::vector<std::string> data_str_list;
    boost::split(data_str_list,txpar->data,boost::is_any_of(","));
    int number_of_str=data_str_list.size();

    uhd::stream_args_t stream_args(cpu_format, cpu_format);
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;

    int counter = 0;
#define ZERO_PAD_BYTES 100
    std::cout << "tx start" << std::endl;
    double old_freq=0;
    std::vector<uint8_t*> data_uint_list;
    std::vector<int> length_list;

    for(int i=0;i<number_of_str;i++){
        int length=data_str_list[i].length();
        length_list.push_back(length/2);
        char data[length];
        for(int j=0;j<length;j++){
            data[j]=data_str_list[i][j];
        }
        uint8_t *data_uint8=new uint8_t[length/2];
        for (int j = 0;j<length/2;j++){
            sscanf(data+(j*2),"%2hhx",data_uint8+j);
        }
        data_uint_list.push_back(data_uint8);

    }
    int flag=0;
    int flag_generate_sample=0;
    int tempV;
    std::vector<std::vector<int8_t>> buffAll;
    int final_flag=0;
    long int timedelay = 255000;
    long int packet1delay;

    std::vector<int> channel_list = {37,37,38,39,4};

    for(int i=0;i<number_of_str;i++){
            flag=0;

            channel = channel_list[i];

            int len_packet=length_list[i];

            if (flag_generate_sample < number_of_str) { // Pre-generate sample
                if(flag_generate_sample == number_of_str -1){
                        flag=4;
                }
                buffAll.push_back(generatesamples(channel, counter, ZERO_PAD_BYTES, data_uint_list[i],len_packet,flag));
                flag_generate_sample += 1;

                if(flag_generate_sample == number_of_str){
                    // std::vector<int8_t> X;
                    // std::vector<int8_t> Y;
                    // X.insert(X.end(),buffAll[0].begin(),buffAll[0].end());
                    // X.insert(X.end(),buffAll[1].begin(),buffAll[1].end());
                    // Y.insert(Y.end(),buffAll[2].begin(),buffAll[2].end());

                    // buffAll[0]=X;
                    // buffAll[1]=Y;
                    // buffAll[2]=Y;
                    // buffAll[3]=X;
                    final_flag=1;
                    // std::cout << "Packet Generated!" << std::endl;
                }
            }
    }

    do {

        /*if (old_freq != First_Packet_Freq){ //Set 1st packet Freq
                double txfreq = First_Packet_Freq;
                uhd::tune_request_t tune_request = uhd::tune_request_t(txfreq);
                usrp->set_tx_freq(tune_request);
                old_freq = First_Packet_Freq;
                printf("Set to %lf frequency!\n",txfreq);
                usleep(5000);

                mtx.lock();
                globalVariable=1;
                mtx.unlock(); 
        }*/
        
        mtx.lock();
        tempV=globalVariable;
        mtx.unlock();
        // printf("temp=%d ",tempV);

        if(tempV==1){
            continue;
        }
        // std::chrono::high_resolution_clock::time_point startTime2;

        for(int i=0;i<number_of_str;i++){
            flag=0;
            // if(final_flag==0){
            // if((data_str_list[i].find("4522"))!=-1){ //A
            //     // printf("send a CONNECT_REQ and wait 326000us\n");
            //     // usleep(2000);
            //     channel=37;
            // }else if((data_str_list[i].find("0707"))!=-1){ //B
            //     channel=37;
            //     // usleep(2000);
            // }else if((data_str_list[i].find("0716"))!=-1){ //D
            //     channel=4;
            // // }else if((data_str_list[i].find("0100"))!=-1){ //C
            // //     flag=1;
            // //     // usleep(2115);
            // //     channel=4;
            // }else if((data_str_list[i].find("0720"))!=-1){ //C
            //     // usleep(2115);
            //     channel=4;
            // }else if((data_str_list[i].find("072a"))!=-1){ //C
            //     // usleep(2115);
            //     channel=4;
            // }
            // else if((data_str_list[i].find("0108"))!=-1){ //C
            //     flag=1;
            //     usleep(2115);
            //     channel=4;
            // }


            // }else{ // MAIN LOOP
                 if(i==0){ //1st
                // printf("send a CONNECT_REQ and wait 326000us\n");
                // usleep(104);
                channel=37;
            }else if(i==1){ //2nd
                channel=37;
                usleep(200);
            }else if(i==2){ //3rd
                channel=38;
                usleep(200);
            }else if(i==3){ //4th
                channel=39;
                usleep(200);
            }else if(i==4){ //4th
                channel=4;
            }

            double new_freq=ble_chan2freq(channel);
            
            std::chrono::high_resolution_clock::time_point startTime=std::chrono::high_resolution_clock::now();
            if (new_freq != old_freq) {
                double txfreq = new_freq;
                uhd::tune_request_t tune_request = uhd::tune_request_t(txfreq);
                usrp->set_tx_freq(tune_request);
                old_freq = new_freq;
                usleep(1000);
                printf("switch to %lf frequency!\n",txfreq);
            }
            else {
                printf("keep in the same frequency!\n");
                // usleep(10000);
            }

            std::chrono::high_resolution_clock::time_point endTime=std::chrono::high_resolution_clock::now();
            std::chrono::microseconds durationTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            std::cout << "Elapsed time2: " << durationTime.count() << " microseconds" << std::endl;

            int len_packet=length_list[i];
            
            // std::cout << "Buff Size: " << buffAll.size() << std::endl;
            
            std::vector<int8_t> buff = buffAll[i];

            size_t num_tx_samps = buff.size() / 2;
            md.end_of_burst = true;
             if(i == number_of_str -1){ //3rd
                // startTime2=std::chrono::high_resolution_clock::now();
                std::chrono::high_resolution_clock::time_point endTime2=std::chrono::high_resolution_clock::now();
                std::chrono::microseconds durationTime2 = std::chrono::duration_cast<std::chrono::microseconds>(endTime2 - startglobal);
                if(durationTime2.count() - packet1delay < timedelay){
                    // printf("count to%ld",timedelay-durationTime2.count());
                    usleep(timedelay-durationTime2.count()+packet1delay);
                }
            }

            int ssent = (int) tx_stream->send(&buff.front(), num_tx_samps, md);

            endglobal=std::chrono::high_resolution_clock::now();
            std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(endglobal - startglobal);
            std::cout << "Elapsed time: " << duration.count() << " microseconds" << std::endl;
            if (i==1){
                packet1delay=duration.count();
            }

            // std::cout << "Sent synch" << std::endl;
            //boost::this_thread::sleep(boost::posix_time::milliseconds(10)); // txpar->msdelay));
            counter ++;
            //usleep(10000);
            // rounds--;

        }

        // Reset to  default CH: CH37
                usleep(1000);
                double temp_freq=ble_chan2freq(37);
                uhd::tune_request_t tune_request = uhd::tune_request_t(temp_freq);
                usrp->set_tx_freq(tune_request);
                old_freq = temp_freq;
                usleep(1000);
                printf("Reset to %lf frequency!\n",temp_freq);

                mtx.lock();
                globalVariable=1;
                mtx.unlock(); 
    } while(rounds>0&&not stop_signal_called);

    //finished
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

    double txrate = 4000000;
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
    printf("Setting TX Freq ------> %lf",txfreq);
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


int UHD_SAFE_MAIN(int argc, char *argv[])
{

    uhd::set_thread_priority_safe();

    std::string args;
    std::string data;
    int rounds;
    double txgain;
    int channel;
    std::string initaddr;
    size_t samples_per_buffer = 10000;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("channel", po::value<int>(&channel)->default_value(37), "ble channel")
        ("data", po::value<std::string>(&data)->default_value(""), "raw data")
        ("rounds", po::value<int>(&rounds)->default_value(10), "rounds")
	    ("txgain", po::value<double>(&txgain)->default_value(50), "tx gain for the RF chain")
        ("initaddr", po::value<std::string>(&initaddr)->default_value("6f61f414430c"), "initiator address")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << boost::format("ble advertiser. %s") % desc << std::endl;
        std::cout << std::endl;
        return ~0;
    }

    uhd::usrp::multi_usrp::sptr usrp;
    usrp = usrp_setup("chain", args, txgain, channel);

    double setup_time = 1;

    check_locked_tx(usrp);
    boost::this_thread::sleep(boost::posix_time::seconds(1));

    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

    pthread_t txthread;
    pthread_t txthread2;
    struct txthreadpars txpar;
    txpar.usrp = &usrp;
    txpar.samps_per_buff = samples_per_buffer;
    txpar.channel = channel;
    txpar.data=data;
    txpar.rounds=rounds;
    txpar.initaddr =initaddr;
    
    pthread_create(&txthread2, NULL, REC_BLE_ADV, NULL);
    // while(true){
    //     printf("%d",globalVariable);
    //     if(globalVariable==0){
    //         printf("%d",globalVariable);
    //         break;
    //     } 

    printf("begin to send \n");
    pthread_create(&txthread, NULL, tx_thread_routine, (void *) &txpar);

    int soc = socket(PF_INET, SOCK_DGRAM, 0);
    if( soc == -1 ) {
        fprintf(stderr, "Cannot create socket\n");
        return -1;
    }

    struct sockaddr_in local, remote;
    local.sin_family = PF_INET;
    local.sin_port = htons ((short) 8082);
    local.sin_addr.s_addr = htonl (INADDR_ANY);

    if (bind (soc, (struct sockaddr*)&local, sizeof(local)) == -1) {
        fprintf (stderr, "Cannot bind socket\n");
        close (soc);
        return 1;
    }

#define BUFFER_LENGTH 1000
    char buffer[BUFFER_LENGTH];

    socklen_t size_remote = sizeof(remote);
    while (not stop_signal_called) {
        int retc = recvfrom(soc, buffer, BUFFER_LENGTH, 0, (struct sockaddr*) &remote, &size_remote);
	if (retc == -1) {
            fprintf(stderr, "Error receiving from socket\n");
            close (soc);
            return 1;
        }
        int *frequency = (int *) buffer;
        printf("new frequency = %d\n", *frequency);
        global_frequency = *frequency;
    }

    pthread_join (txthread, NULL);

    return EXIT_SUCCESS;
}
