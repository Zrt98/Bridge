#ifndef AOARECEIVER
#define AOARECEIVER

#include <csignal>
#include <string>

struct rxthreadpars {
    std::string initaddr;
    std::string args;
};

extern volatile sig_atomic_t bridge_stop_signal;

void *REC_BLE_ADV(void *pars);

#endif // AOARECEIVER