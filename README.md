# BRIDGE

**BRIDGE** is a software-defined radio (SDR) solution that leverages an additional full-duplex trigger node (**TRIGGER**) to make **Bluetooth 5.1 Direction Finding (Angle of Arrival, AoA)** compatible with standard Bluetooth Low Energy (BLE) devices — without requiring native BLE 5.1 Direction Finding support on the target device.

This repository contains the main software implementation for controlling the **TRIGGER** node (built on UHD/USRP), helper scripts for fixed and randomized BLE MAC addresses, example IQ sample datasets, and the companion artifact appendix ([`Bridge artifact.pdf`](./Bridge%20artifact.pdf)).

---

## How It Works

1. **Advertising Sniffing (RX):** The `TRIGGER` node continuously listens on BLE advertising channel **37** (`2402 MHz`, `2 MS/s`) for advertising packets matching the target device's MAC address.
2. **Synchronized Packet Transmission (TX):** Upon detecting the target's advertising packet, the `TRIGGER` node transmits a pre-generated sequence of crafted BLE packets (GFSK-modulated with CRC-24 and BLE whitening at `4 MS/s`) across channels **37, 37, 38, 39, and 4** with microsecond-level timing synchronization.
3. **IQ Sample Collection:** Commercial BLE AoA locators (e.g., Silicon Labs Direction Finding locators) receive the resulting transmissions and output IQ samples and RSSI measurements, which can be monitored and recorded over MQTT.

---

## Repository Structure

```text
.
├── Bridge/
│   ├── CMakeLists.txt              # CMake build configuration (UHD + Boost)
│   └── src/
│       ├── aoatransmitter.cpp      # Main BRIDGE TRIGGER program (TX + RX coordination)
│       ├── aoareceiver2.cpp/.hpp   # BLE Ch. 37 advertising sniffer thread (REC_BLE_ADV)
│       ├── blegenerator.cpp/.hpp   # BLE PHY packet builder (CRC-24, whitening, GFSK IQ)
│       ├── aoareceiver.cpp         # Standalone dual-channel USRP BLE AoA receiver
│       └── blegenerator2.cpp/.hpp  # Auxiliary single-packet BLE TX module
├── fix.sh                          # Wrapper script for targets with a fixed (public) MAC address
├── random.sh                       # Wrapper script for targets with a randomized MAC address
├── Bridge artifact.pdf             # Detailed artifact appendix and evaluation checklist
├── example IQ samples.zip          # Example IQ sample dataset (.mat files from 4 locators)
└── README.md
```

---

## Requirements

### Hardware Dependencies

- **Host Machine:** Linux PC/laptop (tested on a 4-core CPU laptop with at least **4 GB RAM** and **< 1 GB** free disk space).
- **SDR Transceiver:** A full-duplex Software-Defined Radio supported by UHD (tested with an **Ettus Research USRP N210**).
- **BLE AoA Locators:** Bluetooth 5.1 Direction Finding antenna-array locators (tested with **Silicon Labs** direction finding locators).

### Software Dependencies

- **OS:** Ubuntu Linux (implemented and tested on **Ubuntu 20.04**).
- **Build Tools:** `cmake` (>= 2.8), `g++`, `make`, POSIX threads (`pthread`).
- **UHD (USRP Hardware Driver):** Version >= 3.8.0 required by CMake; **> 3.10.0.0 recommended** (see [Ettus UHD & GNU Radio installation guide](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux)).
- **Boost C++ Libraries:** Version >= 1.46 (`program_options`, `system`, `thread`).
- **Companion Tools (for experiments):**
  - **MQTT Explorer** (or any MQTT client) to monitor IQ sample streams from the locators.
  - **Simplicity Connect** mobile app (or another BLE scanner) to identify the active MAC address of devices using BLE address randomization.

On Ubuntu 20.04, the core build dependencies (excluding a custom UHD source build) can be installed with:

```bash
sudo apt update
sudo apt install -y build-essential cmake libboost-all-dev libuhd-dev uhd-host
```

> **Note:** If you build UHD and GNU Radio from source, follow the official [Ettus Research Linux Toolchain Guide](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux).

---

## Installation & Setup

### Step 1: Set Up the UHD Environment

Install UHD (version > 3.10.0.0 recommended) and verify that your USRP device is reachable:

```bash
uhd_find_devices
```

### Step 2: Build the BRIDGE Software

> **Important:** The helper scripts [`fix.sh`](./fix.sh) and [`random.sh`](./random.sh) currently reference the source path `~/Desktop/Bridge/src/aoareceiver2.cpp` and expect to be executed from inside the `build/` directory.

1. Clone this repository and place the inner `Bridge` folder on your Desktop (`~/Desktop/Bridge`):

   ```bash
   git clone https://github.com/Zrt98/Bridge.git bridge-repo
   cp -r bridge-repo/Bridge ~/Desktop/Bridge
   ```

2. Build the executables (`aoatransmitter` and `aoareceiver`):

   ```bash
   cd ~/Desktop/Bridge
   mkdir -p build
   cd build
   cmake ..
   make
   ```

3. Copy the helper scripts into the `build` directory and make them executable:

   ```bash
   cp /path/to/bridge-repo/fix.sh /path/to/bridge-repo/random.sh ~/Desktop/Bridge/build/
   chmod +x ~/Desktop/Bridge/build/fix.sh ~/Desktop/Bridge/build/random.sh
   ```

### Step 3: Deploy and Configure the AoA Locators

Set up your Bluetooth Direction Finding locators following the Silicon Labs Quick Start Guide:

- [QSG175: Silicon Labs Direction Finding Solution Quick-Start Guide (PDF)](https://www.silabs.com/documents/public/quick-start-guides/qsg175-direction-finding-solution-quick-start-guide.pdf)

Ensure that:
1. The locator boards are flashed with the AoA locator firmware and connected to the AoA host application / MQTT broker.
2. An MQTT client (such as **MQTT Explorer**) is connected to the broker to monitor incoming IQ reports.

---

## Running Experiments

### 1. Identify the Target MAC Address

- **Fixed (Public) MAC Address:** Record the target device's static 6-byte BLE MAC address (e.g., `AA:BB:CC:DD:EE:FF`).
- **Randomized MAC Address:** Use a BLE scanner app (e.g., **Simplicity Connect** on Android) to locate the target by its advertised device name and record its current MAC address. Randomized addresses typically rotate every **10–15 minutes**.

> **Byte-Order Requirement:** BLE advertising payloads transmit addresses in **little-endian (reversed byte) order**. Pass the 12-hex-character MAC address to the scripts with **no colons and bytes reversed**:
>
> - Standard MAC address: `AA:BB:CC:DD:EE:FF`
> - Script argument: `FFEEDDCCBBAA`
>
> *(Example from `aoareceiver2.cpp`: a tag with address `0C:43:14:F4:61:6F` uses `6f61f414430c`.)*

### 2. Run BRIDGE

From inside `~/Desktop/Bridge/build`:

- **For a target with a fixed (public) MAC address:**

  ```bash
  cd ~/Desktop/Bridge/build
  ./fix.sh FFEEDDCCBBAA
  ```

- **For a target with a randomized MAC address:**

  ```bash
  cd ~/Desktop/Bridge/build
  ./random.sh FFEEDDCCBBAA
  ```

  > For long-running data collection with randomized-address targets, monitor for address rotation every **10–15 minutes**, stop the process (`Ctrl+C`), and re-run `./random.sh` with the updated reversed MAC address.

#### What the Scripts Do

Both scripts temporarily substitute the target address placeholder (`=========`) in `~/Desktop/Bridge/src/aoareceiver2.cpp`, rebuild `aoatransmitter`, restore the placeholder, and run:

- `fix.sh`: launches `aoatransmitter` with `--txgain 20`, `--rounds 20000`, and a fixed-address (`0522...`) packet payload sequence.
- `random.sh`: launches `aoatransmitter` with `--txgain 20`, `--rounds 20000`, and a randomized-address (`C522...`) packet payload sequence.

> **Tip:** If you interrupt `fix.sh` or `random.sh` while `make` is running, verify that `init_address` in `~/Desktop/Bridge/src/aoareceiver2.cpp` has been restored to `"========="`.

### 3. Monitor Output & Expected Results

Once BRIDGE is running and synchronized with the target, the AoA locators will begin generating IQ samples for the target device. Use **MQTT Explorer** to observe the incoming IQ packets in real time. A typical complete data sampling period takes approximately **15 minutes**.

---

## Command-Line Reference (`aoatransmitter`)

If you want to invoke `aoatransmitter` directly without the wrapper scripts, the supported options are:

| Option | Default | Description |
| --- | --- | --- |
| `--help` | — | Print help message and exit |
| `--args` | `""` | UHD device address arguments (e.g., `"addr=192.168.10.2"`) |
| `--channel` | `37` | Initial BLE channel |
| `--data` | `""` | Comma-separated hex strings for the 5-packet TX sequence |
| `--rounds` | `10` | Number of transmission rounds |
| `--txgain` | `50` | TX RF gain in dB (`fix.sh` / `random.sh` use `20`) |
| `--initaddr` | `6f61f414430c` | Initiator address parameter |

---

## Example IQ Samples Dataset

[`example IQ samples.zip`](./example%20IQ%20samples.zip) contains **99 MATLAB `.mat` files** collected during evaluation (along with an internal `Readme.txt`):

- **Naming Convention:** `A-B.mat`
  - `A`: Target device index (`1`–`10` for standard target devices; `0` for a native BLE v5.1 AoA tag baseline).
  - `B`: Evaluation position index (`1`–`9`).
- **Contents:** Each file stores IQ sample arrays collected simultaneously from **4 locators** (`IQData1` through `IQData4`) along with the corresponding **RSSI** value for each received packet.

---

## Artifact Documentation

For the full ACM Artifact Appendix — including the meta-information checklist, experimental setup timeline, and evaluation methodology — please refer to [`Bridge artifact.pdf`](./Bridge%20artifact.pdf).

---

## Acknowledgments & License

Components of the UHD/USRP BLE SDR implementation and build configuration are adapted from open-source GPLv3 software by **Ettus Research LLC**, **Francesco Gringoli**, **Marco Cominelli**, **Jiang Wei**, **Xianjun Jiao**, and **Florian Echtler**. Source files are licensed under the **GNU General Public License v3.0 (GPL-3.0)**; see the file headers in `Bridge/src/` and `Bridge/CMakeLists.txt` for details.
