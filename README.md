WiFLX: Flexible Wireless Point-to-Multipoint Data System
----

WiFLX is Layer-2 PHY/MAC point-to-multipoint wireless data system that operates
on licensed frequency bands in FDD mode using generic TX/RX capable software defined
radio (SDR) hardware. In particular, it has been developed to operate with PlutoSDR.
It implements poll-based MAC loosely based on point co-ordinated function (PCF) mode
of IEEE 802.11 MAC and uses OFDM modem from liquid-dsp for PHY operation. It implements
air-time fairness scheduler based on concepts from FQ_CODEL packet scheduler including
use of FQ_CODEL algorithm on traffic flows themselves to achieve fairness between flows
and to reduce latency by controlling queuing delay.


Design
----
See docs/mac-design.txt

Dependencies
----
WiFLX depends on following libraries.

- Standard C++17
- Boost ASIO
- Google Protobuf
- liquid-dsp
- libfec
- fftw3f
- libiio
- SoapySDR
- SoapyPlutoSDR

Build and Installation
----
```
1. Build dependencies

x86:

$ cd ~/projects/wiflx/external/build_x86
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/x86 -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc)

arm (plutosdr):

$ cd ~/projects/wiflx/external/build_arm
$ cmake -DCMAKE_TOOLCHAIN_FILE=$HOME/projects/wiflx/external/plutosdr-arm-toolchain.cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/arm -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc)

Patch protobuf installation for arm:

$ cd ~/local/arm/lib/cmake/protobuf
$ patch < ~/projects/wiflx/patches/protobuf-install-arm.patch

2. Build WiFLX

x86:

$ cd ~/projects/wiflx/build_x86
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/x86 -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc) install/strip

arm (plutosdr):

$ cd ~/projects/wiflx/build_arm
$ cmake -DCMAKE_TOOLCHAIN_FILE=$HOME/projects/wiflx/external/plutosdr-arm-toolchain.cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/arm -DCMAKE_BUILD_TYPE=Release -DPROTOBUF_PROTOC_EXEC=$HOME/local/x86/bin/protoc -Wno-dev ..
$ make -j $(nproc) install/strip

NOTE: PlutoSDR ARM build requires building plutosdr firmware with realtime Linux kernel patch and TUN driver support.
TODO - Post instructions to do so.
```

Execution
----
```
On x86 Host:

1. Open root shell
sudo su -
cd ~<user>/projects/wiflx/tests/host

2. Start AP
. ./init.sh
./start.sh ap tap0 172.16.20.1/24 wiflx_ap ap_1000khz.cfg


3. Start Remote #1
. ./init.sh
./start.sh rm1 tap1 172.16.20.2/24 wiflx_rm rm1_1000khz.cfg

4. Start Remote #2

. ./init.sh
./start.sh rm2 tap1 172.16.20.3/24 wiflx_rm rm2_1000khz.cfg

5.) Ping

ip netns exec ap ping 172.16.20.2 -s 1500&
ip netns exec ap ping 172.16.20.3 -s 1500&

6. Iperf

ip netns exec rm1 iperf -s -f k -i 1&
ip netns exec rm2 iperf -s -f k -i 1&
ip netns exec ap iperf -c 172.16.20.2 -f k -i 1 -t 30&
ip netns exec ap iperf -c 172.16.20.3 -f k -i 1 -t 30&

On PlutoSDR:

1. Login into each PlutoSDR using SSH

2. Start AP

mkdir /tmp/host
mount -t nfs -onolock 192.168.3.10:/home/agrewal /tmp/host
cd /tmp/host/projects/wiflx/tests/target/
. ./init.sh
./start.sh tap0 172.16.20.1/24 wiflx_ap ap_500khz.cfg

3. Start Remote #1

mkdir /tmp/host
mount -t nfs -onolock 192.168.4.10:/home/agrewal /tmp/host
cd /tmp/host/projects/wiflx/tests/target/
. ./init.sh
./start.sh tap1 172.16.20.2/24 wiflx_rm rm1_500khz.cfg

4. Start Remote #2

mkdir /tmp/host
mount -t nfs -onolock 192.168.2.10:/home/agrewal /tmp/host
cd /tmp/host/projects/wiflx/tests/target/
. ./init.sh
./start.sh tap2 172.16.20.3/24 wiflx_rm rm2_500khz.cfg
```

References
----

1. Ending the Anomaly: Achieving Low Latency and Airtime Fairness in WiFi
   https://arxiv.org/abs/1703.00064v2
