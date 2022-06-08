WiFLX: Flexible Wireless Point-to-Multipoint Data System
----

WiFLX is Layer-2 PHY/MAC point-to-multipoint wireless data system that operates
on licensed frequency bands in FDD mode using generic TX/RX capable software defined
radio (SDR) hardware. In particular, it has been developed to operate with (as well
as on) PlutoSDR. It implements poll-based MAC loosely based on point co-ordinated
function (PCF) mode of IEEE 802.11 MAC and uses OFDM modem from liquid-dsp for PHY
operation. It implements air-time fairness based on concepts from FQ_CODEL packet
scheduler including use of FQ_CODEL ([RFC8290](https://tools.ietf.org/html/rfc8290))
on traffic flows themselves to achieve fairness between flows and use of CODEL
([RFC8289](https://tools.ietf.org/html/rfc8289)) on flow queues to reduce latency
by controlling queuing delay.

Design
----
![On PlutoSDR](/docs/wiflx-stack-design-on-plutosdr.jpg)

![On x86 Host](/docs/wiflx-stack-design-on-x86.jpg)

See docs/mac-design.txt for protocol operation.

Dependencies
----
WiFLX depends on following libraries.

- Standard C++17
- Boost ASIO, MSM
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
x86:

1. Build dependencies

$ cd ~/projects/wiflx/external/build_x86
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/x86 -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc)

2. Build WiFLX

$ cd ~/projects/wiflx/build_x86
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/x86 -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc) install/strip

arm (plutosdr):

1. PlutoSDR ARM build requires building plutosdr firmware with TUN driver support
and realtime linux kernel patch. Follow instructions in section "PlutoSDR Firmware Build"

2. Build dependencies

$ cd ~/projects/wiflx/external/build_arm
$ cmake -DCMAKE_TOOLCHAIN_FILE=$HOME/projects/wiflx/external/plutosdr-arm-toolchain.cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/arm -DCMAKE_BUILD_TYPE=Release -Wno-dev ..
$ make -j $(nproc)

Patch protobuf installation for arm:

$ cd ~/local/arm/lib/cmake/protobuf
$ patch < ~/projects/wiflx/patches/protobuf-install-arm.patch

3. Build WiFLX

$ cd ~/projects/wiflx/build_arm
$ cmake -DCMAKE_TOOLCHAIN_FILE=$HOME/projects/wiflx/external/plutosdr-arm-toolchain.cmake -DCMAKE_INSTALL_PREFIX=$HOME/local/arm -DCMAKE_BUILD_TYPE=Release -DPROTOBUF_PROTOC_EXEC=$HOME/local/x86/bin/protoc -Wno-dev ..
$ make -j $(nproc) install/strip
```

PlutoSDR Firmware Build
----
```
1. Download and install Xilinx Vivado SDK (Free WebPACK):

https://www.xilinx.com/member/forms/download/xef-vivado.html?filename=Xilinx_Vivado_SDK_Web_2019.1_0524_1430_Lin64.bin

Reference: https://wiki.analog.com/university/tools/pluto/building_the_image

2. Dowload plutosdr firmware source

https://wiki.analog.com/university/tools/pluto/obtaining_the_sources

$ cd ~/projects
$ git clone https://github.com/analogdevicesinc/plutosdr-fw.git
$ git checkout -b v0.32 v0.32
$ git submodule update --init --recursive

3. Apply realtime linux patch

a. determine linux kernel version

$ cd ~/projects/plutosdr-fw/linux
$ make kernelversion
4.19.0

b. Download and apply patch

$ cd ~/tmp
$ wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/4.19/older/patch-4.19-rt1.patch.gz
$ gzip -d patch-4.19-rt1.patch.gz
$ cd -
$ patch -p1 < ~/tmp/patch-4.19-rt1.patch
$ cd ..

4. Setup environment variables for Xilinx toolchain (assumes it is installed
   in $HOME/opt)

$ export CROSS_COMPILE=arm-linux-gnueabihf-
$ export PATH=$PATH:$HOME/opt/Xilinx/SDK/2019.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin/
$ export VIVADO_SETTINGS=$HOME/opt/Xilinx/Vivado/2019.1/settings64.sh

5. Configure kernel features

$ make -C linux ARCH=arm zynq_pluto_defconfig
$ make -C linux ARCH=arm menuconfig

- Enable RT kernel
General Setup -> Preemption Model -> select 'Fully Preemptible Kernel (RT)'

- Enable TUN/TAP driver
Device Drivers -> Network device support -> Universal TUN/TAP device driver support. Select 'Y'.

- Enable NFS client
File systems -> Network File Systems -> NFS client support. Select 'Y'.

- Save config
$ cp linux/.config linux/arch/arm/configs/zynq_pluto_defconfig

6. Configure userspace apps (optional)

$ make -C buildroot ARCH=arm zynq_pluto_defconfig
$ make -C buildroot ARCH=arm menuconfig

- Target Packages->Networking Applications
Select iperf, iperf3, tcpdump, iproute2, tunctl

- Save config
$ make -C buildroot ARCH=arm savedefconfig

7. Add WiFLX apps (optional, assumes they have been built already)

- System configuration ---> Root filesystem overlay directories

set overlay path to $HOME/projects/plutosdr-fw/rootfs-overlay

$ cd ~/project/wiflx
$ ./overlay_onto_plutosdr_fw.sh
$ cd ~/project/plutosdr-fw

- Save config
$ make -C buildroot ARCH=arm savedefconfig

8. Build firmware image

$ make

9. Upgrade PlutoSDR with firmware

$ cp build/pluto.frm /media/$HOME/PlutoSDR/
$ mount | grep PlutoSDR | awk '{print $1}'
/dev/sdb1
$ sudo eject /dev/sdb
$ echo “WAIT 4 minutes for firmware to update!!!!”

Reference:
https://wiki.analog.com/university/tools/pluto/users/firmware

10. Executing WiFLX apps packaged into pluto firmware

# SOAPY_SDR_ROOT=.. LD_LIBRARY_PATH=../lib ../bin/wiflx_test test_sc_tx_50khz.cfg

```

Physical Test Setup
----
The following figure shows the physical test setup with middle PlutoSDR acting as the access point and those to the left and right as remote#1 and #2 respectively with AP's TX port connected to remotes' RX port and vice versa with cabled RF connections through a splitter. Each PlutoSDR is connected via USB to the x86 development host running Linux.

![wilfx_test_setup](/docs/wilfx_test_setup.jpg)


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
./start.sh rm2 tap2 172.16.20.3/24 wiflx_rm rm2_1000khz.cfg

5.) Ping

ip netns exec ap ping 172.16.20.2 -s 1500&
ip netns exec ap ping 172.16.20.3 -s 1500&

6. Iperf

ip netns exec rm1 iperf -s -f k -i 1&
ip netns exec rm2 iperf -s -f k -i 1&
ip netns exec ap iperf -c 172.16.20.2 -f k -i 1 -t 30&
ip netns exec ap iperf -c 172.16.20.3 -f k -i 1 -t 30&

On PlutoSDR:

1. Log into each PlutoSDR using SSH

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

Debugging
----
#### Using GPO for MAC protocol timing analysis

PlutoSDR supports general purpose outputs (GPO) that can be driven from the ARM application processor. The GPIO based debugging can be enabled by enabling WIFLX_GPIO_DEBUG cmake option (i.e. giving -DWIFLX_GPIO_DEBUG=1 to cmake during project configuration). The src/common/ofdm.cpp has been marked GPIO set/clear APIs to measure frame preparation, transmission as well as reception times:

* Receive
  * GPO0 is driven HIGH when OFDM data frame is received and driven LOW after upper layer callback has processed received payload.
* Transmit
  * GPO2 is driven HIGH when OFDM frame preparation is started and driven LOW when it is completed.
  * GPO1 is driven HIGH when OFDM frame transmission (to the radio) is started and driven LOW when it is completed.

Please refer to "GPO Manual Control" at (https://wiki.analog.com/resources/tools-software/linux-drivers/iio-transceiver/ad9361)

A logic analyzer (like DigiView DV518) can be used to analyze these outputs, which can help verify MAC protocol operation and timing. Following figure shows the physical test setup:

![wilfx_test_setup_gpio_debug](/docs/wilfx_test_setup_gpio_debug.jpg)

Following figure shows data transfer (ping) from remote to access point including the initial random access message exchange:

![wiflx_200khz_rm_to_ap_ra_n_data_transfer](/docs/wiflx_200khz_rm_to_ap_ra_n_data_transfer.jpg)

Following figure shows an ongoing  data transfer (TCP) from the access point to a remote:

![wiflx_200khz_ap_to_rm_data_transfer](/docs/wiflx_200khz_ap_to_rm_data_transfer.jpg)

#### Using TRACY profiler for MAC protocol timing analyis

The [Tracy profiler](https://github.com/wolfpld/tracy) is a great piece of software that enables profiling of C/C++ applications with minimal overhead. The application can be run on a target that has a TCP/IP network connection to the development machine runing the Tracy server/viewer. Tracy profiling support can be enabled by enabling WIFLX_TRACY_PROFILING cmake option. The src/common/ofdm.cpp has been marked with profiling scopes to measure frame preparation, transmission as well as reception times.

1. Start Tracy server/viewer on the development host:

 [~/projects/tracy/profiler/build/unix]
 $ ./Tracy-release

2. Provide the IP address of the target running either wiflx_ap or wiflx_rm application and click 'Connect':
![tracy_server_connection_to_target](/docs/tracy_server_connection_to_target.png)

Following screeshot shows Tracy server/viewer in action:
![wiflx_ap_tracy_profiler_200khz_ap_to_rm_data_transfer](/docs/wiflx_ap_tracy_profiler_200khz_ap_to_rm_data_transfer.png)

References
----

#### Interesting/related SDR projects

1. HNAP4PlutoSDR - https://github.com/HAMNET-Access-Protocol/HNAP4PlutoSDR
2. Charon - https://github.com/tvelliott/charon
3. LeadSDR - https://github.com/pabr/leansdr

#### Books:
1. Software Defined Radio For Engineers - https://www.analog.com/media/en/training-seminars/design-handbooks/Software-Defined-Radio-for-Engineers-2018/SDR4Engineers.pdf
2. Software Receiver Design - https://www.amazon.com/Software-Receiver-Design-Digital-Communication/dp/0521189446
3. DSP in Modern Communication Systems - https://www.amazon.com/Digital-Signal-Processing-Communication-Systems/dp/0988873508

#### Papers:
1. Ending the Anomaly: Achieving Low Latency and Airtime Fairness in WiFi https://arxiv.org/abs/1703.00064v2
