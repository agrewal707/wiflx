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
$ git clone --recurse-submodules -j8 https://github.com/analogdevicesinc/plutosdr-fw.git

or if repo has already been cloned ensure it is up to date:

$ git pull --recurse-submodules

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

4. Configure kernel features

$ make -C linux ARCH=arm zynq_pluto_defconfig
$ make -C linux ARCH=arm menconfig

a.) Enable RT kernel
General Setup -> Preemption Model -> select 'Fully Preemptible Kernel (RT)'

b.) Enable TUN/TAP driver
Device Drivers -> Network device support -> Universal TUN/TAP device driver support. Select 'Y'.

5. Configure userspace apps (optional)

make -C buildroot ARCH=arm zynq_pluto_defconfig

- Target Packages->Network Applications

Select iperf, iperf3, tcpdump, iproute2, tunctl

6. Build firmware image

$ make

7. Upgrade PlutoSDR with firmware

$ cp .build/pluto.frm /media/analog/PlutoSDR/
$ mount | grep PlutoSDR | awk '{print $1}'
/dev/sdb1
$ sudo eject /dev/sdb
$ echo “WAIT 4 minutes for firmware to update!!!!”

Reference:
https://wiki.analog.com/university/tools/pluto/users/firmware

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

References
----

1. Ending the Anomaly: Achieving Low Latency and Airtime Fairness in WiFi
   https://arxiv.org/abs/1703.00064v2
