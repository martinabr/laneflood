# LaneFlood

This repository contains the source code of LaneFlood, a routing substrate based on flooding and concurrent transmissions.
To learn more about LaneFlood, check out our IEEE LCN'16 paper.

In case of questions, please contact me at *martina[dot]brachmann[at]tu-dresden[dot]de*

## Code Layout

*Disclaimer: Although we tested the code extensively, LaneFlood is a research prototype that likely contains bugs. We take no responsibility for and give no warranties in respect of using the code.*

The test application for UDP can be found here:

    /apps/laneflood-test/udp-client-stream
    /apps/laneflood-test/udp-server-stream

The test application for TCP can be found here:

    /apps/laneflood-test/tcp-client-stream
    /apps/laneflood-test/tcp-server-stream

To switch between UDP and TCP set/unset `UIP_UDP=1` and `UIP_CONF_TCP=0` in the Makefile.
Note that LaneFlood currently runs only using IPv6.

Glossy and LaneFlood source files can be found in:

    /core/net/mac

We integrate the TCP/IP stack of Contiki with LaneFlood and Glossy. 
To provide the correct interfaces this code heavily borrows from Contiki 3.x.

LaneFlood implements a network interface (`laneflood_interface`) to connect with Contiki's uIP.
