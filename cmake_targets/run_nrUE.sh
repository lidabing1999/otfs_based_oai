#!/bin/bash
sudo RFSIMULATOR=127.0.0.1 ./ran_build/build/nr-uesoftmodem -r 106 --numerology 1 -C 3619200000  -O ../targets/PROJECTS/GENERIC-NR-5GC/CONF/ue.conf --nokrnmod --rfsim --sa
