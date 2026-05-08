#!/bin/bash
# echo -e "\nStarting mission\n"
cd /home/local/BDRS_ML/mqtt-client-nav
/usr/bin/python3 mqtt-client-nav.py -n >>log_out.txt 2>>log_err.txt &
# echo "mission ended"
exit 0
