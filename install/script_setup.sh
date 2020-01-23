#!/bin/bash

mv /home/nao/scripts/autoload.ini /home/nao/naoqi/preferences/autoload.ini

ETH0_NAME=$( connmanctl services | grep -o 'ethernet\w*cable' )
sudo connmanctl config $ETH0_NAME --ipv4 manual 11.0.1.$1 255.255.255.0
connmanctl scan wifi
WIFI_NAME=$( connmanctl services | grep -o 'naonet.*$' | grep -o 'wifi\w*managed_none' )
sudo connmanctl config $WIFI_NAME --ipv4 manual 192.168.1.$1 255.255.255.0
sudo connmanctl connect $WIFI_NAME
touch /home/nao/robocup.conf

# wait in case connmanctl needs a second to connect
(sleep 2 && reboot &)
