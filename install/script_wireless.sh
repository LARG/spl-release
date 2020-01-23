if [ -f /etc/wpa_supplicant.conf ]
  then
    rm /etc/wpa_supplicant.conf
fi
if [ ! -d /etc/wpa_supplicant ]
  then
    mkdir /etc/wpa_supplicant/
fi
mv wpa_supplicant.conf.tmp /etc/wpa_supplicant/wpa_supplicant.conf
if [ -f utwired.tmp ]
  then
    mv utwired.tmp /etc/init.d/utwired
    chmod +x /etc/init.d/utwired
elif [ -f utwiredstart.sh.tmp ]
  then
    mv utwiredstart.sh.tmp /home/nao/scripts/utwiredstart.sh
    chmod +x /home/nao/scripts/utwiredstart.sh
fi
if [ -f utwireless.tmp ]
  then
    mv utwireless.tmp /etc/init.d/utwireless
    chmod +x /etc/init.d/utwireless
elif [ -f utwirelessstart.sh.tmp ]
  then
    mv utwirelessstart.sh.tmp /home/nao/scripts/utwirelessstart.sh
    chmod +x /home/nao/scripts/utwirelessstart.sh
fi
if [ -e /etc/init.d/utwireless ]
  then
    /etc/init.d/utwireless stop
    /etc/init.d/utwireless start
fi
