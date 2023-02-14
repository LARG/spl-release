#/bin/sh
if [ $1 = "start" ]; then
    ~/bin/villa-start.sh
elif [ $1 = "stop" ]; then
    ~/bin/villa-stop.sh
elif [ $1 = "restart" ]; then
    ~/bin/villa-stop.sh
    sleep 2
    ~/bin/villa-start.sh
elif [ $1 = "status" ]; then
    systemctl --user status ut_motion
    systemctl --user status ut_vision
elif [ $1 = "connect" ]; then
    WIFI_NAME=$( connmanctl services | grep -o "$2 .*$" | grep -o 'wifi\w*managed_\(none\|psk\)$' )
    if [ -z "$WIFI_NAME" ]; then
        echo "SSID not found. Check spelling. Try 'connmanctl scan wifi'?"
    else
        ROBOT_ID=$( cat /home/nao/data/config.yaml | grep -o "robot_id:.*$" | grep -o "[0-9]*" )
        echo "Connecting to $WIFI_NAME"
        sudo mkdir -p /var/lib/connman/$WIFI_NAME
        sudo printf "[$WIFI_NAME]\nType=wifi\nName=$2\nSecurity=wpa\nPassphrase=Nao?!Nao?!\nIPv4=10.0.1.$ROBOT_ID/255.255.0.0\nIPv6=off\n" | sudo tee /var/lib/connman/$WIFI_NAME/settings
        sudo connmanctl connect $WIFI_NAME
        # Retry config just in case it's not set
        sudo connmanctl config $WIFI_NAME --ipv4 manual 10.0.1.$ROBOT_ID 255.255.0.0
        sudo connmanctl config $WIFI_NAME autoconnect on
    fi
elif [ $1 = "disconnect" ]; then
    WIFI_NAME=$( connmanctl services | grep -o "$2.*$" | grep -o 'wifi\w*managed_\(none\|psk\)' )
    sudo connmanctl config $WIFI_NAME autoconnect off
    echo "Attempting disconnect $WIFI_NAME"
    sudo connmanctl disconnect $WIFI_NAME
else
    echo "Invalid command"
fi
