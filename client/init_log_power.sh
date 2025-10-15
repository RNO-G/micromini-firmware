#! /bin/bash

#initialize power logging
/home/rno-g/rno-g-BBB-scripts/gpio/select-ext-i2c.sh

if i2cdetect -y -r 2 | grep -q "UU"; then
    echo "DS2482 already initialized"
else
    /home/rno-g/rno-g-BBB-scripts/w1/setup-w1 
fi
