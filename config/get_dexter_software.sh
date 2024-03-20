#!/bin/bash

echo -e "Get Dexter/GoPiGo3/"
git clone http://www.github.com/DexterInd/GoPiGo3.git /home/pi/Dexter/GoPiGo3
echo -e "Get Dexter/DI_Sensors/"
git clone https://github.com/DexterInd/DI_Sensors.git /home/pi/Dexter/DI_Sensors
echo -e "Get Dexter/RFR_Tools/"
git clone https://github.com/DexterInd/RFR_Tools.git /home/pi/Dexter/lib/Dexter/RFR_Tools
cp /home/pi/Dexter/GoPiGo3/Install/list_of_serial_numbers.pkl /home/pi/Dexter/.list_of_serial_numbers.pkl
}
