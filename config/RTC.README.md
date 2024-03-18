# Setting up Pi5 RTC

1) Enable battery charging  
- nano /boot/firmware/config.txt  
  add after [all]  
```
[all]

# enable RTC backup battery charge voltage
dtparam=rtc_bbat_vchg=3000000

```

2) Configure eeprom 
```
sudo -E rpi-eeprom-config --edit 

* Change 0 to 1:  

POWER_OFF_ON_HALT=1

* Add:
 
WAKE_ON_GPIO=0

* Finished eeprom configuration:

[all]
BOOT_UART=1
POWER_OFF_ON_HALT=1
BOOT_ORDER=0xf41
WAKE_ON_GPIO=0

```
