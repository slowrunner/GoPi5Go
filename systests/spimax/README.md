# Handling GoPiGo3 API 'NO SPI Response' Exception on Raspberry Pi5 with PiOS Bookworm 64-bit 

For some reason the GoPiGo3 SPI interface on Raspberry Pi 5 is prone to frequent early chip select release  
resulting in the GoPiGo3 not writing the SPI reply bytes in the transfer array.  This issue has not been  
seen running the GoPiGo3 API on Raspberry Pi 2, 3, 3B+, or Pi4, but [has been reported by others happening  
on the Pi4.](https://github.com/raspberrypi/linux/issues/5655)  

The current released GoPiGo3 API sets the SPI maximum transfer speed at 500000:  

```
    GPG_SPI.max_speed_hz = 500000
```
which results in a transfer speed of 250 MHz / 512 (2^9) = 488281 Hz.  

### The Work-Around 

The transfers appear to proceed without exception by setting this to the next lower transfer speed of 250000:

```
   GPG_SPI.max_speed_hz = 250000
```
which results in a transfer speed of 250 MHz / 1024 (2^10) = 244140 Hz.  

I created mygopigo3.py and rebuilt ~/Dexter/GoPiGo3/Software/Python  

```
cd ~/Dexter/GoPiGo3/Software/Python
modify setup.cfg to version 1.3.2.1.a
cp gopigo3.py to gopigo3.py.orig
cp mygopigo3.py to gopigo3.py
sudo python3 setup.py install 
```

At this reduced transfer rate, the GoPiGo3 can make a two byte SPI transfer in 0.0003 seconds,   
which means it can retrieve the left and right wheel encoder values roughly 1700 times per second     
- plenty fast enough for my ROS 2 Humble robot to publish encoders and the resultant odometry at 30 Hz.  

