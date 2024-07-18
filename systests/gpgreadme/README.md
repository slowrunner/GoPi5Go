# GoPiGo3 [![Documentation Status](https://readthedocs.org/projects/gopigo3/badge/?version=master)](http://gopigo3.readthedocs.io/en/latest/?badge=master)

The GoPiGo3 is a delightful and complete robot for the Raspberry Pi that turns your Pi into a fully operating robot.  GoPiGo3 is a mobile robotic platform for the Raspberry Pi first developed by Dexter Industries, and acquired by [Modular Robotics](https://modrobotics.com). See [GoPiGo.io.](https://GoPiGo.io)

![ GoPiGo3 Raspberry Pi Robot ](https://raw.githubusercontent.com/DexterInd/GoPiGo3/master/GoPiGo3_Raspberry_Pi_Robot.jpg)
# GoPiGo OS

GoPiGo OS is an easy to get started with, all packaged OS for the GoPiGo3. It offers Bloxter, and JupyterLab 2.  You can [download it](https://gopigo.io/downloads/gopigo_os) and install it on your SD card.
### Note:  GoPiGo OS allows you to SSH into the robot and have full access to Linux. You can also use VNC viewer and access the Raspberry Pi Desktop.


# Alternately: GoPiGo3 API Installed Over 32-bit Stable Legacy Raspberry Pi OS

- The following installation will only work with the [stable legacy 32-bit Bullseye Pi OS versions available here](https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-legacy)
### Note: the Pi user must exist while installing the drivers and examples, but you can remove this user afterwards if you want to secure your robot.
 
# Quick Install

In order to quick install the `GoPiGo3` repository, open up a terminal and type the following commands:
```
curl -kL dexterindustries.com/update_gopigo3 | bash
curl -kL dexterindustries.com/update_sensors | bash
sudo reboot
```

The same commands can be used for updating the `GoPiGo3` to the latest version.

# Virtual Environment Installation

```
curl -kL dexterindustries.com/update_gopigo3 | bash -s -- --user-local --bypass-gui-installation
curl -kL dexterindustries.com/update_sensors | bash -s -- --user-local --bypass-gui-installation
```




# License

Please review the [LICENSE.md] file for license information.

[LICENSE.md]: ./LICENSE.md

# See Also

- [Modular Robotics](https://modrobotics.com)
- [GoPiGo.io](https://gopigo.io)
- [Forum Support](https://forum.dexterindustries.com/c/gopigo)
