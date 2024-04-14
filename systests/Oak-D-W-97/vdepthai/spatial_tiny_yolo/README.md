# Spatial Tiny YOLO v4 on Rapsberry Pi5 with Oak-D-W-97


- Install  (Get depthai-python repository  to get Tiny-YOLOv4 blob)

```
cd vdepthai
source bin/activate
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python/examples
python3 install_requirements.py
mkdir -p ~/GoPi5Go/models/depthai
cp /home/pi/GoPi5Go/systests/Oak-D-W-97/vdepthai/depthai-python/examples/models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob ~/GoPi5Go/models/depthai

```

### Spatial Tiny YOLO v4 Performance on PiOS Bookworm 64-bit Desktop with Raspberry Pi 5 4GB (with Pi5 Cooler)


- spatial_tiny_yolo.py:     25-30FPS  uptime 5m load: 0.5 Temp 49degC  Clock Freq 1.6GHz  No Throttling
  (416x416 pixel tiny yolo v4 object recognition with depth active - console output only)

- spatial_tiny_yolo.py -d:  25FPS     uptime 5m load: 2.0 Temp 58degC  Clock Freq 2.4GHz  No Throttling
  (416x416 pixel tiny yolo v4 display with 1280x800 depth display and console out)

- gui_spatial_tiny_yolo.py: 25FPS     uptime 5m load: 1.3 Temp 56degC  Clock Freq 2.4GHz  No Throttling
  (416x416 pixel tiny yolo v4 display and console out - depth active but no separate depth display)
