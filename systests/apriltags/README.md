# AprilTags For ROS 2 GoPi5Go-Dave Robot


REF:  https://github.com/AprilRobotics/apriltag

For ArUco compatibility use tag36h11 family

Most folks should use TagStandard41h12 family

```
mkdir systests/apriltags
git clone https://github.com/AprilRobotics/apriltag-imgs.git
cp apriltag-imgs/tagStandard41h12/tag41_12_00005.png .
cp apriltag-imgs/tag36h11/tag36_11_00005.png .
cp apriltag-imgs/tag_to_svg.py  .
chmod +x tag_to_svg.py

./tag_to_svg.py tag41_12_00005.png tag41_12_00005.svg --size=150
./tag_to_svg.py tag36_11_00005.png tag36_11_00005.svg --size=150

```
