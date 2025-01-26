# An Attempt to Use ChatGPT to translate Python to C++ for the EasyGoPiGo3 GoPiGo3 API

The Dexter/ModularRobotics GoPiGo3 robot includes a C++ API in GoPiGo3.h/cpp and libgopigo3.so  
but does not include an "Easy API" that is available in Python.

This project is a rough implementation of an EasyGoPiGo3 class which includes:

Implemented Methods (by Alan):
```
     set_speed(speed_in_DPS)
     set_speed()  // DEFAULT_SPEED
     int get_speed()
     get_speed(&out)
     float volt()
     forward(): drive forward - use set_speed() or default: 150 DPS
     backward(): drive backward
     stop(): 
```

Attempted implementation by ChatGPT (Does not compile):
```
     target_reached(left_tgt_degrees, right_tgt_degrees):  use to detect when to stop forward(), backward(), right(), left(), spin_right(), spin_left()
                                                           and for non-blocking drive_cm() or drive_inches()
```


TODO Methods:
```
     drive_cm(dist_cm, blocking=true)
     drive_inches(dist_inches, blocking=true)
     right() : pivot cw around right wheel
     left():  pivot ccw around left wheel
     spin_right(): spin in place clockwise
     spin_left(): spin in place counter-clockwise
     target_reached(left_tgt_degrees, right_tgt_degrees):  use to detect when to stop forward(), backward(), right(), left(), spin_right(), spin_left()
                                                           and for non-blocking drive_cm() or drive_inches()
     reset_encoders(): resets both encoders to 0
     read_encoders(out:left, out:right, in:units=CM/INCH/DEGREE)
     read_encoders_average(out:ave, in:units=CM/INCH/DEGREE)
     turn_degrees(in:deg, blocking=true):  left: negative degrees
     blinker_on(id:{LEFT,RIGHT}
     set_left_eye_color(R,G,B)
     set_right_eye_color(R,G,B)
     set_eye_color(R,G,B)
     open_left_eye()
     open_right_eye()
     open_eyes()
     close_left_eye()
     close_right_eye()
     close_eyes()
```

LIMITATIONS and DIFFERENCES:
- i2c mutex not implemented
- does not perform sensor discovery / sensor init
- DEFAULT_SPEED is set to 150 DPS rather than 300 DPS for additional safety from tipping over
- Compiler enforces speed value type


### Setup:  
- sudo apt install cmake  
- cmake CMakeLists.txt  
- make  
- [make install] will put:  
  - /usr/local/include/GoPiGo3.h  
  - /usr/local/lib/libgopigo3.so  

### Other Useful Commands:  
- ```make clean``` will remove all executables from build/  
- ```make``` will rebuild any changed project  
