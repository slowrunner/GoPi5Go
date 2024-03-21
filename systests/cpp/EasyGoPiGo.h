/* FILE: EasyGoPiGo.h

   Instance Vars:
     speed: default 150 DPS
     left_eye_color
     right_eye_color
     left_encoder_target
     right_encoder_target

   Implemented Methods:

   TODO Methods:
     set_speed(speed_in_DPS=150)
     get_speed()
     forward(): drive forward - use set_speed() or default: 150 DPS
     backward(): drive backward
     stop(): 
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
*/
