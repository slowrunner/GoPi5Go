# GoPi5Go-Dave Navigation2

Adapted from turtlebot3_navigation2 for GoPiGo3 robot GoPi5Go-Dave



**GoPiGo3 Robot GoPi5Go-Dave Cannot Safely, Reliably Navigate In My Home Environment**

### GoPi5Go-Dave On The Shelf With Other "Reached Its Limit" Robots  
<img src="https://github.com/slowrunner/GoPi5Go/blob/main/Graphics/2024-11-14_Robots_On_The_Shelf.jpg" width="500" />  
WALL-E from 2023 Create3-WaLi robot, GoPi5Go-Dave built in 2018, and RugWarriorPro robot built in 2000  

### GoPi5Go-Dave View Of Environment Is 45 Degrees Off  
<img src="https://github.com/slowrunner/GoPi5Go/blob/main/Graphics/Dave_45_Deg_Off_Reality.jpg" width="500" />  

### GoPi5Go-Dave Got Confused and Decided To Woo Some Chair Legs
<img src="https://github.com/slowrunner/GoPi5Go/blob/main/Graphics/Dave_Not_Nav_Out_Of_Paper_Bag.jpg" width="500" />  
It was going well on the global path planned, but apparently the local planner decided Dave was too close to a "pop-up" chair leg.  
Perhaps in this case an increased inflation value might work, but would cause more problem than it solves in other areas.  

### GoPi5Go-Dave Cannot Accurately Return To His Dock 
<img src="https://github.com/slowrunner/GoPi5Go/blob/main/Graphics/Dave_Nav_Does_Not_End_Well.jpg" width="500" />  

## Seven Years Invested In GoPiGo3 Robots  
I am done.  Can't do it anymore.  
- The odometry is just not accurate enough.  
- The PID control does not drive straight.  
- Turns are not accurate.  
- The tires are not "grippy".  
- The encoder API is whole degree resolution.  
- The wheelbase is too narrow.
- The vendor stopped maintaining drivers two years ago.
- Vendor has no ROS 2 support  
- No other ROS 2 GoPiGo3 robots to learn from.  
