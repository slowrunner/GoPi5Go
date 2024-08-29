# GoPi5Go-Dave ROS 2 Workspace


### Base GoPi5Go-Dave Processor Utilization

```
top - 11:00:17 up 13:16,  0 users,  load average: 0.21, 0.26, 0.27
Tasks:  23 total,   1 running,  22 sleeping,   0 stopped,   0 zombie
%Cpu(s):  3.7 us,  0.4 sy,  0.0 ni, 95.8 id,  0.0 wa,  0.0 hi,  0.1 si,  0.0 st
MiB Mem :   4045.1 total,   1488.8 free,    975.2 used,   1581.1 buff/cache
MiB Swap:    200.0 total,    200.0 free,      0.0 used.   2981.9 avail Mem 

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND                                                                                                            
    812 pi        20   0  772912  67072  29184 S   8.3   1.6   8:42.53 gopigo3_node                                                                                                       
    131 pi        20   0  764768  57856  26624 S   3.7   1.4   4:13.18 odometer                                                                                                           
    653 pi        20   0  659200  28672  19968 S   0.7   0.7   0:48.89 joy_node                                                                                                           
    132 pi        20   0 1261504  43008  19968 S   0.3   1.0   2:53.94 dave_node                                                                                                          
    133 pi        20   0  894656  44032  19968 S   0.3   1.1   2:24.04 docking_node                                                                                                       
    135 pi        20   0  763680  57344  26624 S   0.3   1.4   3:42.78 battery_node                                                                                                       
    187 pi        20   0  570544  22016  15360 S   0.3   0.5   1:10.00 teleop_node            

```

Base GoPi5Go-Dave Processor Utilization:  
- Total Load (15 min uptime / 4 cores): 7%   
- Instantaneous Load (sum of top %CPU): 14%  

