# GoPi5Go-Dave non-ROS Docking Routines

```
dock.py    ./dock.py     tests docking.dock() to cause single docking maneuver (no logging)
undock.py  ./undock.py   tests docking.undock() to cause single undocking maneuver (no logging)

docking.py  (import docking)
                         docking.dock()   executes docking maneuver (no logging)
                         docking.undock() executes undock

pydocking.py (nohup_pydocking.sh)  imports docking.py
                         undocks (with logging) when charging current < 175ma
                         docks   (with logging) when battery voltage < 10.0v
```
