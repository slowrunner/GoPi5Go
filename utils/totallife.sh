#!/bin/bash
#
# totallife.sh    print total hours and sessions of life in life.log
#
# requires bc  (sudo apt-get install bc)
#
# Counted Keys:
#   Playtimes:           ": success "
#   Sessions:            "- boot -"              ( "\- boot \-" search string )
#   Safety shutdowns:    "SAFETY SHUTDOWN"
#
# Summed Keys:
#   Undocked for ... after __
#   Docking ... after __
#   execution:
#   nap for
#   h playtime
#   h charging
#   moved:          (odometer.log)

declare -i newBatteryAtCycle=416

echo "(Cleaning life.log first)"
/home/pi/GoPi5Go/plib/cleanlifelog.py
echo " "
fn="/home/pi/GoPi5Go/logs/life.log"
ofn='/home/pi/GoPi5Go/logs/odometer.log'
totalAwake=`(awk -F'execution:' '{sum+=$2}END{print sum;}' $fn)`
totalNaps=`(awk -F'nap for' '{sum+=$2}END{print sum;}' $fn)`
totalLife=`(echo "scale=1; ($totalAwake + $totalNaps)" | bc)`
# weirdness without +'' at end, see ls appended to lastDockingStr??
lastDockingStr=`(grep "h playtime" $fn | tail -1 )`+''
# echo "lastDockingStr: " $lastDockingStr " :"
lastUndockingStr=`(grep "h charging" $fn | tail -1)`
totalDockings=`(awk -F"Docking " '{sub(/ .*/,"",$2);print $2}' <<< $lastDockingStr)`
currentBattCycles=`(echo "scale=1; $totalDockings - $newBatteryAtCycle" | bc)`

echo "*** GoPi5Go Dave TOTAL LIFE STATISTICS ***"
echo "Total Awake:  " $totalAwake " hrs"
echo "Total Naps:    " $totalNaps " hrs"
echo "Total Life:   " $totalLife " hrs (since Mar 17, 2024)"
echo "GoPi5Go-Dave Playtimes (Undocked-Docked):" `(grep -c ": success" $fn)`
echo "Total Dockings: " $totalDockings
echo "New Battery Installed At Docking:" $newBatteryAtCycle
echo "This Battery At Cycle: " $currentBattCycles
last3playtimes=`(grep " h playtime" $fn | tail -3 | awk -F" after "  '{sum+=$2}END{print sum;}' )`
last3avePlaytime=`(echo "scale=1; $last3playtimes / 3" | bc)`
echo "Average playtime (last three)" $last3avePlaytime "hrs "
last3dockedtimes=`(grep "Undocking at " $fn | tail -3 | awk -F" after "  '{sum+=$2}END{print sum;}' )`
last3aveDockedtime=`(echo "scale=1; $last3dockedtimes / 3" | bc)`
echo "Average docked time (last three)" $last3aveDockedtime "hrs "
booted=`(grep -c "\- boot \-" $fn)`
echo "Sessions (boot): " `(grep -c "\- boot \-" $fn)`
aveSession=`(echo "scale=1; ($totalAwake / $booted)" | bc -l)`
echo "Average Session: " $aveSession "hrs"
safetyShutdowns=`(grep -c "SAFETY SHUTDOWN" $fn)`
echo "Safety Shutdowns: " $safetyShutdowns 
totalMoved=`(awk -F'moved:' '{sum+=$2}END{printf "%.1f", sum;}' $ofn)`
totalMovedFt=`(echo "scale=1; ($totalMoved / 0.3048)" | bc)`
echo "Total Travel: " $totalMoved "meters" $totalMovedFt "feet"
echo " "
echo "Last Undocking String: " $lastUndockingStr
echo "Last Docking   String: " $lastDockingStr
