#!/bin/bash


dlist=$(ros2 topic echo --once --field detections /color/mobilenet_detections )
stripTo="ObjectHypothesis("
stripFrom="), pose"
end="${dlist#*${stripTo}}"
beg="${end%%${stripFrom}*}"
# echo -e $end
echo -e "\nOnly listing top result"
echo -e $beg
