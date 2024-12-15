#!/bin/bash

# ros2 topic echo --once --flow-style /color/mobilenet_detections 

# 0            "background",
# 1           "aeroplane",
# 2           "bicycle",
# 3           "bird",
# 4           "boat",
# 5           "bottle",
# 6           "bus",
# 7           "car",
# 8           "cat",
# 9           "chair",
# 10           "cow",
# 11           "diningtable",
# 12           "dog",
# 13           "horse",
# 14           "motorbike",
# 15           "person",
# 16           "pottedplant",
# 17           "sheep",
# 18           "sofa",
# 19           "train",
# 20          "tvmonitor"




while [ 1 ]
do
    result=$(ros2 topic echo --once --field detections /color/mobilenet_detections | \
        cut -d',' -f4-5,7-100)
    echo -e "\n** detections: "
    echo $result | sed -e $'s/class_id/\\\n\\nclass_id/g' | sed -e $'s/),/\\\n/g' | grep class_id
    # sleep 1
done
