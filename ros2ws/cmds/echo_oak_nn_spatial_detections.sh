#!/bin/bash

# ros2 topic echo --once --flow-style /oak/nn/spatial_detections 

# 0    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
# 7    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
# 14   "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
# 21   "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
# 28   "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
# 35   "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
# 42   "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
# 49   "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
# 56   "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
# 63   "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
# 70   "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
# 77   "teddy bear",     "hair drier", "toothbrush"


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
    result=$(ros2 topic echo --once --field detections /oak/nn/spatial_detections | \
        cut -d',' -f4-5,7-100)
    echo -e "\n** detections: "
    echo $result | sed -e $'s/class_id/\\\n\\nclass_id/g' | sed -e $'s/),/\\\n/g' | grep class_id
    # sleep 1
done
