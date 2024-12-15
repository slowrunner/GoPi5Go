#!/bin/bash

# ros2 topic echo --once --flow-style /color/yolov4_detections

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


while [ 1 ]
do
    result=$(ros2 topic echo --once --field detections /color/yolov4_detections | \
        cut -d',' -f4-5,7-100)
    echo -e "\n** detections: "
    echo $result | sed -e $'s/class_id/\\\n\\nclass_id/g' | sed -e $'s/),/\\\n/g' | grep class_id
    # sleep 1
done
