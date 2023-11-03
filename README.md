# detector_ros
Object Detector (ultralytics) for ROS2
# Parameters:
- `show_bbox_image`: Show the image with bounding boxes (default: `True`)
- `dps`: Detections per second (default: `15`)
- `weights`: Model to use (default: `yolov8n.pt`)
- `conf`: Confidence threshold (default: `0.25`)
- `iou`: IoU threshold (default: `0.45`)
- `max_det`: Maximum detections (default: `300`)
- `image_topic`: Subscribed image topic (default: `image_raw`)
- `bbox_result_topic`: Bounding box result publisher topic (default: `image_raw/bbox/result`)
- `bbox_image_topic`: Bounding box image publisher topic (default: `image_raw/bbox/image`)