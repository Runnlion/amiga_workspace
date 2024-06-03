 $ roslaunch rtabmap_launch rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera_right/depth/image_raw \
    rgb_topic:=/camera_right/color/image_raw \
    camera_info_topic:=/camera_right/color/camera_info \
    approx_sync:=false
