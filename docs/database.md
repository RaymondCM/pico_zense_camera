# Database Documentation

## 	Format

```json
{
    "_id": "",
    "context": string,
    "doc_datetime": datetime,
    "ros_reftime": float,
    "camera_data": realsense_sensors,
    "environment": environment_meta,
    "localisation": thorvald_localisation,
}
```

### 	Custom Types

#### Realsense D4XX Sensor

##### Sensor Data
```json
{
    "realsense_sensors": {
        "camera_name": string,
        "camera_serial": string,
        "rgb": sensor_image,
        "depth": sensor_image,
        "aligned_depth_to_rgb": sensor_image,
        "aligned_depth_to_ir_left": sensor_image,
        "ir_left": sensor_image,
        "ir_right": sensor_image,
        "depth_to_rgb_transform": extrinsics_meta,
        "depth_to_ir_left_transform": extrinsics_meta,
        "depth_to_ir_right_transform": extrinsics_meta,
    },
    "sensor_image": {
        "data": {binary},
        "ros_time": float,
        "frameid": string,
        "height": int,
        "width": int,
        "intrinsics": intrinsics_dt,
    },
}

```

##### Intrinsic Meta
Based on [sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html).

```json
"intrinsics_meta": {
	"distortion_model": string,
	"distortion_parameters": [float],
	"intrinsic_matrix": [float],
	"rectification_matrix": [float],
	"projection_matrix": [float]
},
```

##### Extrinsic Meta
Based on [realsense2_camera/Extrinsics](http://docs.ros.org/kinetic/api/realsense2_camera/html/msg/Extrinsics.html)

```json
"extrinsics_meta": {
	"rotation": [float],
	"translation": [float],
},
```

#### Thorvald Localisation

```json
"thorvald_localisation": {
	"current_node": int,
	"current_edge": int,
	"closest_node": int,
	"laser_scan": laser_scan,
	"robot_pose": pose,
	"amcl": covariance_pose
},
```

##### Laser Scan
```json
"laser_scan": {
        "acquisition_time": float,
        "frame_id": string,
        "angle_min": float,
        "angle_max": float,
        "angle_increment": float,
        "time_increment": float,
        "scan_time": float,
        "range_min": float,
        "range_max": float,
        "ranges": [float],
        "intensities": [float],
},
```

##### Covariance Pose
```json
"covariance_pose": {
	"pose": pose,
	"covariance": covariance,
},
```

##### Pose
```json
"pose": {
	"position": position,
	"orientation": orientation,
},
```

###### Position
```json
"position": {
	"x": float,
	"y": float,
	"z": float,
},
```

###### Orientation
```json
"orientation": {
	"x": float,
	"y": float,
	"z": float,
	"w": float,
},
```

#### Environment Meta
```json
"environment_meta": {
	"temperature": float,
	"pressure": float,
	"humidity": float,
	"air_quality": float,
},
```
