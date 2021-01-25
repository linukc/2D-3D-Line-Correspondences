**Requirements**:
- ply point cloud [0]
- txt 3d lines [0]
- GT [0]
- camera.distortion_parameters k1 k2 r1 r2 [1, 2]
- camera.rojection_parameters cx cy fx fy [1, 2]
- camera.initialRotation [1]
- camera.initialTranslation [1]
- imu2camera.rotation&translation [2]
- imu.accelerometer_measurement_noise_standard_deviation [2]
- imu.gyroscope_measurement_noise_standard_deviation [2]
- imu.accelerometer_bias_random_walk_noise_standard_deviation [2]
- imu.gyroscope_bias_random_walk_noise_standard_deviation [2]

---
**Steps**:
1. Create your custom launch files like for realsense dataset and change them:
    - map_fusion/launch [0]
    - VINS-Mono/vin_estimator/launch
2. Create your custom config filse like for realsense dataset and change them:
    - map_fusion/config [1]
    - VINS-Mono/config [2]
    - afm/scripts/experiments/afm_unet_rs.yaml
3. Run
    - roslaunch vins_estimator <your>.launch 
    - roslaunch map_fusion <your>.launch

Additional look at VINS-Mono/vins_estimator/src/parameters.h/focal_length
