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

Problem with our data: VINS and imu
May be look for:
1. IMU acc and gyr parameters

    https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/296#issuecomment-553896144
    https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model#the-noise-model-parameters-in-kalibr
    https://github.com/gaowenliang/imu_utils
    https://www.vectornav.com/resources/imu-specifications
    [imu noize](https://docviewer.yandex.ru/view/1130000039377681/?page=3&*=UTJqeihfY%2Bux98hDF4mbnMkbDml7InVybCI6Imh0dHBzOi8vYXJ4aXYub3JnL3BkZi8xNjA4LjA3MDUzLnBkZiIsInRpdGxlIjoiMTYwOC4wNzA1My5wZGYiLCJub2lmcmFtZSI6dHJ1ZSwidWlkIjoiMTEzMDAwMDAzOTM3NzY4MSIsInRzIjoxNjExNTA2NTU3OTY1LCJ5dSI6IjY4NjA3OTU2MzE2MDYyNTQ1MjEiLCJzZXJwUGFyYW1zIjoibGFuZz1lbiZ0bT0xNjExNTA2NTQ5JnRsZD1ydSZuYW1lPTE2MDguMDcwNTMucGRmJnRleHQ9YWNjZWxlcm9tZXRlcittZWFzdXJlbWVudCtub2lzZStzdGFuZGFyZCtkZXZpYXRpb24mdXJsPWh0dHBzJTNBLy9hcnhpdi5vcmcvcGRmLzE2MDguMDcwNTMucGRmJmxyPTEwODQ5Jm1pbWU9cGRmJmwxMG49cnUmc2lnbj03Y2M2MDM0NTVlY2JlYjE3NmE3YmQ2NzJkNTlkNjU5MSZrZXlubz0wIn0%3D&lang=en)

2. Constant speed https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/155#issue-295492833 (#issuecomment-400685206) -> VinsFusion
3. td https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/232
    
    
    
