# MadgwickAHRS

提供姿态和航向参考系统（AHRS）功能的模块 / A module providing Attitude and Heading Reference System (AHRS) functionality

## 硬件需求 / Required Hardware

ramfs

## 构造参数 / Constructor Arguments

* beta:                  0.05
* gyro\_topic\_name:       "imu\_gyro"
* accl\_topic\_name:       "imu\_accl"
* quaternion\_topic\_name: "ahrs\_quaternion"
* euler\_topic\_name:      "ahrs\_euler"
* task\_stack\_depth:      2048

## 依赖 / Depends

无（No dependencies）
