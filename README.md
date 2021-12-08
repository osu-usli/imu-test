# imu-test

A simple test of the MPU-6050 IMU, reading accelerometer & gyroscope input and performing sensor fusion to determine the vehicle's attitude.

This program is designed to run on an APM 2.6 flight control baord. To compile, install [Platformio](https://platformio.org/) and run `pio run` in the project directory. Upload to a connected board with `pio run -t upload`.
