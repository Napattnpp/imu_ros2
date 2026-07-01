# Architectural Design Notes

This document explains **why** the `imu_ros2` framework is designed this way.

## 1. Library vs. ROS Separation
The drivers (`ICM20948Driver`, `BMI160Driver`) operate entirely on standard C++ types (`std::chrono`, `double` arrays) and return a `RawImuData` struct. They know nothing about ROS messages or parameters.

**Why?**
This makes the drivers re-usable outside of ROS (e.g., bare-metal Linux, other frameworks). It also drastically simplifies unit testing and benchmarking, as you do not need to spin up a ROS node to test byte-parsing logic.

## 2. No Orientation Estimation (REP-145)
This driver explicitly does **not** run a complementary or Madgwick filter. It publishes to `/imu/data_raw` with an orientation covariance of `-1.0` in the diagonal.

**Why?**
A hardware driver's only job is to extract data from the hardware as fast and accurately as possible. Estimating yaw/orientation requires tuning specific to the robot's dynamics and environment. It belongs in a dedicated node like `imu_filter_madgwick` or an EKF (`robot_localization`).

## 3. Midpoint Timestamping
Rather than pulling a timestamp *after* a multi-byte I²C read completes, the `I2cTransport` records `t_before` and `t_after` and calculates the midpoint.

**Why?**
I²C reads over Linux can suffer from jitter due to thread scheduling. If the OS preempts the thread during the read, stamping it at the end creates an artificially delayed timestamp, wrecking odometry synchronization. The midpoint is the best statistical estimate of when the sensor actually captured the data in its FIFO.

## 4. Why `I2cTransport` instead of `GenericTransport`
We consciously avoided abstracting the transport layer into generic `SPI` or `CAN` interfaces.

**Why?**
Over-engineering. We are targeting F1TENTH where I²C IMUs are standard. Adding speculative abstractions makes the code harder to read. If SPI is needed next year, it can be added then. Optimize for clarity.
