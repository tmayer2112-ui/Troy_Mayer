| State (RMSE) | IMU-only | Contact-aided | Contact-aided vs IMU-only |
|---|---:|---:|---:|
| Velocity, world 3D (m/s) | 1.38 | 0.0108 | 128× better |
| Velocity x, world (m/s) | 1.28 | 0.00551 | 232× better |
| Velocity y, world (m/s) | 0.514 | 0.0079 | 65× better |
| Velocity z, world (m/s) | 0.0125 | 0.00491 | 2.5× better |
| Velocity, body 3D (m/s) | 1.38 | 0.0102 | 135× better |
| Roll (deg) | 0.226 | 0.144 | 1.6× better |
| Pitch (deg) | 0.195 | 0.564 | **2.9× worse** |
| Yaw (deg) | 0.428 | 0.279 | 1.5× better |
| Position x (m) | 28.64 | 0.0463 | 619× better |
| Position y (m) | 10.67 | 0.0259 | 411× better |
| Position z (m) | 0.367 | 0.0676 | 5.4× better |
| Accel bias x (m/s²) | 0.025 | 0.0956 | **3.8× worse** |
| Accel bias y (m/s²) | 0.026 | 0.0244 | 1.1× better |
| Accel bias z (m/s²) | 0.000505 | 0.000863 | **1.7× worse** |
| Gyro bias x (deg/s) | 0.00712 | 0.00426 | 1.7× better |
| Gyro bias y (deg/s) | 0.0116 | 0.00366 | 3.2× better |
| Gyro bias z (deg/s) | 0.0103 | 0.00808 | 1.3× better |

60 s walk, sensor seed 0, 1 kHz. RMSE over the full window. Distance walked 13.1 m. Final position error: IMU-only 68.9 m, contact-aided 0.15 m (0.13 m horizontal). Final yaw error: IMU-only +0.72°, contact-aided +0.11°.

Monte Carlo, 10 sensor-noise seeds on the same walk: contact-aided velocity RMSE 0.0128 ± 0.0019 m/s (min 0.0108, max 0.0162).
