# This script contains all the transformation matrices necessary to convert the IMU frame (for each particular IMU)to the drone frame (x=forward, y=right, z=down)
# P.B. 2025
import numpy as np

# Standard IMU frame: 
# x=left, y=forward, z=down (matches blackbird IMU frame), where these directions are from the perspective of the drone body
# [ LEFT, FORWARD, DOWN ]
# i.e. x on the IMU points to the left side of the drone body

# This script allows you to transform all our IMUs into this standard frame


############# CHIPSET 1 #############

## (red) LSM6DS0 IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points forward, Y points left, Z points up.

# Transformation matrix from IMU frame to drone frame:
R_LSM6DS0_to_standard = np.array([[0, 1, 0],  # imu's Y becomes LEFT
                                   [1, 0, 0],  # imu's X becomes FORWARD
                                   [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_LSM6DS0 = np.array([[0, -1, 0],  # standard's LEFT becomes IMU's Y
                                   [-1, 0, 0],  # standard's FORWARD becomes IMU's X
                                   [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


## (red) BMI270 IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points left, Y points back, Z points up

# Transformation matrix from IMU frame to drone frame:
R_BMI270_to_standard = np.array([[1, 0, 0], # imu's X becomes LEFT
                                  [0, -1, 0], # imu's -Y becomes FORWARD
                                  [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_BMI270 = np.array([[-1, 0, 0], # standard's LEFT becomes IMU's X
                                  [0, 1, 0], # standard's FORWARD becomes IMU's -Y
                                  [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


############# CHIPSET 2 #############

## (purple) GY-BN008X IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points left, Y points back, Z points up

# Transformation matrix from IMU frame to drone frame:
R_GYBN008X_to_standard = np.array([[1, 0, 0], # imu's X becomes LEFT
                                  [0, -1, 0], # imu's -Y becomes FORWARD
                                  [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_GYBN008X = np.array([[-1, 0, 0], # standard's LEFT becomes IMU's X
                                    [0, 1, 0], # standard's FORWARD becomes IMU's -Y
                                    [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


## (blue) HC ITG/MPU IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points forward, Y points left, Z points up

# Transformation matrix from IMU frame to drone frame:
R_HCITGMPU_to_standard = np.array([[0, 1, 0], # imu's Y becomes LEFT
                                    [1, 0, 0], # imu's X becomes FORWARD
                                    [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_HCITGMPU = np.array([[0, -1, 0], # standard's LEFT becomes IMU's Y
                                    [-1, 0, 0], # standard's FORWARD becomes IMU's X
                                    [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


############# CHIPSET 3 #############

## (purple) GY-ICM20948V2 IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points forward, Y points left, Z points up

# Transformation matrix from IMU frame to drone frame:
R_GYICM20948V2_to_standard = np.array([[0, 1, 0], # imu's Y becomes LEFT
                                        [1, 0, 0], # imu's X becomes FORWARD
                                        [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_GYICM20948V2 = np.array([[0, -1, 0], # standard's LEFT becomes IMU's Y
                                        [-1, 0, 0], # standard's FORWARD becomes IMU's X
                                        [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


# WARNING: I am doing this based on how the chip looks similar to the mounted chip's top (it's soldered into place so I can't verify from the bottom)
## (tiny. blue) ICM42688 IMU. attach with sharpie forward arrow pointing to front of drone body.
# X points left, Y points back, Z points up

# Transformation matrix from IMU frame to drone frame:
R_ICM42688_to_standard = np.array([[1, 0, 0], # imu's X becomes LEFT
                                    [0, -1, 0], # imu's -Y becomes FORWARD
                                    [0, 0, -1]]) # imu's -Z becomes DOWN

# Transformation matrix from standard frame to IMU frame:
R_standard_to_ICM42688 = np.array([[-1, 0, 0], # standard's LEFT becomes IMU's X
                                    [0, 1, 0], # standard's FORWARD becomes IMU's -Y
                                    [0, 0, 1]]) # standard's DOWN becomes IMU's -Z


