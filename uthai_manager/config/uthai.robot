[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE   | DEFAULT JOINT
/dev/ttyUSB0 | 1000000    | r_hip_yaw
/dev/ttyUSB1 | 1000000    | STM-32

[ device info ]
# TYPE    | PORT NAME    | ID  | MODEL          | PROTOCOL | DEV NAME      | BULK READ ITEMS
dynamixel | /dev/ttyUSB0 | 1   | EX-106+        | 1.0      | r_hip_yaw     | present_position
dynamixel | /dev/ttyUSB0 | 11  | EX-106+        | 1.0      | l_hip_yaw     | present_position
dynamixel | /dev/ttyUSB0 | 2   | EX-106+        | 1.0      | r_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 12  | EX-106+        | 1.0      | l_hip_roll    | present_position
dynamixel | /dev/ttyUSB0 | 3   | EX-106+        | 1.0      | r_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 13  | EX-106+        | 1.0      | l_hip_pitch   | present_position
dynamixel | /dev/ttyUSB0 | 4   | EX-106+        | 1.0      | r_knee_pitch  | present_position
dynamixel | /dev/ttyUSB0 | 14  | EX-106+        | 1.0      | l_knee_pitch  | present_position
dynamixel | /dev/ttyUSB0 | 5   | EX-106+        | 1.0      | r_ankle_pitch | present_position
dynamixel | /dev/ttyUSB0 | 15  | EX-106+        | 1.0      | l_ankle_pitch | present_position
dynamixel | /dev/ttyUSB0 | 6   | EX-106+        | 1.0      | r_ankle_roll  | present_position
dynamixel | /dev/ttyUSB0 | 16  | EX-106+        | 1.0      | l_ankle_roll  | present_position
sensor    | /dev/ttyUSB1 | 200 | STM-32         | 1.0      | STM-32        | gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
