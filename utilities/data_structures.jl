struct drone_data
    pitch_control :: NTuple{2,Float32};
    roll_control :: NTuple{2,Float32};
    yaw_control :: NTuple{2,Float32};
    motor_speeds :: NTuple{4,UInt16};
    radio_control :: NTuple{4,Float32};
    acc :: NTuple{3,Float32};
    gyro :: NTuple{3,Float32};
    pry :: NTuple{3,Float32};
    switches :: NTuple{4,UInt8};
end