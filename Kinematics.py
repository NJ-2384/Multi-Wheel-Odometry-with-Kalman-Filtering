import math as m


# Robot Metrics
WHEEL_RADIUS = 5 # Radius of each wheel in cm
BASE_LENGTH = 30 # distance between wheels in cm
MAX_PULSES_PER_REV = 960 # Maximum encoder pulses per revolution



def ForwardKinematics(old_encoder_data, new_encoder_data, prev_robot_pos, time_step):
    ''' 
    Convert encoder pulse differences to position and velocity changes
    
    Parameters:
    old_encoder_data (tuple): previous encoder readings (front_left, back_left, front_right, back_right)
    new_encoder_data (tuple): current encoder readings (front_left, back_left, front_right, back_right)
    prev_robot_pos (tuple): previous robot position (x, y, theta)
    time_step (float): time elapsed between readings
    
    Returns:
    tuple: (x_measured, y_measured, theta_measured, velocity_x, velocity_y, omega)
    '''
    
    x_robot, y_robot, theta_robot = prev_robot_pos
    
    delta_encoder_data = []
    new_sign = True
    old_sign = True
    
    for pos in range(len(new_encoder_data)):
        
        if new_encoder_data[pos] > 0:
            new_sign = True
        else:
            False
            
        if old_encoder_data[pos] > 0:
            old_sign = True
        else:
            False
        
        if (new_sign and old_sign):
            if (abs(new_encoder_data[pos]) > abs(old_encoder_data[pos])):
                delta = ((new_encoder_data[pos])) - (old_encoder_data[pos])
            else:
                delta = (MAX_PULSES_PER_REV - (old_encoder_data[pos])) + ((new_encoder_data[pos]))
                
        if (not new_sign and not old_sign):
            if (abs(new_encoder_data[pos]) > abs(old_encoder_data[pos])):
                delta = ((new_encoder_data[pos])) - (old_encoder_data[pos])
            else:
                delta = (abs(old_encoder_data[pos])) - MAX_PULSES_PER_REV + (abs(new_encoder_data[pos]))
                
        if (not new_sign and old_sign):
            delta = ((new_encoder_data[pos]))
                
        if (new_sign and old_sign):
                delta = ((new_encoder_data[pos]))

        delta_encoder_data.append(delta)
    
    delta_front_left, delta_back_left, delta_front_right, delta_back_right = delta_encoder_data
    
    # Average the pulses from left and right wheels
    avg_left_wheel_delta = (delta_front_left + delta_back_left) / 2
    avg_right_wheel_delta = (delta_front_right + delta_back_right) / 2
    
    # Number of revolutions
    left_wheel_revs = avg_left_wheel_delta / MAX_PULSES_PER_REV
    right_wheel_revs = avg_right_wheel_delta / MAX_PULSES_PER_REV
    
    # Convert to distance
    left_wheel_distance = left_wheel_revs * (2 * m.pi * WHEEL_RADIUS)
    right_wheel_distance = right_wheel_revs * (2 * m.pi * WHEEL_RADIUS)

    # Calculate change in position and orientation
    delta_distance = ((right_wheel_distance + left_wheel_distance) / 2)
    delta_theta = ((right_wheel_distance - left_wheel_distance) / BASE_LENGTH)

    theta = theta_robot + (delta_theta / 2)

    delta_x = delta_distance * m.cos(theta)
    delta_y = delta_distance * m.sin(theta)
    
    # New measured position
    x_measured = delta_x + x_robot
    y_measured = delta_y + y_robot
    theta_measured = delta_theta + theta_robot
    
    # Normalize theta to [0, 360) degrees
    if (theta_measured >= 360):
        theta_measured -= 360
    elif (theta_measured <= -360):
        theta_measured += 360

    # Calculate velocities
    velocity_x = delta_x / time_step
    velocity_y = delta_y / time_step
    omega = delta_theta / time_step
    
    # Return measured position and velocities
    return (x_measured, y_measured, theta_measured, velocity_x, velocity_y, omega)