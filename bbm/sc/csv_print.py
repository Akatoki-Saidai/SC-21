# csvの書き込み

import time
import copy

msg_types = ['monotonic', 'serious_error', 'error', 'warning', 'msg', 'phase', 'time', 'date', 'lat', 'lon', 'alt', 'alt_base_press', 'goal_lat', 'goal_lon', 'temp', 'press', 'camera_area', 'camera_order', 'camera_center_x', 'camera_center_y', 'camera_frame_size_x', 'camera_frame_size_y', 'motor_l', 'motor_r', 'goal_x', 'goal_y', 'accel_all_x', 'accel_all_y', 'accel_all_z', 'accel_line_x', 'accel_line_y', 'accel_line_z', 'mag_x', 'mag_y', 'mag_z', 'gyro_x', 'gyro_y', 'gyro_z', 'grav_x', 'grav_y', 'grav_z', 'euler_x', 'euler_y', 'euler_z']
DEFAULT_DICT = {x : '' for x in msg_types}

filename = 'csv_log_test.txt'

with open(filename, 'a') as f:
    f.write('\n\n\n\n' + ','.join(msg_types) + '\n')

def print(msg_type : str, msg_data):
    output_dict = copy.deepcopy(DEFAULT_DICT)
    if (msg_type == 'accel_all') or (msg_type == 'accel_line') or (msg_type == 'mag') or (msg_type == 'gyro') or (msg_type == 'grav') or(msg_type == 'euler'):
        output_dict[msg_type + '_x'] = str(msg_data[0])
        output_dict[msg_type + '_y'] = str(msg_data[1])
        output_dict[msg_type + '_z'] = str(msg_data[2])
    elif (msg_type == 'goal') or (msg_type == 'camera_center') or (msg_type == 'camera_frame_size'):
        output_dict[msg_type + '_x'] = str(msg_data[0])
        output_dict[msg_type + '_y'] = str(msg_data[1])
    elif (msg_type == 'motor'):
        output_dict[msg_type + '_l'] = str(msg_data[0])
        output_dict[msg_type + '_r'] = str(msg_data[1])
    else:
        output_dict[msg_type] = str(msg_data)
    output_dict['monotonic'] = str(time.monotonic_ns())
    output_msg = ','.join(output_dict.values())
    # print(output_msg)
    with open(filename, 'a') as f:
        f.write(output_msg + '\n')
