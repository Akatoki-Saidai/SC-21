# csvの書き込み

import time
import copy

msg_types = ['monotonic', 'error', 'warning', 'msg', 'fase', 'alt', 'temp', 'press', 'accel_all_x', 'accel_all_y', 'accel_all_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z', 'gyro_x', 'gyro_y', 'gyro_z', 'time', 'date']
DEFAULT_DICT = {x : '' for x in msg_types}

filename = 'csv_log_test.txt'

with open(filename, 'a') as f:
    f.write('\n\n\n\n' + ','.join(msg_types) + '\n')

def print(msg_type : str, msg_data):
    output_dict = copy.deepcopy(DEFAULT_DICT)
    if (msg_type == 'accel_all') or (msg_type == 'accel') or (msg_type == 'mag') or (msg_type == 'gyro'):
        output_dict[msg_type + '_x'] = str(msg_data[0])
        output_dict[msg_type + '_y'] = str(msg_data[1])
        output_dict[msg_type + '_z'] = str(msg_data[2])
    else:
        output_dict[msg_type] = str(msg_data)
    output_dict['monotonic'] = str(time.monotonic_ns())
    output_msg = ','.join(output_dict.values())
    # print(output_msg)
    with open(filename, 'a') as f:
        f.write(output_msg + '\n')
