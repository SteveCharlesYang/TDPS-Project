import numpy as np


def calculate_center(sum_array):
    n_mean = np.mean(sum_array)
    if int(n_mean) is 0:
        return 0
    sum_array_fixed = sum_array
    sum_array_fixed[sum_array_fixed - n_mean < 0] = 0
    left_side, right_side = find_side(sum_array_fixed)
    t_tot = 0
    for i_num in range(left_side, right_side):
        t_tot += i_num * sum_array_fixed[i_num]
    t_av = t_tot / (right_side - left_side)
    n_mean_fixed = np.mean(sum_array_fixed)
    av = t_av / n_mean_fixed
    return int(av)


def calculate_error(sum_array, center):
    n_mean = np.mean(sum_array)
    sum_array_fixed = sum_array
    sum_array_fixed[sum_array_fixed - n_mean < 0] = 0
    left_side, right_side = find_side(sum_array_fixed)
    t_tot = 0
    for i_num in range(left_side, right_side):
        add_num = (max(sum_array_fixed) - sum_array_fixed[i_num])
        t_tot += add_num
    return t_tot / np.max(sum_array_fixed)


def find_side(sum_array):
    i = 0
    while sum_array[i] is 0:
        i += 1
    left_side = i
    i = sum_array.size - 1
    while sum_array[i] is 0:
        i -= 1
    return left_side, i


def calculate_direction(positions, length):
    output_val = [0, 0]
    turn_threshold = 0.15
    center_threshold = 0.15
    if abs(positions[0] - positions[1]) / length > turn_threshold:
        output_val[0] = (positions[0] - (length / 2)) / (length / 2)
        output_val[1] = (positions[1] - (length / 2)) / (length / 2)
    elif abs(positions[0] - length / 2) / length > center_threshold or abs(positions[1] - length / 2) / length > center_threshold:
        output_val[0] = (positions[0] - (length / 2)) / (length / 2)
        output_val[1] = (positions[1] - (length / 2)) / (length / 2)
    return output_val