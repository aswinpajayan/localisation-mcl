#!/usr/bin/env python
"""

Robot Models

Ref: Probabilistic robotics by Sebastian Thrun


"""
import cv2
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np


SIGMA_SQ = 5
# MAP for the landmarks
MAP_L = np.array([[3, 1], [0, 5], [-2, 3], [-4, -1], [1, -2], [2, -1]])


def get_sq_kernel(k_size, radius, std):
    """function to generate  square kernel to produce
    discretised likelihood grid we can convolve
    this kernel with landmarks(ground_truth /
    observed to produce discretised likelihood field)

    :k_size : kernel size (-k_size:k_size)
    :radius : radius of landmark
    :std    : standard deviation of meaurement model
    returns: flat top gaussian kernel  """
    # build a gaussian with  a flat top..
    x_values = np.hstack((range(-(k_size / 2 - radius), 0),
                          np.zeros(radius),
                          range(0, k_size / 2 - radius))).reshape(-1, 1)
    kernel_1d = np.exp(-x_values ** 2 / (2 * std * std)).reshape(-1, 1)
    kernel_2d = np.dot(kernel_1d, kernel_1d.T)
    kernel_2d = kernel_2d / np.max(kernel_2d)
    return kernel_2d


def get_circ_kernel(k_size, radius, std):
    """function to generate kernel to produce
    discretised likelihood grid we can convolve
    this kernel with landmarks(ground_truth /
    observed to produce discretised likelihood field)

    :k_size : kernel size (-k_size:k_size)
    :radius : radius of landmark
    :std    : standard deviation of meaurement model
    returns: flat top gaussian kernel  """
    # build a gaussian with  a flat top..
    x_values = np.arange(-k_size, k_size).reshape(-1, 1)
    kernel_1d = np.exp(-x_values ** 2 / (2 * std * std)).reshape(-1, 1)
    kernel_2d = np.dot(kernel_1d, kernel_1d.T)
    top = np.exp(-(radius ** 2) / (2 * std * std))
    kernel_2d[kernel_2d > top] = top
    kernel_2d = kernel_2d / np.max(kernel_2d)
    return kernel_2d


def get_kernel(k_size, radius, std):
    """function to generate kernel to produce
    discretised likelihood grid we can convolve
    this kernel with landmarks(ground_truth /
    observed to produce discretised likelihood field)

    :k_size : kernel size (-k_size:k_size) (scaled)
    :radius : radius of landmark
    :std    : standard deviation of meaurement model
    returns: flat top gaussian kernel  """

    cov = np.diag([std, std])
    mean = np.array([0, 0], dtype=np.float)
    x_axis = np.arange(-k_size, k_size)
    y_axis = np.arange(-k_size, k_size)
    x_values, y_values = np.meshgrid(x_axis, y_axis)
    grid = np.empty(x_values.shape + (2,))
    grid[:, :, 0] = x_values
    grid[:, :, 1] = y_values
    cov_inv = np.linalg.inv(cov)
    z_norm = np.einsum('...k,kl,...l->...', grid - mean, cov_inv, grid - mean)
    lim = radius ** 2 / (2 * std * std)
    z_norm[z_norm < lim] = 0
    z_norm[z_norm > lim] = z_norm[z_norm > lim] - lim
    kernel = np.exp(-0.5*z_norm)
    kernel = kernel / np.amax(kernel)
    return kernel


def get_likelihood_field(size, map_l, k_type, world_params):
    """
    function to generate measure ment model
    as a discretised likelihood grid

    size : size of the likeihood field
    dim  : radius of the landmark
    std  : standard deviation of measurement model
    scale : phyical to grid scale factor

    returns: discretised likelihood field

    """
    dim, std, scale = world_params
    dim = dim * scale
    std = std * scale
    size = size * scale
    if(k_type == "gaussian"):
        kernel = get_kernel(dim + 3 * std, dim, std)
    else:
        kernel = get_sq_kernel(dim + 3 * std, dim, std)
    field = np.zeros((size, size), dtype=np.float)
    for point in map_l:
        field[scale * point[0] + size / 2, -scale * point[1] + size / 2] = 1.0
    field = signal.convolve2d(field, kernel)
    # field[field < 0] = np.finfo(float).eps
    return field


def motion_model(pose, cmd, delta_t):
    """velocity motion_model for the robot.
    constants taken from the m2wr URDF
    for implementation details, check
    http://docs.ros.org/jade/api/gazebo_plugins/html/gazebo__ros__diff__drive_8cpp_source.html

    :pose: prev pose of the robot
    :cmd: velocity command given to robot
    :delta_t: time elapsed
    :returns: next pose of the robot

    """
    x, y, phi = pose
    lin_vel, ang_vel = cmd
    SEP = 0.4  # wheel seperation , see m2wr.gazebo
    RAD = 0.1  # wheel radius , see m2wr.gazebo
    v_left = (lin_vel + ang_vel * SEP / 2)
    v_right = (lin_vel - ang_vel * SEP / 2)
    slip_left = v_left * RAD * delta_t
    slip_right = v_right * RAD * delta_t
    slip_sum = slip_left + slip_right
    slip_diff = slip_left - slip_right
    #x = x + (slip_sum / 2) * np.cos(phi + slip_diff / (2 * SEP))
    #y = y + (slip_sum / 2) * np.sin(phi + slip_diff / (2 * SEP))
    ang_vel = -ang_vel
    if(np.abs(ang_vel) >= 0.01):
        r = lin_vel / ang_vel
        x = x - r * np.sin(phi) + r * np.sin(phi + ang_vel * delta_t)
        y = y + r * np.cos(phi) - r * np.cos(phi + ang_vel * delta_t)
    else:
        x = x + lin_vel * np.cos(phi) * delta_t
        y = y + lin_vel * np.sin(phi) * delta_t
    phi = phi + ang_vel * delta_t
    #phi = phi + slip_diff / SEP
    return [x, y, phi]



def main():
    """main method
    """
    field = get_likelihood_field(120, MAP_L, "gaussian", [5, 5, 10])
    plt.imshow(field)
    plt.show()


if __name__ == '__main__':
    main()
