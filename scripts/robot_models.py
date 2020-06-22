#!/usr/bin/env python
"""

Robot Models

Ref: Probabilistic robotics by Sebastian Thrun


"""
import cv2
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
    eta = np.sqrt(np.linalg.det(2*np.pi*cov))
    cov_inv = np.linalg.inv(cov)
    z_norm = np.einsum('...k,kl,...l->...', grid - mean, cov_inv, grid - mean)
    lim = radius ** 2 / (2 * std * std)
    z_norm[z_norm < lim] = 0
    z_norm[z_norm > lim] = z_norm[z_norm > lim] - lim
    kernel = np.exp(-0.5*z_norm) / eta
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
    field = cv2.filter2D(field, -1, kernel)
    return field


def main():
    """main method
    """
    field = get_likelihood_field(120, MAP_L, "gaussian", [5, 5, 10])
    plt.imshow(field)
    plt.show()


if __name__ == '__main__':
    main()
