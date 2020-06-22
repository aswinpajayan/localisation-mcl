"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import numpy as np


def re_sampling(weights):
    """
    low variance re-sampling
    """
    NP = len(weights)
    w_cum = np.cumsum(weights)
    base = np.arange(0.0, 1.0, 1 / NP)
    re_sample_id = base + np.random.uniform(0, 1 / NP)
    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)
    return indexes
