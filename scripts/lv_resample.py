"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import numpy as np


def re_sampling(weights):
    """
    low variance re-sampling
    """
    NP = 40
    w_cum = np.cumsum(weights)
    base = np.linspace(0.0, 1.0, NP)
    re_sample_id = base + np.random.uniform(0, 1 , NP)
    indexes = []
    ind = 0
    print(re_sample_id.shape)
    print(w_cum.shape)
    for ip in np.arange(40):
        while re_sample_id[ip] > w_cum[ind]:
            ind += 1
        indexes.append(ind)
    return indexes
