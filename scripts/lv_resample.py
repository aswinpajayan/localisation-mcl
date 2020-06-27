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

def lv_sampler(weights):
    """ 
    function to perform low variance sampling 
    taken from S thruns Book
    :weights: importance weights of corresponding particles
    :returns: indeces 
    """
    indeces = []
    NUM = len(weights)
    r = np.random.uniform(0, 1 / NUM)
    c = weights[0]
    i = 0
    for m in range(NUM):
        u = r + (m - 1) * (1 / NUM)
        while(u > c):
            i += 1
            c += weights[i]
        indeces.append(i)
    return np.array(indeces, dtype='int')

