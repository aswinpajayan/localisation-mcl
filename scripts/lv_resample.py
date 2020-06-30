"""

Particle Filter localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import numpy as np
import random
import bisect


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
    NUM = np.float(len(weights))
    r = np.random.uniform(0, 1 / NUM)
    c = weights[0]
    i = 0
    for m in np.arange(NUM):
        u = r + (m - 1) * (1 / NUM)
        while(u > c):
            i += 1
            c += weights[i]
        indeces.append(i)
    return np.array(indeces, dtype='int')


class WeightedDistribution(object):
    def __init__(self, weights):
        accum = 0.0
        self.weights = [p for p in weights if p.w > 0]
        self.distribution = []
        for x in self.weights:
            accum += weights
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.weights[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None


def stratic_sampler(weights):
    """ https://github.com/mjl/particle_filter_demo/blob/master/particle_filter.py """
    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(weights)

    for _ in weights:
        p = dist.pick()
#        if p is None:  # No pick b/c all totally improbable
#            new_particle = Particle.create_random(1, world)[0]
#        else:
#            new_particle = Particle(p.x, p.y,
#                    heading=robbie.h if ROBOT_HAS_COMPASS else p.h,
#                    noisy=True)
#        new_particles.append(new_particle)
#
#    particles = new_particles


if __name__ == '__main__':
    weights = np.random.rand(10)
    for i in np.arange(5):
        print('supplied random weights {}'.format(weights))
        indeces = lv_sampler(weights)
        print('resampled indeces are {}'.format(indeces))
        weights = weights[indeces]
