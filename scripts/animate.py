# First set up the figure, the axis, and the plot element we want to animate
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# initialization function: plot the background of each fram
a = np.random.rand(10,10)
plt.ion()
plt.figure()
ax = plt.gca()
im=ax.imshow(a, interpolation='none', extent=[0,10,0,10], aspect="auto")
def init():
    im.set_data(np.random.random((5,5)))
    return [im]

# animation function.  This is called sequentially
def animate(i):
    global a
    a=a*np.exp(-0.001*i)    # exponential decay of the values
    return a

def main():
    global im
    i = 0
    while(True):
        i += 0.5
        f =  animate(i)
        np.random.rand(10,10) + i
        im.set_array(a)
        plt.draw()
        print(i)
        time.sleep(1)
        fig.canvas.draw()
        fig.canvas.flush_events()

if __name__=='__main__':
    main()
