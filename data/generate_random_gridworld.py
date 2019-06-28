import numpy as np
import matplotlib.pyplot as plt
from mazelab.generators import random_shape_maze

if __name__ == "__main__":
    size = 500
    max_obs_size = size/10
    max_shapes_num = size/2
    x = random_shape_maze(width=size, height=size, max_shapes=max_shapes_num,
            max_size=max_obs_size, allow_overlap=False, shape=None)
    x = 1 - x;
    plt.imsave( "random_gridworld.jpg", x, cmap='gray');
