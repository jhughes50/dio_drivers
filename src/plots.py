import matplotlib.pyplot as plt
import numpy as np
import time

if __name__ == '__main__':
    density = 20
    alpha = 20
    x, y = [], []
    for i in range(density+1):
        theta = (i/density) *(2*np.pi)
        x.append(alpha*np.sin(theta) * np.cos(theta))
        y.append(alpha*np.sin(theta))
        print(theta)

    print('x: ', x)
    print('y: ', y)

    plt.plot(y,x)
    plt.show()
    
