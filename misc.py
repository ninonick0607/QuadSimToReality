import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-180, 180, 1000)
x = np.abs(x)
reward = np.maximum(0, np.maximum(0.5 - (x - 5) / 170, 1 - x / 10))
plt.plot(x, reward)
plt.xlabel('x')
plt.ylabel('reward')
plt.title('Reward Function')
plt.grid()
plt.show()