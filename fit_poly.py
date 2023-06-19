import numpy as np
import matplotlib.pyplot as plt

x = np.array([10.0, 12.5, 18.0, 40.0, 65.0, 80.0])
y = np.array([2.6, 2.11, 1.5, 0.75, 0.5, 0.4])

z = np.polyfit(x, y, 4)
print(z)

xp = np.linspace(10, 80 , 200)
p = np.poly1d(z)
_ = plt.plot(x,y,'.', xp, p(xp))
plt.show()