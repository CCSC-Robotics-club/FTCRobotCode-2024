import matplotlib.pyplot as plt
import numpy as np

# Sample data
x = np.array([1, 2, 3, 4, 5, 6])
y = np.array([2.2, 3.8, 4.9, 4.7, 5.2, 6])

# Scatter plot
plt.scatter(x, y, color='blue', label='sensor data')

# Best fit line
coefficients = np.polyfit(x, y, 1)  # 1 means linear
polynomial = np.poly1d(coefficients)
y_fit = polynomial(x)

plt.plot(x, y_fit, color='red', label='regression line')

# Adding labels and legend
plt.xlabel('parallel encoder difference')
plt.ylabel('robot yaw difference (by imu)')
plt.title('odometer calibration')
plt.legend()

# Show the plot
plt.show()