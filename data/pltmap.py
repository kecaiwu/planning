import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# read map data
data = np.genfromtxt('/home/kecai/Desktop/term3/CarND-Path-Planning-Project/data/highway_map.csv')

# convert map data to dataframe type (just for viewing)
dataframe = pd.DataFrame(data, columns=['x', 'y', 's', 'dx', 'dy'])

x = data[:, 0]
y = data[: ,1]

plt.scatter(x, y, edgecolors='none', s=40)

# start point
plt.scatter(x[0], y[0], edgecolors='none', s=120)

# destination point
plt.scatter(x[-1], y[-1], edgecolors='none', s=120)

# set chart title and label axes
plt.title('PathPlanning XY Map', fontsize=20)
plt.xlabel('X', fontsize=14)
plt.ylabel('Y', fontsize=14)

# set size of tick labels
plt.tick_params(axis='both', which='major', labelsize=14)

plt.show()
