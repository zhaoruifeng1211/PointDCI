import numpy as np
import matplotlib.pyplot as plt
import os
from sys import argv
component_file=argv[1]
components=[]
birth=[]
death=[]
# Open the text file
with open(component_file, 'r') as file:
    # Read lines from the file
    lines = file.readlines()
    print(len(lines),'\n')
    # persistence_len=lines[:2]
    for line in lines:
        line=line.split(' ')
        components.append(line[:2])
        birth.append(int(line[0]))
        death.append(int(line[1]))

# Process the lines as needed
# ...
scale_size=50
num_points = scale_size
colors = np.random.rand(len(components))

# x = np.random.randint(0, 51, num_points)
# y = np.random.randint(0, 51, num_points)

plt.scatter(np.array(birth), np.array(death),c=colors)

plt.xticks(np.arange(0, 50, step=1))
plt.yticks(np.arange(0, 50, step=1))

plt.title('Scatter plot')
plt.xlabel('Birth')
plt.ylabel('Death')
plt.grid(which='both', axis='both', color='gray', linestyle='-', linewidth=1)

plt.show()