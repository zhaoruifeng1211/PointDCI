"""Plot hand-crafted DCI values across scale counts together with their variance trend."""

import matplotlib.pyplot as plt

# m values.
m_values = [20, 30, 50, 80, 100]

# DCI strength values for five points at each m.
dci_dict = {
    20: [0.26495725000000003, 0.32051280000000004, 0.7350427500000001, 0.7008547, 0.8888889],
    30: [0.10714285000000001, 0.17857144999999996, 0.57142855, 0.64285715, 0.82142855],
    50: [0.047619049999999996, 0.18589745000000002, 0.36446884999999996, 0.5274725499999999, 0.7142857],
    80: [0.19696970000000003, 0.23977274999999998, 0.59848485, 0.73333335, 0.83333335],
    100: [0.23809524999999998, 0.2598039, 0.56442575, 0.76120445, 0.91666665]


}

# Compute the variance curve.
import numpy as np
# The variance curve summarizes disagreement among the selected points.
variances = [np.var(dci_dict[m]) for m in m_values]

# Visualize the curves.
fig, ax1 = plt.subplots(figsize=(8, 5))

# Left axis: DCI curve for each point.
# Extract the values of each point across all m values.
point_num = len(next(iter(dci_dict.values())))
for i in range(point_num):
    # Track the same point across all tested scale settings.
    values = [dci_dict[m][i] for m in m_values]
    ax1.plot(m_values, values, label=f'Point {i+1}', marker='o')

ax1.set_xlabel('Number of Scales (m)')
ax1.set_ylabel('DCI Strength')
ax1.set_ylim(0, 1)
ax1.grid(True)
ax1.legend(loc='upper left')

# Right axis: variance curve.
ax2 = ax1.twinx()
ax2.plot(m_values, variances, color='red', linestyle='--', marker='D', label='Variance')
ax2.set_ylabel('Variance among Points', color='red')
ax2.tick_params(axis='y', labelcolor='red')

# Title and final layout adjustments.
plt.title('DCI Strength vs. Scale, with Inter-Point Variance')
fig.tight_layout()
plt.show()
