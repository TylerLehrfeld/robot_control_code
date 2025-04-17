import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv('img.csv')  # replace with your file

# Get bounds
x_min, x_max = df['x'].min(), df['x'].max()
y_min, y_max = df['y'].min(), df['y'].max()

# Compute grid size
width = x_max - x_min + 1
height = y_max - y_min + 1

# Initialize grid with NaNs
grid = np.full((height, width), np.nan)

# Shift coordinates to zero-based indexing
for _, row in df.iterrows():
    x_idx = int(row['x'] - x_min)
    y_idx = int(row['y'] - y_min)
    grid[y_idx, x_idx] = row['val']

# Create colormap with white for missing values
cmap = plt.cm.hot
cmap.set_bad(color='white')

# Plot
plt.imshow(grid, cmap=cmap, origin='lower', extent=[x_min, x_max+1, y_min, y_max+1])
plt.colorbar(label='val')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Heatmap')
plt.show()
