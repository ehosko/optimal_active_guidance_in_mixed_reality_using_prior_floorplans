import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

file = '/home/michbaum/Projects/optag_EH/data/maze_test_eval/run_1/opt_path.txt'
df_graph = pd.read_csv(file, sep=' ', header=None, names=['indx', 'vertex', 'x', 'y', 'dist'])

print(df_graph.head())

vertices = {}
edges = []
for i in range(0, len(df_graph)):
    vertex = (df_graph['x'][i], df_graph['y'][i])
    vertices[i] = vertex

    if(i == 0):
        continue
    else:
        edges.append((i-1, i))

# Set Seaborn style
sns.set(style="whitegrid")

# Create a new figure
fig, ax = plt.subplots(figsize=(10, 10))

# Plot vertices
for vertex, coordinates in vertices.items():
    ax.scatter(*coordinates, label=vertex)

# Plot edges
for edge in edges:
    start, end = edge
    ax.plot([vertices[start][0], vertices[end][0]], [vertices[start][1], vertices[end][1]], 'k-')

# Add labels
# for vertex, coordinates in vertices.items():
#     ax.text(coordinates[0], coordinates[1], vertex)

# Show the plot
plt.grid(alpha=0)  # Adjust grid transparency
# Set the background color to be transparent
plt.gca().set_facecolor('none')

# Set the figure's background color to be transparent
plt.gcf().set_facecolor('none')

plt.savefig('/home/michbaum/Projects/optag_EH/data/floorplan/maze_graph.png', dpi=300, bbox_inches='tight')