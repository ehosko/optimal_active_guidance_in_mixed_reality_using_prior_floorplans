import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
csv_file = '/home/michbaum/optag_ws/data/vi_map_evaluation/maze_local_planner.csv'  # Replace with your CSV file path
data = pd.read_csv(csv_file)

# Group the DataFrame by mission_id
grouped = data.groupby('mission_id')

# Calculate the total number of columns with each mission_id
total_columns = grouped.size()

# Calculate the number of columns with non-empty position_x, position_y, and position_z entries
non_empty_columns = grouped.apply(lambda group: group[['position_x', 'position_y', 'position_z']].notna().all(axis=1).sum())

# Calculate the fraction of non-empty columns for each mission_id
fractions = non_empty_columns / total_columns

# Create a new DataFrame for visualization
result_df = pd.DataFrame({'mission_id': fractions.index, 'fraction': fractions})

# Plot a histogram using seaborn
plt.figure(figsize=(10, 6))
sns.histplot(data=result_df, x='fraction', bins=10, kde=False)
plt.title('Fraction of Re-Localizable Rig Poses')
plt.xlabel('Successful Re-Localization within a Query Trajectory [%]')
plt.ylabel('Number of Query Trajectories')
# plt.xticks(range(0, 11), [f'{i * 10}%' for i in range(0, 11)])
plt.xticks([i / 10 for i in range(0, 11)], [f'{i * 10}%' for i in range(0, 11)])
plt.tight_layout()

# Display the plot
plt.show()