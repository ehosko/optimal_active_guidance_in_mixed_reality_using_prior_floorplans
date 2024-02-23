import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import numpy as np

import sys

def plot_planner_comp(dfs, mission_ids,planners):
    print("Plotting planner comparison...")

    merged_df = pd.DataFrame({'MissionID': mission_ids})

    # for planner in planners:
    #     planner_df = dfs[planner]
    #     merged_df = pd.merge(merged_df, planner_df, on='MissionID', how='left')
    
    # print(merged_df)

    heatmap_data = pd.DataFrame(index=mission_ids, columns=planners)

    for planner in planners:
        heatmap_data[planner] = dfs[planner].groupby('MissionID').count()

    print(heatmap_data)

    heatmap_data = heatmap_data.fillna(0)
    heatmap_data = heatmap_data.astype(int)

    heatmap_data_transpose = heatmap_data.transpose()

    plt.figure(figsize=(25, 3))
    palette = sns.color_palette("rocket", as_cmap=True)
    # sns.set(style="whitegrid", palette="rocket")
    sns.heatmap(heatmap_data_transpose, cmap='flare', annot=False, fmt='d', linewidths=0.9, xticklabels=False, cbar_kws={'label': 'Number of Localized Vertices'},rasterized=False)
    plt.ylabel('Planners')
    plt.xlabel('Sample Trajectories')
    plt.title('Re-localization Heatmap')
    plt.show()

def plot_relocerror(df,planner):
    print("Plotting relocalization error...")

    # bins = pd.IntervalIndex.from_tuples([(0, 1),(1 , 2), (2, 3), (3,4),(4, 5)])
    # Define the bins
    bins = np.linspace(0,5,6)
    df['bin'] = pd.cut(df["error"], bins,right=False)

    # Group by 'run' and 'bin', and count occurrences
    result = df.groupby(['run', 'bin']).size().reset_index(name='count')

    print(result)

    # print(df.head())    
    mission_list = df['mission'].unique()
    
    # Set seaborn style to use pastel colors
    sns.set(style="whitegrid")

    # # # Plot the line plot
    plt.figure(figsize=(10, 6))
    
    sns.catplot(data=result, x='bin', y='count', kind='bar')

    plt.xlabel('Average Euclidean Distance [m]')
    plt.ylabel('Frequency')
    plt.title('Average Squared Error of '+ str(len(mission_list))+ ' Missions for ' + planner + ' Planner')


    # # # Show the plot
    plt.show()


if __name__ == "__main__":

    # planners = ['example', 'exploration', 'reconstruction', 'drift_aware']


    # df_missionID = pd.read_csv('/home/michbaum/Projects/maplab/data/loopclosure/maze/drift_aware_eval/missionIDList.txt',sep=',',names=['MissionID'],header=None)

    # mission_ids = df_missionID['MissionID'].unique().tolist()

    # print(mission_ids)
    # dfs = {}

    # for planner in planners:
    #     file_path = '/home/michbaum/Projects/maplab/data/loopclosure/maze/'+ planner + '_eval/lc_edges.csv'
    #     dfs[planner] = pd.read_csv(file_path,names=['MissionID','x_t','y_t','z_t','x_s','y_s','z_s'],header=None)

    # # mission_ids = [1,2,3,4,5]# List of all 258 mission IDs

    # plot_planner_comp(dfs, mission_ids,planners)

    df = pd.read_csv('/home/michbaum/ElisaSemesterProject/data/test/test_error.csv')
    plot_relocerror(df,'test')


