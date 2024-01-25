import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

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


if __name__ == "__main__":

    planners = ['example', 'exploration', 'reconstruction', 'drift_aware']


    df_missionID = pd.read_csv('/home/michbaum/Projects/maplab/data/loopclosure/maze/drift_aware_eval/missionIDList.txt',sep=',',names=['MissionID'],header=None)

    mission_ids = df_missionID['MissionID'].unique().tolist()

    print(mission_ids)
    dfs = {}

    for planner in planners:
        file_path = '/home/michbaum/Projects/maplab/data/loopclosure/maze/'+ planner + '_eval/lc_edges.csv'
        dfs[planner] = pd.read_csv(file_path,names=['MissionID','x_t','y_t','z_t','x_s','y_s','z_s'],header=None)

    # mission_ids = [1,2,3,4,5]# List of all 258 mission IDs

    plot_planner_comp(dfs, mission_ids,planners)
