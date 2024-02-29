import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import numpy as np

import sys

def plot_reloc_heatmap(dfs, mission_ids, file):
    print("Heatmap...")
    
    planners = dfs['planner'].unique()

    planners_list = planners.tolist()
    planners_list.sort()
    mission_ids = mission_ids.flatten()

    heatmap_data = pd.DataFrame(index=mission_ids, columns=planners_list)


    for planner in planners:
        df_filtered = dfs[dfs['planner'] == planner]
        df_filtered.reset_index(drop=True, inplace=True)

        count_df = df_filtered.groupby(['id_1', 'mission', 'run']).size().reset_index(name='count')

        # Find the maximum count over id_1 for each id_2 and run
        max_count_df = count_df.groupby(['mission', 'run'])['count'].max().reset_index()

        # Calculate the mean of id_2 counts over runs
        mean_count_df = max_count_df.groupby('mission')['count'].mean().reset_index()

        # Update the corresponding column in heatmap_data
        heatmap_data[planner] = mean_count_df.set_index('mission')['count']

    heatmap_data = heatmap_data.fillna(0)
    heatmap_data = heatmap_data.astype(int)

    heatmap_data_transpose = heatmap_data.transpose()

    plt.figure(figsize=(15, 3), dpi = 600)
    # sns.set(style="whitegrid", palette="rocket")
    heatmap = sns.heatmap(heatmap_data_transpose, cmap='flare', annot=False, fmt='.2f', xticklabels=False, cbar_kws={'label': 'Number of Localized Vertices'},rasterized=False)

    # Get the colorbar
    cbar = heatmap.collections[0].colorbar

    # Set the font size for colorbar label
    cbar.set_label("Number of Localized Vertices", fontsize=10)
    # Set the font size for colorbar tick labels
    cbar.ax.tick_params(labelsize=10)

    plt.ylabel('Planners',fontsize=10)
    plt.xlabel('Query Trajectories',fontsize=10)
    plt.title('Re-localization Heatmap',fontsize=12)

    plt.yticks(fontsize=10) 
    # plt.show()
    plt.savefig(file,bbox_inches='tight')

def plot_relocerror(df,planner,name, file):
    print("Plotting relocalization error...")
    
    unique_missions_per_run = df.groupby('run')['mission'].nunique().reset_index()
    
    mean_mission_per_run = unique_missions_per_run['mission'].mean()
    std_mission_per_run = unique_missions_per_run['mission'].std()

    print(mean_mission_per_run, std_mission_per_run)

    df = df.groupby(['mission', 'run']).mean()

    # Define the bins
    bins = np.linspace(0,7,8)
    df['bin'] = pd.cut(df["error"], bins,right=False)

    # Group by 'run' and 'bin', and count occurrences
    result = df.groupby(['run', 'bin']).size().reset_index(name='count')

    # Set seaborn style to use pastel colors
    sns.set(style="whitegrid")

    # setting the dimensions of the plot
    plt.figure(figsize=(24, 12))
    
    sns.catplot(data=result, x='bin', y='count', kind='bar', height=8, aspect=2)

    plt.xlabel('Average Euclidean Distance [m]',fontsize=30)
    plt.ylabel('Count', fontsize=30)
    # plt.title('Average Squared Error of '+ str(mean_mission_per_run) + ' Missions for ' + name,fontsize=35)
    plt.title('Re-Localization Error for the ' + name,fontsize=35)

    # Resize the x-axis ticks
    plt.xticks(fontsize=25) 
    plt.yticks(fontsize=25) 

    plt.tight_layout()
    # # # Show the plot
    plt.savefig(file + planner +'_RelocError.png')
    # plt.show()


if __name__ == "__main__":

    # Go through all the files in the directory and read them
    # enviroment = 'Warehouse'
    enviroment = 'maze'
    # planner_list = ['exploration_planner', 'example_config']
    planner_list = ['drift_aware_floorplan_planner', 'drift_aware_planner', 'drift_aware_floorplan_TSP_planner', 'drift_aware_TSP_planner', 'reconstruction_planner', 'exploration_planner', 'example_config']
    planner_name_dict = {'drift_aware_floorplan_planner': "DAFP", 'drift_aware_planner': "DAP", 'drift_aware_floorplan_TSP_planner': "DAFP-TSP", 'drift_aware_TSP_planner': "DAP-TSP", 'reconstruction_planner': "RCP", 'exploration_planner': "AEP", 'example_config': "RH-NBV"}
    # planner_list= ['drift_aware_floorplan_planner']
    number_runs = 5

    df_all = pd.DataFrame()
    df_gt = pd.DataFrame()
    df_est = pd.DataFrame()

    # Read the occupancy.csv files and add the planner name as a column
    for planner in planner_list:
        print(planner)
        path = f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{planner}/'

        df_runs = pd.DataFrame()
        df_runs_heat = pd.DataFrame()
        # Get all subdirectories for each run
        for i in range(1, number_runs + 1):

            file = path + 'run_' + str(i) + '/reloc_error.csv'
            file_heat = path + 'run_' + str(i) + '/lc_edges.csv'

            df_heat = pd.read_csv(file, usecols=[0,1], names=['id_1', 'mission'])
            df = pd.read_csv(file)
            df['run'] = i

            if(i == 1):
                df_runs = df
                df_runs_heat = df_heat

            else:
                df_runs = pd.concat([df_runs, df])
                df_runs_heat = pd.concat([df_runs_heat, df_heat])

        plot_relocerror(df_runs,planner,planner_name_dict[planner],f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{enviroment}_')
   
        df_runs_heat['planner'] = planner_name_dict[planner]
        if(planner == planner_list[0]):
            df_all = df_runs_heat

        else:
            df_all = pd.concat([df_all, df_runs_heat])

    # mission id list in all subdirectories    
    mission_ids = pd.read_csv(f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{planner}/run_1/missionIDList.txt', header=None)

    df_all.reset_index(drop=True, inplace=True)
    plot_reloc_heatmap(df_all, mission_ids.values, f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{enviroment}_reloc_heatmap.png')



