import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import os

def plot_time_occupancy(df_Time):

    # # Convert time to minutes (if time is too large)
    # round df time to 2 decimal places
    df_Time['rounded_time'] = df_Time['time'].round(0)

    df = df_Time.groupby(['rounded_time','run', 'planner']).mean('occupancy').reset_index()

    print(df.head())

    # Drop the original 'time' column and unnecessary columns
    df = df.drop(['time'], axis=1)
    df['rounded_time'] = df['rounded_time'] / 60

    #df = df.iloc[::200, :]
    # Convert occupancy to percentage
    df['occupancy_percentage'] = df['occupancy'] * 100

    # # Set seaborn style to use pastel colors
    sns.set(style="whitegrid")

    # Plot the line plot
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=df, x='rounded_time', y='occupancy_percentage',hue='planner', err_style="band")

    # Set plot labels and title
    plt.xlabel('Simulated Time (minutes)')
    plt.ylabel('Coverage (%)')
    plt.title('Coverage over Time')

    # Show the plot
    plt.show()


def main():
    # Go through all the files in the directory and read them
    # enviroment = 'Warehouse'
    enviroment = 'maze'
    planner_list = ['drift_aware_floorplan_planner', 'drift_aware_planner', 'drift_aware_floorplan_TSP_planner', 'drift_aware_TSP_planner', 'reconstruction_planner', 'exploration_planner', 'example_config']
    planner_name_dict = {'drift_aware_floorplan_planner': "DAFP", 'drift_aware_planner': "DAP", 'drift_aware_floorplan_TSP_planner': "DAFP-TSP", 'drift_aware_TSP_planner': "DAP-TSP", 'reconstruction_planner': "RCP", 'exploration_planner': "AEP", 'example_config': "RH-NBV"}

    number_runs = 5

    df_all = pd.DataFrame()

    # Read the occupancy.csv files and add the planner name as a column
    for planner in planner_list:
        print(planner)
        path = f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{planner}/'

        df_runs = pd.DataFrame()

        # Get all subdirectories for each run
        for i in range(1, number_runs + 1):
            
            file = path + 'run_' + str(i) + '/occupancy.csv'

            df = pd.read_csv(file)
            df['planner'] = planner_name_dict[planner]
            df['run'] = i

            # files are very large, so we only take every 5th entry
            df = df.iloc[::5, :]

            
            if(i == 1):
                df_runs = df

            else:
                df_runs = pd.concat([df_runs, df])


            
        if planner == planner_list[0]:
            df_all = df_runs

        else:
            df_all = pd.concat([df_all, df_runs])



    df_all.reset_index(drop=True, inplace=True)

    print(df_all.head())

    plot_time_occupancy(df_all)

if __name__ == "__main__":
    main()