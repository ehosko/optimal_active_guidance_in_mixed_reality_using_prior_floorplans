import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

import os

def plot_time_occupancy(df):

    # Convert time to minutes (if time is too large)
    df['time'] = df['time'] / 60

    # Convert occupancy to percentage
    df['occupancy_percentage'] = df['occupancy'] * 100

    # Set seaborn style to use pastel colors
    sns.set(style="whitegrid")

    # Plot the line plot
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=df, x='time', y='occupancy_percentage',hue='planner', ci='sd')

    # Set plot labels and title
    plt.xlabel('Simulated Time (minutes)')
    plt.ylabel('Coverage (%)')
    plt.title('Coverage over Time')

    # Show the plot
    plt.show()


def main():
    # Go through all the files in the directory and read them
    enviroment = 'maze'
    planner_list = ['drift_aware_floorplan', 'drift_aware', 'drift_aware_floorplan_TSP', 'drift_aware_TSP']
    number_runs = 1

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
            df['planner'] = planner

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