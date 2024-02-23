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
    #df = df.iloc[::200, :]

    # # Calculate mean and standard deviation
    # mean_data = df.groupby(['planner', 'time']).mean().reset_index()
    # std_data = df.groupby(['planner', 'time']).std().reset_index()

    # # Plot the mean line
    # plt.figure(figsize=(10, 6))

    # for planner, color in zip(df['planner'].unique(), sns.color_palette('pastel')):
    #     planner_data = mean_data[mean_data['planner'] == planner]
    #     plt.plot(planner_data['time'], planner_data['occupancy_percentage'], label=planner, color=color)

    # # # Shade the error around the mean
    # # for planner, color in zip(df['planner'].unique(), sns.color_palette('pastel')):
    # #     planner_data = std_data[std_data['planner'] == planner]
    # #     plt.fill_between(planner_data['time'], 
    # #                     mean_data[mean_data['planner'] == planner]['occupancy_percentage'] - planner_data['occupancy_percentage'], 
    # #                     mean_data[mean_data['planner'] == planner]['occupancy_percentage'] + planner_data['occupancy_percentage'], 
    # #                     alpha=0.2, color=color)

    # # Set plot labels and title
    # plt.xlabel('Simulated Time (minutes)')
    # plt.ylabel('Coverage (%)')
    # plt.title('Coverage over Time')
    # plt.legend()
    # plt.show()
def plot_translation_error(gt, est):

    # Load the CSV file into a pandas DataFrame
    data = pd.DataFrame()
    data['Time'] = gt['Time']
    data['TranslationError'] = ((gt['xPosition'] - est['xPosition'])**2 + (gt['yPosition'] - est['yPosition'])**2)**0.5

    # Convert the 'Timestamp' column to datetime format
    # data['Timestamp'] = pd.to_datetime(data['Timestamp'])

    # Set the figure size
    plt.figure(figsize=(10, 6))

    # Plot TranslationError over Timestamp
    sns.lineplot(data=data, x='Time', y='TranslationError', label='Translation Error')

    # Add title and labels
    plt.title('Translation Error Over Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Translation Error (meters)')

    # Limit y-axis to 5
    # plt.ylim(0, 5)

    # Rotate x-axis labels for better visibility
    plt.xticks(rotation=45)

    # Display the plot
    plt.tight_layout()
    plt.show()

def main():
    # Go through all the files in the directory and read them
    enviroment = 'maze'
    # planner_list = ['drift_aware_floorplan_planner', 'drift_aware_planner', 'drift_aware_floorplan_TSP_planner', 'drift_aware_TSP_planner', 'reconstruction_planner', 'exploration_planner', 'example_config']
    planner_list= ['drift_aware_floorplan_planner']
    number_runs = 5

    df_all = pd.DataFrame()
    df_gt = pd.DataFrame()
    df_est = pd.DataFrame()

    # Read the occupancy.csv files and add the planner name as a column
    for planner in planner_list:
        print(planner)
        path = f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{planner}/'

        df_runs = pd.DataFrame()
        df_runs_gt = pd.DataFrame()
        df_runs_est = pd.DataFrame()
        # Get all subdirectories for each run
        for i in range(1, number_runs + 1):
            
            if planner == 'example_config' and i == 5:
                continue

            file = path + 'run_' + str(i) + '/occupancy.csv'


            file_gt = path + 'run_' + str(i) + '/groundtruth_short.csv'
            file_est = path + 'run_' + str(i) + '/floorplan_short_estimate.csv'

            df = pd.read_csv(file)
            df['planner'] = planner
            df['run'] = i

            df = df.iloc[::5, :]


            df_gt = pd.read_csv(file_gt)
            df_gt['planner'] = planner
            df_gt['run'] = i


            df_est = pd.read_csv(file_est)
            df_est['planner'] = planner
            df_est['run'] = i

            
            if(i == 1):
                df_runs = df
                df_runs_gt = df_gt
                df_runs_est = df_est
            else:
                df_runs = pd.concat([df_runs, df])
                df_runs_gt = pd.concat([df_runs_gt, df_gt])
                df_runs_est = pd.concat([df_runs_est, df_est])

            
        if planner == planner_list[0]:
            df_all = df_runs
            df_gt = df_runs_gt
            df_est = df_runs_est
        else:
            df_all = pd.concat([df_all, df_runs])
            df_gt = pd.concat([df_gt, df_runs_gt])
            df_est = pd.concat([df_est, df_runs_est])


    df_all.reset_index(drop=True, inplace=True)
    df_gt.reset_index(drop=True, inplace=True)
    df_est.reset_index(drop=True, inplace=True)

    print(df_all.head())

    plot_translation_error(df_gt, df_est)
    # plot_time_occupancy(df_all)

if __name__ == "__main__":
    main()