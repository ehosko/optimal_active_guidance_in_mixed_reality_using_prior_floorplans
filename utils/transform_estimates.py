import pandas as pd
import numpy as np

def transform_estimates(path, i):
     
    trajfile = path + 'run_' + str(i) + '/traj_rosbag_estimate_2.csv'
    transformfile = path + 'run_' + str(i) + '/transform_est.txt'

    df_traj = pd.read_csv(trajfile, sep=',',usecols=[u'Time',u'xPosition',u'yPosition',u'zPosition'])

    vectors = df_traj[['xPosition', 'yPosition', 'zPosition']].to_numpy()

    # Add ones to the vectors
    ones = np.ones((len(vectors), 1))
    vectors = np.hstack((vectors, ones))

    print(vectors.shape)

    # Read the matrix from the file
    with open(transformfile, 'r') as file:
        lines = file.readlines()

    # Process the lines and create a 2D NumPy array
    matrix_data = [list(map(float, line.split())) for line in lines]
    matrix_array = np.array(matrix_data)

    transformed_vectors = np.dot(matrix_array, vectors.T).T

    df_traj['xPosition'] = transformed_vectors[:,0]
    df_traj['yPosition'] = transformed_vectors[:,1]
    df_traj['zPosition'] = transformed_vectors[:,2]

    print(df_traj.head())


    trajfile_new = path + 'run_' + str(i) + '/traj_rosbag_estimate_transform_2_test.csv'
    df_traj.to_csv(trajfile_new, index=False)

def concat_baglogs(path, i):

    est_1 = path + 'run_' + str(i) + '/traj_rosbag_estimate_1.csv'
    est_2 = path + 'run_' + str(i) + '/traj_rosbag_estimate_transform_2.csv'

    gt_1 = path + 'run_' + str(i) + '/traj_rosbag_gt_1.csv'
    gt_2 = path + 'run_' + str(i) + '/traj_rosbag_gt_2.csv'

    df_est_1 = pd.read_csv(est_1, usecols=[u'Time',u'xPosition',u'yPosition',u'zPosition'])
    df_est_2 = pd.read_csv(est_2)

    df_gt_1 = pd.read_csv(gt_1)
    df_gt_2 = pd.read_csv(gt_2)

    df_est = pd.concat([df_est_1, df_est_2])
    df_gt = pd.concat([df_gt_1, df_gt_2])

    df_est['Time'] = round(df_est['Time'],3)
    df_gt['Time'] = round(df_gt['Time'],3)

    df_est.reset_index(drop=True, inplace=True)
    df_gt.reset_index(drop=True, inplace=True)

    #Keep only one time entry
    df_est = df_est.drop_duplicates(subset=['Time'], keep='first')
    df_gt = df_gt.drop_duplicates(subset=['Time'], keep='first')

    df_est.reset_index(drop=True, inplace=True)
    df_gt.reset_index(drop=True, inplace=True)

    print(df_est_1.shape)
    print(df_gt_1.shape)
    print(df_est_2.shape)
    print(df_gt_2.shape)
    print(df_est.shape)
    print(df_gt.shape)

    df_est.to_csv(path + 'run_' + str(i) + '/traj_rosbag_estimate.csv', index=False)
    df_gt.to_csv(path + 'run_' + str(i) + '/traj_rosbag_gt.csv', index=False)
    

def main():
    
    planner_list = ['drift_aware_floorplan_planner', 'drift_aware_planner', 'drift_aware_floorplan_TSP_planner', 'drift_aware_TSP_planner', 'reconstruction_planner', 'exploration_planner', 'example_config']
    # planner_list = ['drift_aware_floorplan_planner']
    number_runs = 5

    enviroment = 'maze'

    # Read the occupancy.csv files and add the planner name as a column
    for planner in planner_list:
        print(planner)
        path = f'/home/michbaum/Projects/optag_EH/data/{enviroment}/{planner}/'

        
        # Get all subdirectories for each run
        for i in range(1, number_runs + 1):
            
            if planner == 'example_config' and i == 5:
                continue

            # transform_estimates(path, i)
            concat_baglogs(path, i)
        

if __name__ == "__main__":
    main()