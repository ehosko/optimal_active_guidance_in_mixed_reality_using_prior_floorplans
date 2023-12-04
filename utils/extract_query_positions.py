import os
import rosbag
import numpy as np
import pandas as pd
from tqdm import tqdm
from rospy import Duration

# Specify the input folder containing ROS bag files
# input_folder = '/home/michbaum/optag_ws/data/sim_bags/maze_slowdown_gt_split/' # Maze
input_folder = '/home/michbaum/optag_ws/data/sim_bags/warehouse_slowdown_gt_without_windows_split' # Warehouse

# Specify the output CSV file
# csv_file = '/home/michbaum/optag_ws/data/vi_map_evaluation/maze_query_maps.csv' # Maze
csv_file = '/home/michbaum/optag_ws/data/vi_map_evaluation/warehouse_query_maps.csv' # Warehouse

# Lists to store extracted data
data_list = []

# Get a list of bag files in the input folder
bag_files = [bag_file for bag_file in os.listdir(input_folder) if bag_file.endswith('.bag')]

# Create a tqdm progress bar for the loop
for bag_file in tqdm(bag_files, desc='Processing bags', unit='bag'):
    bag_path = os.path.join(input_folder, bag_file)
    
    # Open the ROS bag file
    bag = rosbag.Bag(bag_path, 'r')

    # Get the bag number from the filename
    bag_number = int((bag_file.split('.')[0]).split('_')[1])

    # Iterate through messages in the bag
    frame_number = 0
    for topic, msg, t in bag.read_messages():
        if topic == '/cam0/image_raw':
            # Find the closest PointStamped position
            closest_position = None
            min_time_diff = float('inf')
            # Search for the closest position message within 20 nanoseconds
            for _, position_msg, position_time in bag.read_messages(topics='/firefly/ground_truth/position', start_time=t - Duration(nsecs=1e7), end_time=t + Duration(nsecs=1e7)):
                time_diff = abs((position_time - t).to_sec())
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_position = position_msg
            
            if closest_position is None:
                print("No position found for frame %d in bag %d" % (frame_number, bag_number))
                data_list.append([bag_number, frame_number, np.nan, np.nan, np.nan])
                frame_number += 1
                continue
            else:
                if min_time_diff > 0.001:
                    print("Time difference > 1 ms: %f, for frame %d in bag %d" % (min_time_diff, frame_number, bag_number))

            # Extract position information
            position_x = closest_position.point.x
            position_y = closest_position.point.y
            position_z = closest_position.point.z

            # Append data to the list
            data_list.append([bag_number, frame_number, position_x, position_y, position_z])
            frame_number += 1

    # Close the ROS bag
    bag.close()

# Convert the data list to a pandas DataFrame
data_df = pd.DataFrame(data_list, columns=['bag_number','frame_number','position_x','position_y','position_z'])

# Write the DataFrame to a CSV file
data_df.to_csv(csv_file, index=False)

print("Data saved to %s" % csv_file)
