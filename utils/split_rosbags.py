import rosbag
import os
import rospy

def split_rosbag(rosbag_path, split_duration):
    # Create a directory to store the split bags
    output_dir = os.path.splitext(rosbag_path)[0] + "_split"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Open the original rosbag file
    bag = rosbag.Bag(rosbag_path)
    
    # Get the start and end times of the rosbag
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()
    
    # Initialize variables
    current_time = start_time
    split_index = 0
    
    while current_time < end_time:
        # Calculate the end time of the current split bag
        split_end_time = current_time + split_duration
        
        # Create a new split bag
        split_bag_path = os.path.join(output_dir, "split_{}.bag".format(split_index))
        split_bag = rosbag.Bag(split_bag_path, 'w')
        
        # Write messages within the time range to the split bag
        for topic, msg, t in bag.read_messages(start_time=rospy.Time.from_sec(current_time), end_time=rospy.Time.from_sec(split_end_time)):
            split_bag.write(topic, msg, t)
        
        # Close the split bag
        split_bag.close()
        
        # Update variables for the next split
        current_time = split_end_time
        split_index += 1
    
    # Close the original rosbag
    bag.close()
    
    print("Split complete. {} bags created in {}.".format(split_index, output_dir))

def main():
    # Provide the path to the rosbag file and the desired split duration in seconds
    rosbag_path = "/home/michbaum/optag_ws/data/sim_bags/query_trajectory.bag"
    split_duration = 7.0

    # Call the function to split the rosbag
    split_rosbag(rosbag_path, split_duration)

if __name__ == "__main__":
    main()