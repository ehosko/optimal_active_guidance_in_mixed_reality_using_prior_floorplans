import os
import sys
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_translation_errors(data_list, names):
    # Set the figure size
    plt.figure(figsize=(10, 6))

    # Plot TranslationError over Timestamp for each data file
    for data, name in zip(data_list, names):
        sns.lineplot(data=data, x='Timestamp', y='TranslationError', label=name)

    # Add title and labels
    plt.title('Translation Error Over Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Translation Error (meters)')

    plt.ylim(0, 5)

    # Rotate x-axis labels for better visibility
    plt.xticks(rotation=45)

    # Display the plot
    plt.tight_layout()
    plt.legend()
    plt.show()

def plot_angle_errors(data_list, names):
    # Set the figure size
    plt.figure(figsize=(10, 6))

    # Plot AngleError over Timestamp for each data file
    for data, name in zip(data_list, names):
        sns.lineplot(data=data, x='Timestamp', y='AngleError', label=name)

    # Add title and labels
    plt.title('Angle Error Over Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Angle Error (radians)')

    plt.ylim(0, 0.4)

    # Rotate x-axis labels for better visibility
    plt.xticks(rotation=45)

    # Display the plot
    plt.tight_layout()
    plt.legend()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <folder_path>")
    else:
        folder_path = sys.argv[1]
        
        data_list = []
        names = []

        for filename in os.listdir(folder_path):
            if filename.endswith(".csv"):
                # Check if the filename includes 'maze'
                if 'reconstruction' in filename:
                    csv_file = os.path.join(folder_path, filename)
                    data = pd.read_csv(csv_file)
                    data_list.append(data)
                    # Extract the 1st, 4th and 5th word from the filename split by '_'
                    filename = filename.split('.')[0]
                    filename = '_'.join([filename.split('_')[0], filename.split('_')[3], filename.split('_')[4]])
                    names.append(filename)

        plot_translation_errors(data_list, names)
        plot_angle_errors(data_list, names)
