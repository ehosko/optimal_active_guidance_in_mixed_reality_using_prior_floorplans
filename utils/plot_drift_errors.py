import sys
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_translation_error(csv_file):
    # Load the CSV file into a pandas DataFrame
    data = pd.read_csv(csv_file)

    # Convert the 'Timestamp' column to datetime format
    # data['Timestamp'] = pd.to_datetime(data['Timestamp'])

    # Set the figure size
    plt.figure(figsize=(10, 6))

    # Plot TranslationError over Timestamp
    sns.lineplot(data=data, x='Timestamp', y='TranslationError', label='Translation Error')

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


def plot_angle_error(csv_file):
    # Load the CSV file into a pandas DataFrame
    data = pd.read_csv(csv_file)

    # Convert the 'Timestamp' column to datetime format
    # data['Timestamp'] = pd.to_datetime(data['Timestamp'])

    # Set the figure size
    plt.figure(figsize=(10, 6))

    # Plot AngleError over Timestamp
    sns.lineplot(data=data, x='Timestamp', y='AngleError', color='orange', label='Angle Error')

    # Add title and labels
    plt.title('Angle Error Over Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Angle Error (radians)')

    # Rotate x-axis labels for better visibility
    plt.xticks(rotation=45)

    # Display the plot 
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <csv_file>")
    else:
        csv_file = sys.argv[1]
        plot_translation_error(csv_file)
        plot_angle_error(csv_file)
