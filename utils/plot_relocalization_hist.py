import sys
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Function to create and display the histogram
def plot_histogram(csv_file_path):
    # Read the CSV file into a pandas DataFrame
    data = pd.read_csv(csv_file_path)

    # Extract the ratio values from the DataFrame
    ratios = data['ratio']

    # Define the bin edges for the histogram in 10% steps
    bins = [i/10 for i in range(0, 10)]

    # Create the histogram
    plt.hist(ratios, bins=bins, edgecolor='black')

    # Set labels and title
    plt.xlabel('Percentage')
    plt.ylabel('Count')
    plt.title('Histogram of Ratios')

    # Display the histogram
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <csv_file>")
    else:
        csv_file = sys.argv[1]
        plot_histogram(csv_file)
