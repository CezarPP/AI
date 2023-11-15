import matplotlib.pyplot as plt
import numpy as np


def plot_for_each_epoch(error_for_epoch, title: str):
    plt.plot(range(1, len(error_for_epoch) + 1), error_for_epoch)
    plt.title(title)
    plt.xlabel('Epochs')
    plt.ylabel('Error')
    plt.grid(True)
    plt.show()


def plot_misclassified_two_dimensions(input_data, true_labels, predicted_labels):
    misclassified_indices = np.where(true_labels != predicted_labels)[0]

    plt.scatter(input_data[:, 0], input_data[:, 1], c=true_labels, marker='o')
    plt.scatter(input_data[misclassified_indices, 0], input_data[misclassified_indices, 1],
                c='red', marker='x', s=100, label='Misclassified')

    plt.title('Misclassified Points')
    plt.xlabel('Area')
    plt.ylabel('Perimeter')
    plt.legend()
    plt.show()
