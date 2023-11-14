import numpy as np
import re
from neural_network import NeuralNetwork


def prepare_data():
    file_path = "seeds_dataset.txt"
    with open(file_path, 'r') as file:
        data = file.read()

    lines = data.split('\n')
    parsed_data = []
    for line in lines:
        values = re.split(r'\s+', line)
        parsed_data.append([float(val) for val in values])

    # input dataset, validation dataset
    print(len(parsed_data))

    # 2/3 - input size
    np_parsed_data = np.array(parsed_data)
    np.random.shuffle(np_parsed_data)

    input_data = np_parsed_data[:140, :-1]
    input_label = np_parsed_data[:140, -1]

    validation_data = np_parsed_data[140:, :-1]
    validation_label = np_parsed_data[140:, -1]

    return input_data, input_label, validation_data, validation_label


def main():
    input_data, input_label, validation_data, validation_label = prepare_data()

    nn = NeuralNetwork([7])
    nn.train(input_data, input_label, 0.05, 1000)
    nn.test(validation_data, validation_label)


if __name__ == "__main__":
    main()


