import numpy as np
import re



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
    np.random.permutation(np_parsed_data)

    input_data = np_parsed_data[:140, :-1]
    input_label = np_parsed_data[:140, -1]

    validation_data = np_parsed_data[140:, :-1]
    validation_label = np_parsed_data[140:, -1]

    return input_data, input_label, validation_data, validation_label


def main():
    input_data, input_label, validation_data, validation_label = prepare_data()
    print(input_data)
    # neuralNetwork = NeuralNetwork(1, [7])


if __name__ == "__main__":
    main()

    nimic = np.array([1, 2, 3])

    altnimic = np.array([[1, 2], [1, 2], [1, 2]])

    print(nimic.shape)
    print(altnimic.shape)
    a = np.dot(nimic, altnimic)
    print(a)
    print(a.shape)
