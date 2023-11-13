import numpy as np
from my_neuron import HiddenLayer, OutputLayer

def probability(x):
    p = np.zeros(3)
    p[x - 1] = 1
    return p

def cross_entropy(output, true_probability):
    epsilon = 1e-15
    # avoid 0
    output = np.clip(output, epsilon, 1 - epsilon)
    error = -np.sum(true_probability * np.log(output))
    return error


class NeuralNetwork:
    # first layer will have size 7 and the last will have size 3 users can only
    def __init__(self, *layer_size):
        self.layers = []
        prev = 7
        for sz in layer_size:
            self.layers.append(HiddenLayer(prev, sz))
            prev = sz

        # initialize last layer
        self.layers.append(OutputLayer(prev, 3))

    def evaluate(self, data): # feed forward step
        output = np.copy(data)
        for layer in self.layers:
            output = layer.compute_value(output)
        return output

    # the last function will be softMax
    def train(self, input_data, input_label, alfa, epochs):
        for _ in range(epochs):
            for data, label in zip(input_data, input_label):
                output = self.evaluate(data)
                true_probability = probability(input_label)
                loss = cross_entropy(output, true_probability)

                # aici ar trebui computati gradientii pt ultim layer

                for index in range(len(self.layers), -1, -1)):
                    # ceva cu gradientii mei in functioe de weight-urile si gradientii lui urmatorul
                    # ceva cu delta_w si delta_b in funtie de alfa, inputul meu si gradientii

    def test(self, validation_data, validation_label):
        good_predictions = 0
