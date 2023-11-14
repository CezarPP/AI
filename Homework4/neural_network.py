import numpy as np
from my_neuron import NeuronLayer

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


def sigmoid(x):
    return 1 / (1 + np.exp(x))

def sigmoid_derivative(x):
    sigmoid_x = sigmoid(x)
    return sigmoid_x * (1 - sigmoid_x)

def softmax(x):
    np.exp(x) / np.sum(np.exp(x))

def softmax_derivative(x):
    return 0

class NeuralNetwork:
    # first layer will have size 7 and the last will have size 3 users can only
    def __init__(self, *layer_size):
        self.layers = []
        prev = 7
        for sz in layer_size:
            self.layers.append(NeuronLayer(prev, sz, sigmoid, sigmoid_derivative))
            prev = sz

        # initialize last layer
        self.layers.append(NeuronLayer(prev, 3, softmax, softmax_derivative))

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
                loss = cross_entropy(output, true_probability)  # loss average should pe plotted, maybe

                # aici ar trebui computati gradientii pt ultim layer

                for index in range(len(self.layers), -1, -1)):
                    self.layers[index].compute_gradients(self.layers[index+1].w, self.layers[index+1].gradients)
                    self.layers[index].add_to_deltas()

                    # ceva cu gradientii mei in functioe de weight-urile si gradientii lui urmatorul
                    # ceva cu delta_w si delta_b in funtie de alfa, inputul meu si gradientii

    def test(self, validation_data, validation_label):
        good_predictions = 0
