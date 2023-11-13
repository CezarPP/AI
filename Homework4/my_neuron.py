# will have a single activation function :))
import numpy as np


class NeuronLayer:
    def __init__(self, input_size, number_neurons):
        self.w = np.zeros((input_size, number_neurons))
        self.beta = np.zeros(number_neurons)
        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.w.shape)

        self.input = np.zeros(input_size)

    def compute_value(self, data):
        self.input = np.copy(data)
        return self.activ_function(np.dot(self.w, self.input) + self.beta)

    def add_deltas(self):
        self.w += self.delta_w
        self.beta += self.delta_beta
        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.delta_beta.shape)

    def activ_function(self, x):
        pass

    def compute_gradients(self):

class HiddenLayer(NeuronLayer):
    def __init__(self, input_size, number_neurons):
        super().__init__(input_size, number_neurons)

    def activ_function(self, x):  # sigmoid
        return 1 / (1 + np.exp(x))


class OutputLayer(NeuronLayer):
    def __init__(self, input_size, number_neurons):
        super().__init__(input_size, number_neurons)

    def activ_function(self, x):  # softmax
        return np.exp(x) / np.sum(np.exp(x))

