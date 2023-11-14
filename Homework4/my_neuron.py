# will have a single activation function :))
import numpy as np


class NeuronLayer:
    def __init__(self, input_size, number_neurons, activation, derivative):
        self.w = np.random.randn(input_size, number_neurons)
        self.beta = np.random.randn(number_neurons)

        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.beta.shape)

        self.input = np.zeros(input_size)
        self.cross_output = np.zeros(number_neurons)    # one per neuron

        self.gradients = np.zeros(number_neurons)   # one gradient per neuron

        self.activation = activation
        self.derivative = derivative

    def compute_value(self, data):
        self.input = np.copy(data)
        self.cross_output = np.dot(self.input, self.w) + self.beta
        return self.activation(self.cross_output)

    def add_deltas(self):
        self.w += self.delta_w
        self.beta += self.delta_beta
        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.beta.shape)

    def compute_gradients(self, next_w, next_grad):
        transposed_weights = np.transpose(next_w)
        weight_gradient = np.dot(next_grad, transposed_weights)
        self.gradients = self.derivative(self.cross_output) * weight_gradient

    def add_to_deltas(self, alfa):
        # this will be a little harder - hope this is correct
        for i in range(self.delta_w.shape[1]):
            self.delta_w[:, i] += alfa * self.input * self.gradients[i]
        self.delta_beta += alfa * self.gradients

