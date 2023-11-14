# will have a single activation function :))
import numpy as np

class NeuronLayer:
    def __init__(self, input_size, number_neurons, activation, derivative):
        self.w = np.zeros((input_size, number_neurons))
        self.beta = np.zeros(number_neurons)
        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.w.shape)

        self.input = np.zeros(input_size)
        self.cross_output = np.zeros(input_size)

        self.gradients = 0

        self.activation = activation
        self.derivative = derivative

    def compute_value(self, data):
        self.input = np.copy(data)
        self.cross_output = np.dot(self.w, self.input) + self.beta
        return self.activation(self.cross_output)

    def add_deltas(self):
        self.w += self.delta_w
        self.beta += self.delta_beta
        self.delta_w = np.zeros(self.w.shape)
        self.delta_beta = np.zeros(self.delta_beta.shape)

    def compute_gradients(self, next_w, next_grad):
        aux = np.dot(next_grad, next_w)
        self.gradients = np.dot(self.derivative(self.cross_output), aux)

    def add_to_deltas(self):
        self.delta_w =

