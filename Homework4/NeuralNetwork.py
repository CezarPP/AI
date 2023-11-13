import numpy as np


class NeuralNetwork:
    def __init__(self, nr_layers, *layer_size):
        self.nr_layers = nr_layers

    # will implement feed forward backpropagation + evaluation on last layer
