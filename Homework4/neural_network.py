from activation_functions import *
from my_neuron import NeuronLayer


def probability(x):
    p = np.zeros(3)
    p[int(x) - 1] = 1
    return p


def cross_entropy(output, true_probability):
    epsilon = 1e-15
    # avoid 0
    output = np.clip(output, epsilon, 1 - epsilon)
    error = -np.sum(true_probability * np.log(output))
    return error


def min_max_scaling(data):
    min_value = np.min(data)
    max_value = np.max(data)
    scaled_data = (data - min_value) / (max_value - min_value)
    return scaled_data


class NeuralNetwork:
    # first layer will have size 7 and the last will have size 3 users can only
    def __init__(self, layer_size):
        self.layers = []
        prev = 7
        for sz in layer_size:
            self.layers.append(NeuronLayer(prev, sz, sigmoid, sigmoid_derivative))
            prev = sz

        # initialize last layer
        self.layers.append(NeuronLayer(prev, 3, softmax, softmax_derivative))

    def evaluate(self, data):  # feed forward step
        output = np.copy(data)
        for layer in self.layers:
            output = layer.compute_value(output)
        return output

    # the last function will be softMax
    def train(self, input_data, input_label, alfa, epochs):
        for _ in range(epochs):
            loss = []
            for data, label in zip(input_data, input_label):

                scaled_data = min_max_scaling(data)  # scaling is needed

                output = self.evaluate(scaled_data)
                true_probability = probability(label)
                loss.append(cross_entropy(output, true_probability))  # loss average should pe plotted, maybe

                # compute gradients for last error
                self.layers[-1].gradients = true_probability - output
                self.layers[-1].add_to_deltas(alfa)

                for index in range(len(self.layers) - 2, -1, -1):
                    self.layers[index].compute_gradients(self.layers[index + 1].w, self.layers[index + 1].gradients)
                    self.layers[index].add_to_deltas(alfa)

                for layer in self.layers:
                    layer.add_deltas()

            print(sum(loss) / len(loss))

    def test(self, validation_data, validation_label):
        good_predictions = 0
        for data, label in zip(validation_data, validation_label):
            scaled_data = min_max_scaling(data)
            output = self.evaluate(scaled_data)
            index = np.argmax(output) + 1
            if index == label:
                good_predictions += 1
        print(f"{good_predictions / len(validation_data) * 100}%")
