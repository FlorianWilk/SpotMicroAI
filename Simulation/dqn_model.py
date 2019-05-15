import tensorflow as tf
from tensorflow.keras import layers

class QNetwork(tf.keras.Model):

    def __init__(self, action_size, dense_one=64, dense_two=64):
        '''
        Params
        ======
        :param action_size: Dimension of each action
        :param dense_one: Number of nodes in first hidden layer
        :param dense_two: Number of nodes in second hidden layer
        '''
        super(QNetwork, self).__init__(name='q-network')
        # self.seed = np.random.seed(seed)
        self.denseOne = layers.Dense(dense_one, activation='relu')
        self.denseTwo = layers.Dense(dense_two, activation='relu')
        self.denseActions = layers.Dense(action_size, activation='linear')
        print(self.denseActions)

    def call(self, inputs):
        '''
        # Define your forward pass here,
        # using layers you previously defined (in `__init__`).
        Params
        ======
        Keras standard call for inputs
        :param inputs: batch of states
        :return: outputs for given input batch
        '''
        hidden_one = self.denseOne(inputs)
        hidden_two = self.denseTwo(hidden_one)
        hiden_three = self.denseActions(hidden_two)
        return hiden_three

    def compute_output_shape(self, input_shape):
        # You need to override this function if you want to use the subclassed model
        # as part of a functional-style model.
        # Otherwise, this method is optional.
        shape = tf.TensorShape(input_shape).as_list()
        shape[-1] = self.num_classes
        return tf.TensorShape(shape)