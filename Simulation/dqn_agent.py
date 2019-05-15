import os
import numpy as np
import tensorflow as tf

from replay_buffer import ReplayBuffer
from dqn_model import QNetwork


class Agent():
    """Interacts with and learns from the environment."""

    def __init__(self, state_size: int, action_size: int):
        '''
        """Initialize an Agent object.
        Params
        ======
        :param state_size: dimension of each state
        :param action_size: dimension of each action
        '''
        self.buffer_size = int(1e5)
        self.batch_size = 64
        self.tau = 1e-3
        self.update_every = 4
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.0005  # or 5e-4
        self.checkpoint_path = "checkpoints/cp-{epoch:04d}.ckpt"
        self.checkpoint_dir = os.path.dirname(self.checkpoint_path)
        self.cp_callback = tf.keras.callbacks.ModelCheckpoint(self.checkpoint_path,
                                                              save_weights_only=True,
                                                              period=50,
                                                              verbose=1)
        self.qnetwork = QNetwork(self.action_size)
        self.qnetwork.compile(optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate),
                           loss='mse')

        self.target_network = QNetwork(self.action_size)
        self.target_network.compile(optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate),
                              loss='mse')

        self.update_target_network()

        self.memory = ReplayBuffer(action_size, self.buffer_size, self.batch_size)

    def step(self, state, action, reward, next_state, done):
        '''
        Save experience in replay memory
        Params
        ======
        :param state: state
        :param action: action
        :param reward: reward
        :param next_state: next_staet
        :param done: done
        :return:
        '''
        self.memory.add(state, action, reward, next_state, done)

    def replay(self):
        '''
        Train agent on the replay memory. DQN algorithm with local q-network and target-network
        :return: None
        '''
        states, actions, rewards, next_states, dones = self.memory.sample()
        outputs = self.target_network.predict(next_states)
        targets = rewards + (self.gamma * np.reshape(np.amax(outputs, axis=1), [64, 1])*(1.0 - dones))
        predicts = self.qnetwork.predict(states)

        for i in range(predicts.shape[0]):
            predicts[i][actions[i]] = targets[i]

        self.qnetwork.fit(states, predicts, epochs=1,
                          verbose=0
                          # callbacks=[self.cp_callback]
                          )

        self.update_target_network()

    def act(self, state):
        '''
        Agents choosen action for given state
        Params
        ======
        :param state: state to act on
        :return: action
        '''
#        if np.random.rand() <= self.epsilon:
#            return np.random.randint(0, self.action_size)
        act_values = self.qnetwork.predict(state)
        return act_values
        #return np.argmax(act_values[0])

    def update_target_network(self):
        '''
        Update weights from target-network with local q-network
        :return: None
        '''
        self.target_network.set_weights(self.qnetwork.get_weights())

    def save_weights(self, epoch: int):
        self.qnetwork.save_weights(self.checkpoint_path.format(epoch=epoch))