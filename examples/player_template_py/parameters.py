class Parameters():
    def __init__(self):

        # simulation parameters
        self.algorithm = "ddpg"
        self.state_type = "state_polar"
        self.action_type = "action_position"
        self.reward_type = "reward_binary"
        self.frame_skip = 10
        self.load = True
        self.play = False

        # agent parameters
        self.num_layers = 1
        self.hidden_dim = 256

        # ddpg parameters
        self.ddpg_gamma = 0.99
        self.ddpg_batch_size = 256
        self.ddpg_buffer_size = 10000
        self.ddpg_actor_lr = 0.0001
        self.ddpg_critic_lr = 0.001
        self.ddpg_grad_norm_clip = 10
        self.ddpg_dec_exploration = 5000

        # dqn parameters
        self.dqn_gamma = 0.99
        self.dqn_batch_size = 256
        self.dqn_buffer_size = 10000
        self.dqn_lr = 0.001
        self.dqn_grad_norm_clip = 10