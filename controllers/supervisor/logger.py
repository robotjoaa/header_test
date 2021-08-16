class Logger():
    def __init__(self):
        self.frame = []
        self.value = []

    def update(self, frame, value):
        self.frame.append(frame)
        self.value.append(value)

class Avg_Success() : 
    def __init__(self): 
        self.success_num = 0
        self.epi_num = 0
        self.save_plot_interval = 20
        self.last_mean_success = 0
        self.logger_success = Logger()

    def update(self, success): 
        self.epi_num += 1
        if success:
            self.success_num += 1
        if self.epi_num % self.save_plot_interval == 0 :
            mean_success = self.success_num/self.save_plot_interval
            self.last_mean_success = mean_success
            self.logger_success.update(self.epi_num, mean_success)
            self.success_num = 0

    def get_last_success(self):
        return self.last_mean_success

class Avg_Reward() : 
    def __init__(self, interval = 2000): 
        self.total_rewards = 0
        self.t = 0
        self.logger_reward = Logger()
        self.save_png_interval = interval

    def update(self, reward): 
        self.t += 1
        self.total_rewards += reward
        if self.t % self.save_png_interval == 0 :
            mean_total_reward = self.total_rewards/self.save_png_interval
            self.logger_reward.update(self.t, mean_total_reward)
            self.total_rewards = 0