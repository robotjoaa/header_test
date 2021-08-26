import matplotlib.pyplot as plt
import os

class Logger():
    def __init__(self):
        self.frame = []
        self.value = []

    def update(self, frame, value):
        self.frame.append(frame)
        self.value.append(value)
    
    def plot(self, name_, label_):
        name = str(name_)
        filename = os.path.dirname(__file__) + '/' + str(name) + '.png'
        plt.title(str(name))
        if len(self.value) == 0 :
            plt.savefig(filename)
            return
        label = label_ 
        if len(label_) != len(self.value[0]) : 
            label = list(range(len(self.value)))
        for i in range(len(self.value[0])) :
            color = 'b'
            if i == 1 : 
                color = 'r'
            elif i == 2 :
                color = 'g'
            plt.plot(self.frame, [j[i] for j in self.value], c = color, label=label[i])
        plt.legend(loc='upper right')
        plt.savefig(filename)
        plt.close()

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

class Avg_Distance() : 
    def __init__(self, interval = 2000): 
        self.total_min_dist = 0
        self.total_avg_dist = 0
        self.total_max_z = 0
        self.temp_min_dist = 1000 # arbitrary large number
        self.temp_max_z = 0
        self.temp_avg_dist = 0
        self.epi_num = 0
        self.iter_num = 0
        self.save_plot_interval = 20
        self.logger_dist = Logger()

    def update_iter(self, dist, z): 
        self.iter_num += 1
        if dist < self.temp_min_dist : 
            self.temp_min_dist = dist 
        self.temp_avg_dist += dist
        if z > self.temp_max_z : 
            self.temp_max_z = z

    
    def update_epi(self, do_plot = False) : 
        self.epi_num += 1
        self.total_min_dist += self.temp_min_dist
        self.total_avg_dist += self.temp_avg_dist/self.iter_num
        self.total_max_z += self.temp_max_z
        if self.epi_num % self.save_plot_interval == 0 :
            mean_distance = self.total_avg_dist/self.save_plot_interval
            min_distance = self.total_min_dist/self.save_plot_interval
            max_z = self.total_max_z/self.save_plot_interval
            self.logger_dist.update(self.epi_num, [mean_distance, min_distance, max_z])
            self.total_min_dist = 0 
            self.total_avg_dist = 0
            self.total_max_z = 0
        if do_plot : 
            self.logger_dist.plot('Ball-to-Head-Distance', ['AVG','MIN','ZPOS'])

        # reset
        self.temp_min_dist = 1000
        self.temp_avg_dist = 0
        self.temp_max_z = 0
        self.iter_num = 0 
