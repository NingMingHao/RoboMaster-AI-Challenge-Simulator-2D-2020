# -*- coding: utf-8 -*-
# RoboMaster AI Challenge Simulator (RMAICS)

from kernal import kernal

class rmaics(object):

    def __init__(self, agent_num, render=True):
        self.game = kernal(car_num=agent_num, render=render)
        self.g_map = self.game.get_map()
        self.memory = []

    def reset(self):
        self.state = self.game.reset()
        # state, object
        self.obs = self.get_observation(self.state)
        return self.obs

    def step(self, actions):
        state = self.game.step(actions)
        obs = self.get_observation(state)
        rewards = self.get_reward(state)

        self.memory.append([self.obs, actions, rewards])
        self.state = state

        return obs, rewards, state.done, None
    
    def get_observation(self, state):
        # personalize your observation here
        obs = state
        return obs
    
    def get_reward(self, state):
        # personalize your reward here
        rewards = None
        return rewards

    def play(self):
        self.game.play()

    def save_record(self, file):
        self.game.save_record(file)
        
        
if __name__ == '__main__':
    import time
    # save_file_name = './records/record1.npy'
    save_file_name = './records/%d.npy'%int(time.time())
    game = rmaics(agent_num=4, render=True)
    game.reset()
    # only when render = True
    game.play()
    
#    from kernal import record_player
#    print('play saved file')
#    player = record_player()
#    player.play(save_file_name)