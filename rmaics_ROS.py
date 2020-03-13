# -*- coding: utf-8 -*-
# RoboMaster AI Challenge Simulator (RMAICS)


import sys
all_path = sys.path
py2_path = [tmp_path for tmp_path in all_path if not 'python3.7' in tmp_path]
py3_path = [tmp_path for tmp_path in all_path if not 'python2.7' in tmp_path]

from roborts_msgs.msg import GimbalAngle, GameStatus, GameResult, GameSurvivor, BonusStatus, SupplierStatus #2-7
from roborts_msgs.msg import RobotStatus, RobotHeat, RobotBonus, RobotDamage, RobotShoot, ProjectileSupply, EnemyInfo, PartnerInformation #8-16
from roborts_msgs.msg import BulletVacant, BuffInfo, RobotPunish # new defined for 2020
from geometry_msgs.msg import PoseStamped, Twist
import rospy

sys.path = py3_path
from kernal import kernal
import numpy as np



class rmaics(object):
    def __init__(self, agent_num, render=True):
        self.car_num = agent_num
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
        
        
        
    def init_pub_and_sub(self):
        rospy.init_node('Simulator_node')
        self.cmd_sub_blue1 = rospy.Subscriber(self.ns_names[0]+'/'+'car_cmds', Twist, callback=self.cmd_callback_blue1, queue_size=1)###TODO
        self.cmd_sub_red1 = rospy.Subscriber(self.ns_names[1]+'/'+'car_cmds', Twist, callback=self.cmd_callback_red1, queue_size=1)
        self.cmd_sub_blue2 = rospy.Subscriber(self.ns_names[2]+'/'+'car_cmds', Twist, callback=self.cmd_callback_blue2, queue_size=1)
        self.cmd_sub_red2 = rospy.Subscriber(self.ns_names[3]+'/'+'car_cmds', Twist, callback=self.cmd_callback_red2, queue_size=1)
        
        ## 2019
        self.enemy_pubs_ = [rospy.Publisher(ns+'/'+'goal', PoseStamped, queue_size=1) for ns in self.ns_names]### for cmd line use, not suitable in codes.
        self.gimbal_angle_pubs_ = [rospy.Publisher(ns+'/'+'cmd_gimbal_angle', GimbalAngle, queue_size=1) for ns in self.ns_names] ### for gimbal control
        self.game_status_pubs_ = [rospy.Publisher(ns+'/'+'game_status', GameStatus, queue_size=30) for ns in self.ns_names]
        self.game_result_pubs_ = [rospy.Publisher(ns+'/'+'game_result', GameResult, queue_size=30) for ns in self.ns_names]
        self.game_survival_pubs_ = [rospy.Publisher(ns+'/'+'game_survivor', GameSurvivor, queue_size=30) for ns in self.ns_names]
        self.bonus_status_pubs_ = [rospy.Publisher(ns+'/'+'field_bonus_status', BonusStatus, queue_size=30) for ns in self.ns_names]
        self.supplier_status_pubs_ = [rospy.Publisher(ns+'/'+'field_supplier_status', SupplierStatus, queue_size=30) for ns in self.ns_names]
        self.robot_status_pubs_ = [rospy.Publisher(ns+'/'+'robot_status', RobotStatus, queue_size=30) for ns in self.ns_names]
        self.robot_heat_pubs_ = [rospy.Publisher(ns+'/'+'robot_heat', RobotHeat, queue_size=30) for ns in self.ns_names]
        self.robot_bonus_pubs_ = [rospy.Publisher(ns+'/'+'robot_bonus', RobotBonus, queue_size=30) for ns in self.ns_names]
        self.robot_damage_pubs_ = [rospy.Publisher(ns+'/'+'robot_damage', RobotDamage, queue_size=30) for ns in self.ns_names]
        self.robot_shoot_pubs_ = [rospy.Publisher(ns+'/'+'robot_shoot', RobotShoot, queue_size=30) for ns in self.ns_names]
        self.projectile_supply_pubs_ = [rospy.Publisher(ns+'/'+'projectile_supply', ProjectileSupply, queue_size=1) for ns in self.ns_names]
        self.partner_pubs_ = [rospy.Publisher(ns+'/'+'partner_msg', PartnerInformation, queue_size=1) for ns in self.ns_names]
        
        ## 2020
        self.buff_info_pubs_ = [rospy.Publisher(ns+'/'+'BuffInfo', BuffInfo, queue_size=1) for ns in self.ns_names]
        self.robot_punish_pubs_ = [rospy.Publisher(ns+'/'+'RobotPunish', RobotPunish, queue_size=1) for ns in self.ns_names]
        self.bullet_status_pubs_ = [rospy.Publisher(ns+'/'+'BulletVacant', BulletVacant, queue_size=1) for ns in self.ns_names]
        
        rospy.on_shutdown(self.save_record_h) ###save game before shut down
        rospy.spin()
        
    def save_record_h(self):
        self.game.save_record(self.file_name)
        print('saved this game in: ', self.file_name)
        
    def process_cmd(self, twist, car_index):
        ### simple orders consist of only x,y,rotate speeds and auto_aim
        self.new_cmds[car_index,0] = twist.linear.x
        self.new_cmds[car_index,1] = -twist.linear.y ## TODO:up down reverse
        self.new_cmds[car_index,2] = -np.degrees(twist.angular.z)
        self.new_cmds[car_index,3] = 1
        self.new_cmd_mask[car_index] = True
        self.check_and_step()
        
    def cmd_callback_blue1(self, twist):
        self.process_cmd(twist, 0)
        
    def cmd_callback_red1(self, twist):
        self.process_cmd(twist, 1)
    
    def cmd_callback_blue2(self, twist):
        self.process_cmd(twist, 2)
    
    def cmd_callback_red2(self, twist):
        self.process_cmd(twist, 3)
        
    def step_ros(self, actions):
        state = self.game.step_simple_control(actions)
        self.publish_every_thing(state)
        if state.done:
            rospy.signal_shutdown('Game end') ###shut down ros node
        
    def check_and_step(self):
        if sum(self.new_cmd_mask[:self.car_num]) == self.car_num:
            self.step_ros(self.new_cmds)
            for i in range(self.car_num):
                self.new_cmd_mask[i] = False
            
    def publish_every_thing(self, state):
        ### state(time, self.cars, buff_info, time <= 0, detect, vision)
        state = self.game.step_simple_control(None)
        time, cars, buff_info, done, detect, vision = state.time, state.agents, state.buff, state.done, state.detect, state.vision
        
        # 2019
        ## Game state
        game_state_msg = GameStatus()
        if done:
            game_state_msg.game_status = 5
        else:
            game_state_msg.game_status = 4
        game_state_msg.remaining_time = time
        for i in range(self.car_num):
            self.game_status_pubs_[i].publish(game_state_msg)
            
        ## Game result
        if done and self.car_num==4:
            game_result_msg = GameResult()
            b1_hp = cars[0,6]
            r1_hp = cars[1,6]
            b2_hp = cars[2,6]
            r2_hp = cars[3,6]
            is_blue_done = ( (b1_hp<=0) and (b2_hp<=0) )
            is_red_done = ( (r1_hp<=0) and (r2_hp<=0) )
            if is_blue_done:
                game_result_msg.result = 1
            elif is_red_done:
                game_result_msg.result = 2
            else:
                if b1_hp+b2_hp > r1_hp+r2_hp:
                    game_result_msg.result = 2
                elif b1_hp+b2_hp < r1_hp+r2_hp:
                    game_result_msg.result = 1
                else:
                    game_result_msg.result = 0
            for i in range(self.car_num):
                self.game_result_pubs_[i].publish(game_result_msg)
        
        ## Game survival
        if self.car_num==4:
            game_survival_msg = GameSurvivor()
            game_survival_msg.blue3 = cars[0,6] > 0
            game_survival_msg.red3 = cars[1,6] > 0
            game_survival_msg.blue4 = cars[2,6] > 0
            game_survival_msg.red4 = cars[3,6] > 0
            for i in range(self.car_num):
                self.game_survival_pubs_[i].publish(game_survival_msg)
                
        ## Robot status
        id_dict = {0:13, 1:3, 2:14, 3:13}
        for i in range(self.car_num):
            robot_status_msg = RobotStatus()
            robot_status_msg.id = id_dict[i]
            robot_status_msg.remain_hp = cars[i,6]
            robot_status_msg.max_hp = 2000
            robot_status_msg.heat_cooling_limit = 360
            robot_status_msg.heat_cooling_rate = 240 if cars[i,6]<400 else 120
            self.robot_status_pubs_[i].publish(robot_status_msg)
        
        ## Robot heat
        for i in range(self.car_num):
            robot_heat_msg = RobotHeat()
            robot_heat_msg.shooter_heat = cars[i,5]
            self.robot_heat_pubs_[i].publish(robot_heat_msg)
        
        ## Robot damage
        damage_source_dict = {0:0, 1:3, 2:2, 3:1}
        for i in range(self.car_num):
            robot_damage_msg = RobotDamage()
            robot_damage_msg.damage_type = 4 #default value, unattacked
            robot_damage_msg.damage_source = 4 #default value, no armor
            if cars[i,5] > 240:
                robot_damage_msg.damage_type = 2
            if (cars[i,11] != 4) and (cars[i,15] <= 100):  #attacked during the last 0.5 second
                robot_damage_msg.damage_type = 0
                robot_damage_msg.damage_source = damage_source_dict[int(cars[i,11])]
            self.robot_damage_pubs_[i].publish(robot_damage_msg)
        
        ## Partner information
        ### ROS coordinate, origin: left_down_point; x:right;  y:up
        ### Simulator coordinate, origin:left_up_point; x:right; y:down
        who_is_enemy = {0:[1,3], 1:[0,2], 2:[1,3], 3:[0,2]}
        for i in range(self.car_num):
            partner_info_msg = PartnerInformation()
            partner_info_msg.header.stamp = rospy.Time.now()
            tmp_vision = vision[i]
            for enemy_ind in who_is_enemy[i]:
                if tmp_vision[enemy_ind]:
                    partner_info_msg.enemy_detected = True
                    one_enemy_info = EnemyInfo()
                    one_enemy_info.num = 1
                    one_enemy_info.enemy_pos.pose.position.x = cars[enemy_ind,1]/100 ##pixel to meter
                    one_enemy_info.enemy_pos.pose.position.y = (448 - cars[enemy_ind,2])/100 ##pixel to meter
                    yaw_at_ros = -np.radians(cars[enemy_ind,3]) ## deg to radians
                    one_enemy_info.enemy_pos.pose.orientation.w = np.cos(yaw_at_ros)
                    one_enemy_info.enemy_pos.pose.orientation.z = np.sin(yaw_at_ros)
                    partner_info_msg.enemy_info.append(one_enemy_info)
                    
            partner_info_msg.partner_pose.pose.position.x = cars[i,1]/100
            partner_info_msg.partner_pose.pose.position.y = (448 - cars[i,2])/100
            yaw_at_ros = -np.radians(cars[i,3])
            partner_info_msg.partner_pose.pose.orientation.w = np.cos(yaw_at_ros)
            partner_info_msg.partner_pose.pose.orientation.z = np.sin(yaw_at_ros)
            
            partner_info_msg.bullet_num = cars[i,10]
            self.partner_pubs_[i].publish(partner_info_msg)
            
            
        # 2020
        ## Bullet vacant
        for i in range(self.car_num):
            bullet_vacant_msg = BulletVacant()
            bullet_vacant_msg.bullet_vacant = (cars[i,10] <= 0)
            bullet_vacant_msg.remaining_number = cars[i,10]
            self.bullet_status_pubs_[i].publish(bullet_vacant_msg)
            
        ## Buff info
        buff_info_msg = BuffInfo()
        buff_info_msg.F1.type = int(buff_info[0,0])
        buff_info_msg.F1.used = False if buff_info[0,1] else True
        buff_info_msg.F2.type = int(buff_info[1,0])
        buff_info_msg.F2.used = False if buff_info[1,1] else True
        buff_info_msg.F3.type = int(buff_info[2,0])
        buff_info_msg.F3.used = False if buff_info[2,1] else True
        buff_info_msg.F4.type = int(buff_info[3,0])
        buff_info_msg.F4.used = False if buff_info[3,1] else True
        buff_info_msg.F5.type = int(buff_info[4,0])
        buff_info_msg.F5.used = False if buff_info[4,1] else True
        buff_info_msg.F6.type = int(buff_info[5,0])
        buff_info_msg.F6.used = False if buff_info[5,1] else True
        for i in range(self.car_num):
            self.buff_info_pubs_[i].publish(buff_info_msg)
            
        ## Robot punish
        for i in range(self.car_num):
            robot_punish_msg = RobotPunish()
            robot_punish_msg.type = cars[i,8]
            robot_punish_msg.remaining_time = cars[i,7]/200 ##epoch to second
            self.robot_punish_pubs_[i].publish(robot_punish_msg)
            
        # Unuseful
        ## Robot shoot, Unused
        robot_shoot_msg = RobotShoot()
        robot_shoot_msg.frequency = 6
        robot_shoot_msg.speed = 18
        for i in range(self.car_num):
            self.robot_shoot_pubs_[i].publish(robot_shoot_msg)
            
        ## Projectile_supply, Unused
        projectile_supply_msg = ProjectileSupply()
        projectile_supply_msg.number = 0
        for i in range(self.car_num):
            self.projectile_supply_pubs_[i].publish(projectile_supply_msg)
        
        ## Robot bonus, Unused
        robot_bonus_msg = RobotBonus()
        for i in range(self.car_num):
            self.robot_bonus_pubs_[i].publish(robot_bonus_msg)
        
        ## Bonus status, Unused
        bonus_status_msg = BonusStatus()
        for i in range(self.car_num):
            self.bonus_status_pubs_[i].publish(bonus_status_msg)
        
        ## Supplier status, Unused
        supplier_status_msg = SupplierStatus()
        for i in range(self.car_num):
            self.supplier_status_pubs_[i].publish(supplier_status_msg)
        
        
    def play_using_ros(self, save_file_name='./records/record1.npy'):
        self.file_name = save_file_name
        self.ns_names = ['blue1', 'red1', 'blue2', 'red2'] ### {0:'blue1', 1:'red1', 2:'blue2', 3:'red2'}
        self.new_cmds = np.zeros((4, 4))
        self.new_cmd_mask = [False, False, False, False]
        self.init_pub_and_sub()
        
        
        
if __name__ == '__main__':
    import time
    # save_file_name = './records/record1.npy'
    save_file_name = './records/%d.npy'%int(time.time())
    game = rmaics(agent_num=4, render=False)
    game.reset()
    # only when render = True
    print('Start ROS simulation')
    game.play_using_ros(save_file_name)
    
    from kernal import record_player
    print('play saved file')
    player = record_player()
    player.play(save_file_name)