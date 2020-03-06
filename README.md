# RoboMaster AI Challenge Simulator 2D 2020

此模拟器是基于[LoveThinkinghard](https://github.com/LoveThinkinghard)的[RoboMaster AI Challenge Simulator 2D](https://github.com/LoveThinkinghard/RoboMaster-AI-Challenge-Simulator-2D)

## 主要做了如下修改：

* 更改地图为新地图
* 加入了奖励区/惩罚区的随机刷新机制
* 完善了line_rect_check函数（用于判断子弹是否与其他物体接触）
* 考虑了不同装甲板伤害不同（同时提供本机器人在最近1s内受攻击装甲板编号）
* 删除了agent的freeze_time, is_supply, stay_time属性，actions的supply属性
* 增加了agents的punish_time, punish_state, last_attacked_armor, time_since_last_attacked属性
* 将compet改为buff_info

==========================================

RoboMaster AI Challenge Simulator 2D，简称`RMAICS`，是为参加 [ICRA 2019 RoboMaster AI Challenge](https://www.robomaster.com/zh-CN/resource/pages/980?type=announcementSub) 设计的模拟器，主要作用是为智能决策组训练神经网络提供仿真环境

![demo](./demo.gif)

游戏帧率在200fps左右（1秒迭代200次）

## 一、依赖

numpy

[pygame](https://www.pygame.org/)（作用仅为可视化）

## 二、使用指南

### 1、基本信息

该模拟器由两个层次组成：

>上层的封装类`rmaics`  
>底层的实际执行类`kernal`

使用者需要定义`rmaisc`类中的`get_observation`和`get_reward`函数，来定义观测值和奖励值；而`kernal`类只负责物理环境和裁判系统的仿真。故训练网络时，直接与使用者打交道的为`rmaisc`类

### 2、内容引索

请根据以下引索，寻找需要的内容

`rmaics`使用说明：[rmaics_manual.md](./docs/rmaics_manual.md)

`kernal`使用说明：[kernal_manual.md](./docs/kernal_manual.md)

`record player`使用说明：[record_player.md](./docs/record_player.md)

控制指令说明：[operation.md](./docs/operation.md)

参数格式说明：[params.md](./docs/params.md)

`kernal`开发指南：[develop.md](./docs/develop.md)

### 3、其他

`kernal`为`kernel`的错误写法，有时间改过来



## TODO

加入ros接口

