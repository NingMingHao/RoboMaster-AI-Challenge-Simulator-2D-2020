# 仿真使用 ROS topics

## 基本介绍
* 最新版仿真`python rmaics_ROS_auto_pub.py`进入仿真，并等待开始（或暂停）信号（`rostopic pub /start_signal std_msgs/Int16 1`，此版本会发送odom，base_link，map之间的tf（其中odom与base_link前应加上namespace/)
* 启动仿真只需运行  `python rmaics_ROS.py`或`python rmaics_ROS_fb.py`（强制发送比赛状态信息， rostopic pub /force_broadcast std_msgs/Int16 1 -r 10        #-r 后边的数字决定强制发送比赛状态信息的频率）
* 为提高仿真速度，仿真过程中默认不渲染，等比赛结束后可进行回放
* 仿真节点名称为： **Simulator_node**
* 不同机器人对应的命名域：**['blue1', 'red1', 'blue2', 'red2']**
* 下面介绍的 topics 的命名均为**相对名称**（即最左边无“/”)，其**绝对名称为 “命名域+/+相对名称”**

* 仿真订阅四个机器人发出的**控制指令** msg，其 topic 为‘car_cmds’，类型为‘geometry_msgs/Twist'，（只选用 linear.x linear.y angular.z，单位分别为 m/s 以及 rad/s，坐标为车身坐标：x前，y 左，z 上）
* ROS 中的地图坐标系：左下角为地图原点，x为右，y 为上，z 垂直地面向上

## 2019有用参数

### 比赛状态

* topic：game_status

* msg tyoe: roborts_msgs/GameStatus

* 描述

  ```c++
    1. uint8 PRE_MATCH = 0      //未开始比赛
    2. uint8 SETUP = 1        //3分钟准备阶段
    3. uint8 INIT = 2       //裁判系统自检阶段
    4. uint8 FIVE_SEC_CD = 3    //对战5s倒计时阶段
    5. uint8 ROUND = 4        //对战阶段
    6. uint8 CALCULATION = 5    //比赛结算阶段
    7. uint8 game_status      //存储当前比赛阶段的变量
    8. uint16 remaining_time    //阶段剩余时间
  ```

* 仿真只提供了 4 （正在比赛）与 5（比赛结束）两种比赛状态

* 剩余时间单位为秒

### 比赛结果

* topic：game_result

* msg type: roborts_msgs/GameResult

* 描述

  ```c++
  1. uint8 DRAW = 0			//平局
  2. uint8 RED_WIN = 1		//红方胜利
  3. uint8 BLUE_WIN = 2		//蓝方胜利
  4. uint8 result				//存储比赛结果的变量
  ```

### 比赛目前机器人生存状态

* topic: game_survivor

* msg type: roborts_msgs/GameSurvivor

* 描述

  ```c++
  1. bool red3	//True = 红1血量不为0 False = 红1血量为0
  2. bool red4	//True = 红2血量不为0 False = 红2血量为0
  3. bool blue3	//True = 蓝1血量不为0 False = 蓝1血量为0
  4. bool blue4	//True = 蓝2血量不为0 False = 蓝2血量为0
  ```

### 机器人状态

* topic: robot_status

* msg type: roborts_msgs/RobotStatus

* 描述

  ```c++
  #robot status
  uint8 id                   //机器人ID，是机器人的所属与种类
      					   //红1、红2、蓝1、蓝2的id号分别为3、4、13、14
  uint8 level                //机器人等级（无用）
  uint16 remain_hp           //剩余血量
  uint16 max_hp              //上限血量
  uint16 heat_cooling_limit  //枪口热量上限
  uint16 heat_cooling_rate   //枪口每秒冷却值
  bool gimbal_output         //主控电源gimbal口有电压输出（无用）
  bool chassis_output        //主控电源chassis口有电压输出（无用）
  bool shooter_output        //主控电源shooter口有电压输出（无用）
  ```

* 仿真只能提供：id号、剩余血量、上限血量、枪口热量上限、枪口每秒冷却值，id号见上备注，其他信息为默认值（0 或 False）

### 机器人热量

* topic：robot_heat

* msg type：roborts_msgs/RobotHeat

* 描述

  ```c++
  #robot power and heat data
  uint16 chassis_volt          //底盘电压（无用）
  uint16 chassis_current       //底盘电流（无用）
  float64  chassis_power       //底盘功率（无用）
  uint16 chassis_power_buffer  //底盘功率缓冲（无用）
  uint16 shooter_heat          //枪口热量
  ```

* 只提供 枪口热量

### 机器人收到伤害状态

* topic：robot_damage

* msg type：roborts_msgs/RobotDamage

* 描述

  ```c++
  #robot damage
  uint8 ARMOR = 0         //装甲板被攻击
  uint8 OFFLINE = 1       //模块掉线
  uint8 EXCEED_HEAT = 2   //超热量
  uint8 EXCEED_POWER = 3  //超功率（无用）
  uint8 damage_type       //伤害类型，共以上四种，其中第四种本比赛不涉及到 (4 表示无伤害，且装甲板受攻击优先级更高)
  
  uint8 FORWARD = 0       //前装甲板
  uint8 LEFT = 1          //左装甲板
  uint8 BACKWARD = 2      //后装甲板
  uint8 RIGHT = 3         //右装甲板
  uint8 damage_source     //伤害来源，共以上四种 （4 表示无装甲板受到攻击）
  ```

* 默认 damage_type 和 damage_source 均为 4，即表示未受到攻击；且同时因热量和装甲板扣血时，**优先显示装甲板扣血**

### 队友机器人信息

* topic：partner_msg

* msg type：roborts_msgs/PartnerInformation

* 描述

  ```c++
  #PartnerInformation
  Header header
  int8 status
  bool enemy_detected  //是否检测到敌人,True为检测到
  EnemyInfo[] enemy_info  //敌方信息，具体见下
  uint8 patrol_count   //枪口巡航角度所对应的索引值
  geometry_msgs/PoseStamped partner_pose  //队友位置；尺寸坐标单位为 m，角度为四元数（弧度）
  int32 bullet_num  //子弹数量
  ```

  ```c++
  #EnemyInfo
  geometry_msgs/PoseStamped  enemy_pos //敌人车在世界坐标的位姿;尺寸坐标单位为 m，角度为四元数（弧度）
  int32 num  //检测到的装机板数；仿真时，如果检测到，num 为 1
  ```

* status 不知道含义，发送默认值 0；

* patrol count：当前巡航角度所对应的索引值（枪口相对底盘朝向）巡航角度列表：{70, 45, 20, -20, -45, -70}

### 队友机器人状态

* 内容同[机器人状态](#机器人状态)
* 通过使用**绝对 topic 名称**来获取队友的状态

## 2020 新增参数

### 机器人子弹状态

* topic：BulletVacant

* msg type: roborts_msgs/BulletVacant

* 描述

  ```c++
  bool bullet_vacant  //1为子弹空了，0为还有子弹
  int32 remaining_number //剩余子弹数目
  ```

### Buff 状态

* topic：BuffInfo

* msg type：roborts_msgs/BuffInfo

* 描述

  ```c++
  #BuffInfo
  OneBuffInfo F1 //地图中F1区域buff类型及状态
  OneBuffInfo F2 //地图中F2区域buff类型及状态
  OneBuffInfo F3 //地图中F3区域buff类型及状态
  OneBuffInfo F4 //地图中F4区域buff类型及状态
  OneBuffInfo F5 //地图中F5区域buff类型及状态
  OneBuffInfo F6 //地图中F6区域buff类型及状态
  ```

  ```c++
  #OneBuffInfo
  uint8 BLUE_HP = 0 //蓝色回血
  uint8 BLUE_BULLET= 1 //蓝色加100子弹
  uint8 RED_HP = 2  //红色回血
  uint8 RED_BULLET = 3  //红色加 100 子弹
  uint8 NO_SHOOT = 4  //禁止射击
  uint8 NO_MOVE = 5  //禁止移动
  uint8 type //Buff类型
  bool used //Buff是否已被使用，True 为已使用
  ```

### 机器人被惩罚状态

* topic：RobotPunish

* msg type：roborts_msgs/RobotPunish

* 描述

  ```c++
  #RobotPunish
  uint8 NORMAL = 0  //未受到惩罚
  uint8 NO_SHOOT = 1  //被禁止射击中
  uint8 NO_MOVE = 2  //被禁止移动中
  
  uint8 type  // 被惩罚类型
  float32 remaining_time  // 如被惩罚，该惩罚剩余时间
  ```

## 无用参数

### 防御加成区状态

* topic: field_bonus_status

* msg type: roborts_msgs/BonusStatus

* 描述

  ```c++
  1. uint8 UNOCCUPIED = 0			//防御加成未激活
  2. uint8 BEING_OCCUPIED = 1		//防御加成5s触发激活中
  3. uint8 OCCUPIED = 2			//防御加成已激活
  4. uint8 red_bonus				//存储红方防御加成区状态的变量
  5. uint8 blue_bonus				//存储蓝方防御加成区状态的变量
  ```

* 始终发送 未激活

### 子弹补给区状态

* topic: field_supplier_status

* msg type: roborts_msgs/SupplierStatus

* 描述

  ```c++
  1. uint8 CLOSE = 0			//出弹口关闭
  2. uint8 PREPARING = 1		//子弹准备中
  3. uint8 SUPPLYING = 2		//子弹下落
  4. uint8 status				//存储当前补给站动作的变量
  ```

* 始终发送 出弹口关闭

### 目标敌人

* topic：goal

* msg type：geometry_msgs/PoseStamped

* 该话题的订阅者有两个：

  * blackboard.h中的enemy_sub_。注释名为“给目标点调试用的”，所以很有可能是用命令行直接发布的一个话题，用于测试goal_behavior。

  * global_planner_test.cpp中的goal_sub_。变量传递给global_planner_actionlib_client\_，用于调试全局规划算法。

### 枪口角度控制

* topic：cmd_gimbal_angle

* msg type: roborts_msgs/GimbalAngle

* 描述

  ```c++
  1. bool yaw_mode    //Yaw轴角度控制方式  True = 相对角度 False = 绝对角度
  2. bool pitch_mode    //Pitch轴角度控制方式  True = 相对角度 False = 绝对角度
  3. float64 yaw_angle  //Yaw轴控制角度
  4. float64 pitch_angle  //Pitch轴控制角度
  ```

### 机器人防御 buff 状态

* topic：robot_bonus

* msg type：roborts_msgs/RobotBonus

* 描述

  ```c++
  #robot bonus
  bool bonus  //机器人是否有防御加成，1为有，0为没有
  ```

* 始终发送  无防御加成

### 机器人射击状态

* topic：robot_shoot

* msg type: roborts_msgs::RobotShoot

* 描述

  ```c++
  #robot shoot data
  uint8 frequency  //射频 Hz
  float64 speed    //射速 m/s
  ```

* （**放弃利用里边的信息**，始终发送射频为 6Hz，射速为 18m/s）；子弹数目更新另有 [BulletVacant](#机器人子弹状态)
* 这里的射速射频是裁判系统检测到的，并非控制指令

### 机器人补弹量（无用）

* topic：projectile_supply

* msg type：roborts_msgs/ProjectileSupply

* 描述

  ```c++
  #projectile supply
  uint8 number  //补弹量
  ```

* 补弹量 恒定发送 0
* 直接在 [BulletVacant](#机器人子弹状态) 里边发送可用子弹数
* 这里相当于roborts_dicision节点判断正在子弹补给区时，就让黑板发送补弹指令给到roborts_base节点，roborts_base节点再向裁判系统申请补弹。今年是有子弹加成区，可能是踩到buff区后，裁判系统就会把允许发送弹量增加了
