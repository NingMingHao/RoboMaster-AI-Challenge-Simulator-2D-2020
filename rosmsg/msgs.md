1. enemy_sub_: \<geometry_msgs::PoseStamped\> "goal"

    把所有的文件都搜索了一遍，这个"goal"找不到发布者。blackboard.h文件中在该订阅者上面写了注释Enemy fake pose（假的敌方坐标）。

    该话题的订阅者有两个：

    1. blackboard.h中的enemy_sub_。注释名为“给目标点调试用的”，所以很有可能是用命令行直接发布的一个话题，用于测试goal_behavior。
    2. global_planner_test.cpp中的goal_sub_。变量传递给global_planner_actionlib_client\_，用于调试全局规划算法。

2. gimbal_angle_sub_: \<roborts_msgs::GimbalAngle\> "cmd_gimbal_angle"

    由behavior_test.cpp中创建的behavior_test_node节点发布，

    .cpp文件中初始化GimbalExecutor对象，该对象初始化时创建该消息的发布者

    ```c++
    cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
    ```

    ```c++
    1. bool yaw_mode		//Yaw轴角度控制方式	True = 相对角度 False = 绝对角度
    2. bool pitch_mode		//Pitch轴角度控制方式	True = 相对角度 False = 绝对角度
    3. float64 yaw_angle	//Yaw轴控制角度
    4. float64 pitch_angle	//Pitch轴控制角度
    ```

3. game_status_sub_: \<roborts_msgs::GameStatus\> "game_status"

    由roborts_base_node.cpp中创建的roborts_base_node节点发布，

    .cpp文件中初始化RefereeSystem对象，该对象初始化时创建该消息的发布者

    ```c++
    ros_game_status_pub_ = ros_nh_.advertise<roborts_msgs::GameStatus>("game_status", 30);
    ```

    ```c++
    1. uint8 PRE_MATCH = 0			//未开始比赛
    2. uint8 SETUP = 1				//3分钟准备阶段
    3. uint8 INIT = 2				//裁判系统自检阶段
    4. uint8 FIVE_SEC_CD = 3		//对战5s倒计时阶段
    5. uint8 ROUND = 4				//对战阶段
    6. uint8 CALCULATION = 5		//比赛结算阶段
    7. uint8 game_status			//存储当前比赛阶段的变量
    8. uint16 remaining_time		//阶段剩余时间
    ```

4. game_result_sub_: \<roborts_msgs::GameResult\> "game_result"

    创建地点同3

    ```c++
    ros_game_result_pub_ = ros_nh_.advertise<roborts_msgs::GameResult>("game_result", 30);
    ```

    ```c++
    1. uint8 DRAW = 0			//平局
    2. uint8 RED_WIN = 1		//红方胜利
    3. uint8 BLUE_WIN = 2		//蓝方胜利
    4. uint8 result				//存储比赛结果的变量
    ```

5. game_survival_sub_: \<roborts_msgs::GameSurvivor\> "game_survivor"

    创建地点同3

    ```c++
    ros_game_survival_pub_ = ros_nh_.advertise<roborts_msgs::GameSurvivor>("game_survivor", 30);
    ```

    ```c++
    1. bool red3	//True = 红1血量不为0 False = 红1血量为0
    2. bool red4	//True = 红2血量不为0 False = 红2血量为0
    3. bool blue3	//True = 蓝1血量不为0 False = 蓝1血量为0
    4. bool blue4	//True = 蓝2血量不为0 False = 蓝2血量为0
    ```

6. bonus_status_sub_: \<roborts_msgs::BonusStatus\> "field_bonus_status"

    创建地点同3

    ```c++
    ros_bonus_status_pub_ = ros_nh_.advertise<roborts_msgs::BonusStatus>("field_bonus_status", 30);
    ```

    ```c++
    1. uint8 UNOCCUPIED = 0			//防御加成未激活
    2. uint8 BEING_OCCUPIED = 1		//防御加成5s触发激活中
    3. uint8 OCCUPIED = 2			//防御加成已激活
    4. uint8 red_bonus				//存储红方防御加成区状态的变量
    5. uint8 blue_bonus				//存储蓝方防御加成区状态的变量
    ```

7. supplier_status_sub_: \<roborts_msgs::SupplierStatus\> "field_supplier_status"

    创建地点同3

    ```c++
    ros_supplier_status_pub_ = ros_nh_.advertise<roborts_msgs::SupplierStatus>("field_supplier_status", 30);
    ```

    ```c++
    1. uint8 CLOSE = 0			//出弹口关闭
    2. uint8 PREPARING = 1		//子弹准备中
    3. uint8 SUPPLYING = 2		//子弹下落
    4. uint8 status				//存储当前补给站动作的变量
    ```

    

