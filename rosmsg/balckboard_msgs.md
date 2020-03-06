```c++
ros::NodeHandle rviz_nh("move_base_simple");   
1.  enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &Blackboard::GoalCallback, this);
ros::NodeHandle nh;
2. gimbal_angle_sub_= nh.subscribe<roborts_msgs::GimbalAngle>("cmd_gimbal_angle",100 , &Blackboard::GimbalCallback, this);  /****************/
3. game_status_sub_ = nh.subscribe<roborts_msgs::GameStatus>("game_status",30 , &Blackboard::GameStatusCallback, this);
4. game_result_sub_ = nh.subscribe<roborts_msgs::GameResult>("game_result",30 , &Blackboard::GameResultCallback, this);
5. game_survival_sub_ = nh.subscribe<roborts_msgs::GameSurvivor>("game_survivor",30 , &Blackboard::GameSurvivorCallback, this);
6. bonus_status_sub_ = nh.subscribe<roborts_msgs::BonusStatus>("field_bonus_status",30 , &Blackboard::BonusStatusCallback, this);
7. supplier_status_sub_ = nh.subscribe<roborts_msgs::SupplierStatus>("field_supplier_status",30 , &Blackboard::SupplierStatusCallback, this);
8. robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>("robot_status",30 , &Blackboard::RobotStatusCallback, this);
9. robot_heat_sub_ = nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",30 , &Blackboard::RobotHeatCallback, this);
10. robot_bonus_sub_ = nh.subscribe<roborts_msgs::RobotBonus>("robot_bonus",30 , &Blackboard::RobotBonusCallback, this);
11. robot_damage_sub_ = nh.subscribe<roborts_msgs::RobotDamage>("robot_damage",30 , &Blackboard::RobotDamageCallback, this);
12. robot_shoot_sub_ = nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot",30 , &Blackboard::RobotShootCallback, this);
13. projectile_supply_pub_ = nh.advertise<roborts_msgs::ProjectileSupply>("projectile_supply", 1);
14. bullet_vacant_sub_ = nh.subscribe<roborts_msgs::BulletVacant>("BulletVacant",30 , &Blackboard::BulletVacantCallback, this);

  std::string partner_topic_sub = "/" + partner_name + "/partner_msg";
15. partner_sub_ = nh.subscribe<roborts_msgs::PartnerInformation>(partner_topic_sub, 1, &Blackboard::PartnerCallback, this);
   
  std::string partner_status_topic = "/" + partner_name + "/robot_status";
16. partner_robot_status_sub_ = nh.subscribe<roborts_msgs::RobotStatus>(partner_status_topic, 30, &Blackboard::PartnerRobotStatusCallback, this);


```

共16个sub，对应话题名一致即可，16个话题分别订阅来自

goal 

cmd_gimbal_angle

game_status

game_result

game_survivor

field_bonus_status

field_supplier_status

robot_status

robot_heat

robot_bonus

robot_damage

robot_shoot

projectile_supply

BulletVacant

"/" + partner_name + "/partner_msg" （可认为是4个话题）

"/" + partner_name + "/robot_status" 同上

每个话题对应的msg见上文代码，对应格式在roborts_msgs包之中，pub的缓冲区大小与sub对应上就可以