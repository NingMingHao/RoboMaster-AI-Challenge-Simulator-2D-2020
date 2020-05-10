

  /*******************Enemy Information from roborts_detection*******************/
  void Blackboard::ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr& feedback) {
    if (feedback->detected){
      enemy_detected_ = true;
      last_enemy_disappear_time_ = ros::Time::now();
      // ROS_INFO("Find Enemy!");
      //enemy_info_ = feedback->enemy_info;
      search_count_ = 5;
      enemy_info_.clear();
      for (int i = 0; i < feedback->enemy_info.size(); i++) {
        tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
        geometry_msgs::PoseStamped global_pose_msg;

        global_pose_msg = feedback->enemy_info[i].enemy_pos;
        try
        {

          //tf_ptr_->transformPose("map", camera_pose_msg, global_pose_msg);
            if (GetDistance(global_pose_msg, enemy_pose_)>0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2)
              enemy_pose_ = global_pose_msg;
          roborts_msgs::EnemyInfo enemy_info;
          enemy_info.enemy_pos = global_pose_msg;
          enemy_info.num = feedback->enemy_info[i].num;
          enemy_info_.push_back(enemy_info);
          // std::cout <<"enemy_pose:" << enemy_pose_ << std::endl;
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("tf error when transform enemy pose from camera to map");
        }
      }
    } else {
      enemy_detected_ = false;
      // ROS_INFO(" nnnnnnnnnnnnnnnnFind Enemy!");
    }
    PublishPartnerInformation();
  } 
