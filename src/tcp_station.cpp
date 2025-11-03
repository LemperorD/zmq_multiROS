#include "tcp_drone_station.hpp"

ros::Subscriber takeoff_command_sub_; // 地面到飞机：起飞指令
ros::Subscriber land_command_sub_;    // 地面到飞机：降落或返航
ros::Subscriber mission_sub_; // 地面到飞机：切换任务模式
ros::Subscriber clear_wp_sub_; // 地面到飞机：清除航点
ros::Subscriber ryCtrl_sub_; // 地面到飞机：吊舱角度控制指令

ros::Publisher* pose_pubs = nullptr;
ros::Publisher* vel_pubs = nullptr;
ros::Publisher* battery_pubs = nullptr;
ros::Publisher* state_pubs = nullptr;
ros::Publisher* waypoint_list_pubs = nullptr;
ros::Publisher* gps_pubs = nullptr;
ros::Publisher* ryState_pub_ = nullptr; // 飞机到地面：吊舱当前角度
ros::Publisher* alt_pub_ = nullptr; // 飞机到地面：吊舱当前角度
ros::Publisher* traj_pub_ = nullptr; // 飞机到地面：轨迹

std::vector<ros::Subscriber> waypoint_list_subs_; // 地面到飞机：航点下发
std::vector<ros::Subscriber> ryCtrl_subs_; // 地面到飞机：吊舱角度控制指令

void takeoff_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg); // 地面到飞机：起飞指令
void land_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);    // 地面到飞机：降落或返航指令
void mission_mode_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg); // 地面到飞机：切换任务模式
void clear_wp_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);    // 地面到飞机：清除航点

void ryCtrl_sub_cb(const ruiyan_ros_sdk::RuiyanControl::ConstPtr &msg, int drone_id); // 地面到飞机：吊舱角度控制指令
void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg, int drone_id); // 地面到飞机：航点下发

// void takeoff_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int drone_id); // 地面到飞机：起飞指令
// void land_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int drone_id);    // 地面到飞机：降落或返航指令

void pose_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：位姿
void vel_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：速度
void battery_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：电池状态
void state_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：飞控状态
void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：当前航点列表
void gps_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：GPS消息
void ryState_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：吊舱当前角度
void alt_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：高度信息
void traj_bridge_cb(int ID, ros::SerializedMessage &m); // 飞机到地面：轨迹

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // 参数读取
  nh.param("self_id", self_id_, 16);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 16);
  nh.param("ground_station_num", ground_station_num_, 1);

  // 配置 ID & IP
  id_list_.resize(drone_num_ + ground_station_num_);
  ip_list_.resize(drone_num_ + ground_station_num_);
  for (int i = 0; i < drone_num_ + ground_station_num_; ++i) {
    nh.param((i < drone_num_ ? "drone_ip_" + to_string(i)
                            : "ground_station_ip_" + to_string(i - drone_num_)),
            ip_list_[i], string("127.0.0.1"));
    id_list_[i] = i;
  }

  self_id_in_bridge_ = self_id_;
  if (is_groundstation_) self_id_in_bridge_ = remap_ground_station_id(self_id_);

  bridge.reset(new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));
    
  pose_pubs = new ros::Publisher[drone_num_];
  vel_pubs = new ros::Publisher[drone_num_];
  battery_pubs = new ros::Publisher[drone_num_];
  state_pubs = new ros::Publisher[drone_num_];
  waypoint_list_pubs = new ros::Publisher[drone_num_];
  gps_pubs = new ros::Publisher[drone_num_];
  ryState_pub_ = new ros::Publisher[drone_num_];
  alt_pub_ = new ros::Publisher[drone_num_];
  traj_pub_ = new ros::Publisher[drone_num_];

  for (int i = 0; i < drone_num_; ++i) {
    pose_pubs[i] = nh.advertise<geometry_msgs::PoseStamped>("/drone_"  + std::to_string(i) + "/pose", 10);
    vel_pubs[i] = nh.advertise<geometry_msgs::TwistStamped>("/drone_"  + std::to_string(i) +"/vel" , 10);
    battery_pubs[i] = nh.advertise<sensor_msgs::BatteryState>("/drone_"  + std::to_string(i) +"/battery" , 10);
    state_pubs[i] = nh.advertise<mavros_msgs::State>("/drone_"  + std::to_string(i) +"/state" , 10);
    waypoint_list_pubs[i] = nh.advertise<mavros_msgs::WaypointList>("/drone_"  + std::to_string(i) +"/wplist" , 10);
    gps_pubs[i] = nh.advertise<sensor_msgs::NavSatFix>("/drone_"  + std::to_string(i) +"/gps" , 10);
    ryState_pub_[i] = nh.advertise<ruiyan_ros_sdk::RuiyanState>("/drone_"  + std::to_string(i) +"/ryState" , 10);
    alt_pub_[i] = nh.advertise<std_msgs::Float64>("/drone_"  + std::to_string(i) +"/alt" , 10);
    traj_pub_[i] = nh.advertise<mavros_msgs::Trajectory>("/drone_"  + std::to_string(i) +"/traj" , 10);
  }

  for (int i = 0; i < drone_num_; ++i) {
    std::string topic_name = "/drone_" + std::to_string(i) + "/mission_upload";
    ros::SubscribeOptions ops;
    ops.init<mavros_msgs::WaypointList>(topic_name, 10, boost::bind(&waypoint_list_sub_cb, _1, i));
    ops.transport_hints = ros::TransportHints().tcpNoDelay();
    waypoint_list_subs_.push_back(nh.subscribe(ops));
    ROS_INFO("Subscribed to drone %d: %s", i, topic_name.c_str());
  }

  for (int i = 0; i < drone_num_; ++i) {
    std::string topic_name = "/drone_" + std::to_string(i) + "/RuiyanCtrl";
    ros::SubscribeOptions ops;
    ops.init<ruiyan_ros_sdk::RuiyanControl>(topic_name, 10, boost::bind(&ryCtrl_sub_cb, _1, i));
    ops.transport_hints = ros::TransportHints().tcpNoDelay();
    ryCtrl_subs_.push_back(nh.subscribe(ops));
    ROS_INFO("Subscribed to drone %d: %s", i, topic_name.c_str());
  }

  for (int i = 0; i < drone_num_; ++i) {
    if (bridge->register_callback(i, "/pose_tcp_" + std::to_string(i), pose_bridge_cb))
    {
      ROS_INFO("Register pose callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/vel_tcp_" + std::to_string(i), vel_bridge_cb))
    {
      ROS_INFO("Register vel callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/battery_tcp_" + std::to_string(i), battery_bridge_cb))
    {
      ROS_INFO("Register battery callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/state_tcp_" + std::to_string(i), state_bridge_cb))
    {
      ROS_INFO("Register state callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/wplist_tcp_" + std::to_string(i), waypoint_list_bridge_cb))
    {
      ROS_INFO("Register waypoint list callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/gps_tcp_" + std::to_string(i), gps_bridge_cb))
    {
      ROS_INFO("Register gps callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/ryState_tcp_" + std::to_string(i), ryState_bridge_cb))
    {
      ROS_INFO("Register ruiyan state callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/alt_tcp_" + std::to_string(i), alt_bridge_cb))
    {
      ROS_INFO("Register alt callback for drone %d", i);
    }
    if (bridge->register_callback(i, "/traj_tcp_" + std::to_string(i), traj_bridge_cb))
    {
      ROS_INFO("Register traj callback for drone %d", i);
    }
  }

  takeoff_command_sub_ = nh.subscribe("/takeoff_trigger", 10, takeoff_command_sub_cb, ros::TransportHints().tcpNoDelay());
  land_command_sub_ = nh.subscribe("/land_trigger", 10, land_command_sub_cb, ros::TransportHints().tcpNoDelay());
  mission_sub_ = nh.subscribe("/mission_trigger", 10, mission_mode_sub_cb, ros::TransportHints().tcpNoDelay());
  clear_wp_sub_ = nh.subscribe("/clear_trigger", 10, clear_wp_sub_cb, ros::TransportHints().tcpNoDelay());

  
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  if (bridge) {
    bridge->StopThread();
    bridge.reset();
  }

  delete[] pose_pubs;
  delete[] vel_pubs;
  delete[] battery_pubs;
  delete[] state_pubs;
  delete[] waypoint_list_pubs;
  delete[] gps_pubs;
  delete[] ryState_pub_;
  delete[] alt_pub_;
  return 0;
}

void takeoff_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_ERROR("Takeoff command received, sending to all drones.");
  send_to_all_drone_except_me("/takeoff_tcp", *msg);
}

void land_command_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Land command received, sending to all drones.");
  send_to_all_drone_except_me("/land_tcp", *msg);
}

void mission_mode_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("mission mode setting command received, sending to all drones.");
  send_to_all_drone_except_me("/mission_tcp", *msg);
}

void clear_wp_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("clearing wplist command received, sending to all drones.");
  send_to_all_drone_except_me("/clear_tcp", *msg);
}

void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg, int drone_id) {
  bridge->send_msg_to_one(drone_id, "/wplist_"+std::to_string(drone_id), *msg);
  ROS_INFO("send wplist to drone %d", drone_id);
}

void ryCtrl_sub_cb(const ruiyan_ros_sdk::RuiyanControl::ConstPtr &msg, int drone_id) {
  bridge->send_msg_to_one(drone_id, "/ryCtrl_"+std::to_string(drone_id), *msg);
  ROS_INFO("send ruiyan control drone %d", drone_id);
}

void pose_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped pose_msg_;
  ros::serialization::deserializeMessage(m, pose_msg_);
  pose_pubs[ID].publish(pose_msg_);
}

void vel_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::TwistStamped vel_msg_;
  ros::serialization::deserializeMessage(m, vel_msg_);
  vel_pubs[ID].publish(vel_msg_);
}

void battery_bridge_cb(int ID, ros::SerializedMessage &m) {
  sensor_msgs::BatteryState battery_msg_;
  ros::serialization::deserializeMessage(m, battery_msg_);
  battery_pubs[ID].publish(battery_msg_);
}

void state_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::State state_msg_;
  ros::serialization::deserializeMessage(m, state_msg_);
  state_pubs[ID].publish(state_msg_);
}

void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::WaypointList wplist_msg_;
  ros::serialization::deserializeMessage(m, wplist_msg_);
  waypoint_list_pubs[ID].publish(wplist_msg_);
}

void gps_bridge_cb(int ID, ros::SerializedMessage &m) {
  sensor_msgs::NavSatFix gps_msg_;
  ros::serialization::deserializeMessage(m, gps_msg_);
  gps_pubs[ID].publish(gps_msg_);
}

void ryState_bridge_cb(int ID, ros::SerializedMessage &m) {
  ruiyan_ros_sdk::RuiyanState ryState_msg_;
  ros::serialization::deserializeMessage(m, ryState_msg_);
  ryState_pub_[ID].publish(ryState_msg_);
}

void alt_bridge_cb(int ID, ros::SerializedMessage &m) {
  std_msgs::Float64 alt_msg_;
  ros::serialization::deserializeMessage(m, alt_msg_);
  alt_pub_[ID].publish(alt_msg_);
}

void traj_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::Trajectory traj_msg_;
  ros::serialization::deserializeMessage(m, traj_msg_);
  traj_pub_[ID].publish(traj_msg_);
}