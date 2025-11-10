#include "tcp_drone_station.hpp"
#include <thread>

std::mutex wp_mutex;
std::condition_variable wp_cv;
std::atomic<bool> waypoint_received(false);

ros::Publisher takeoff_command_pub_; // 地面到飞机：起飞指令
ros::Publisher land_command_pub_;    // 地面到飞机：降落或返航指令
ros::Publisher mission_mode_pub_; //地面到飞机：切换任务模式
ros::Publisher clear_wp_pub_; //地面到飞机：清除航点
ros::Publisher ryCtrl_pub_; //地面到飞机：吊舱角度控制指令

ros::ServiceClient waypoint_client; // 地面到飞机：航点下发
ros::ServiceClient set_current_client; // 地面到飞机：输入航点后设置当前航点为0
ros::ServiceClient set_mission_client; // 地面到飞机：输入航点后设置为任务模式

mavros_msgs::WaypointPush waypoint_push_srv;
mavros_msgs::WaypointSetCurrent set_current_srv;
mavros_msgs::SetMode set_mission_srv;

ros::Subscriber pose_sub_; // 飞机到地面：位姿
ros::Subscriber vel_sub_; // 飞机到地面：速度
ros::Subscriber battery_sub_; // 飞机到地面：电池状态
ros::Subscriber state_sub_; // 飞机到地面：飞控状态
ros::Subscriber waypoint_list_sub_; // 飞机到地面：当前航点列表
ros::Subscriber gps_sub_ ; //飞机到地面：GPS信息
ros::Subscriber ryState_sub_; // 飞机到地面：吊舱当前角度
ros::Subscriber alt_sub_; // 飞机到地面：吊舱当前角度
ros::Subscriber traj_sub_; // 飞机到地面：轨迹
ros::Subscriber fov_sub_; // 飞机到地面：相机视场角

void takeoff_command_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：起飞指令
void land_command_bridge_cb(int ID, ros::SerializedMessage &m);    // 地面到飞机：降落或返航指令
void mission_mode_bridge_cb(int ID, ros::SerializedMessage &m);    // 地面到飞机：降落或返航指令
void clear_wp_bridge_cb(int ID, ros::SerializedMessage &m);    // 地面到飞机：清除航点
void ryCtrl_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：吊舱角度控制指令

void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m); // 地面到飞机：航点下发

void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg); // 飞机到地面：位姿
void vel_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg); // 飞机到地面：速度
void battery_sub_cb(const sensor_msgs::BatteryState::ConstPtr &msg); // 飞机到地面：电池状态
void state_sub_cb(const mavros_msgs::State::ConstPtr &msg); // 飞机到地面：飞控状态
void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg); // 飞机到地面：当前航点列表
void gps_sub_cb(const sensor_msgs::NavSatFix::ConstPtr &msg); //飞机到地面：GPS信息
void ryState_sub_cb(const ruiyan_ros_sdk::RuiyanState::ConstPtr &msg); // 飞机到地面：吊舱当前角度
void alt_sub_cb(const std_msgs::Float64::ConstPtr &msg); // 飞机到地面：吊舱当前角度
void traj_sub_cb(const mavros_msgs::Trajectory::ConstPtr &msg); // 飞机到地面：轨迹
void fov_sub_cb(const visualization_msgs::Marker::ConstPtr &msg); // 飞机到地面：相机视场角

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  // 参数读取
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 1);
  nh.param("ground_station_num", ground_station_num_, 0);

  nh.param("pose_freq",  pose_freq_, 10.0);
  nh.param("gps_freq",  gps_freq_, 10.0);
  nh.param("vel_freq",  vel_freq_, 10.0);
  nh.param("battery_freq",  battery_freq_, 10.0);
  nh.param("state_freq",  state_freq_, 10.0);
  nh.param("ryState_freq",  ryState_freq_, 10.0);
  nh.param("alt_freq",  alt_freq_, 10.0);
  nh.param("waypoint_freq",  waypoint_freq_, 1.0);
  nh.param("traj_freq",  traj_freq_, 1.0);
  nh.param("fov_freq",  fov_freq_, 1.0);

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

  // 注册回调函数
  pose_sub_ = nh.subscribe("/mavros/local_position/pose", 10, pose_sub_cb, ros::TransportHints().tcpNoDelay());
  vel_sub_ = nh.subscribe("/mavros/local_position/velocity_local", 10, vel_sub_cb, ros::TransportHints().tcpNoDelay());
  battery_sub_ = nh.subscribe("/mavros/battery", 10, battery_sub_cb, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh.subscribe("/mavros/state", 10, state_sub_cb, ros::TransportHints().tcpNoDelay());
  waypoint_list_sub_ = nh.subscribe("/mavros/mission/waypoints", 10, waypoint_list_sub_cb, ros::TransportHints().tcpNoDelay());
  gps_sub_ = nh.subscribe("/mavros/global_position/global", 10, gps_sub_cb, ros::TransportHints().tcpNoDelay());
  alt_sub_ = nh.subscribe("/mavros/global_position/rel_alt", 10, alt_sub_cb, ros::TransportHints().tcpNoDelay());
  ryState_sub_ = nh.subscribe("/state_info", 10, ryState_sub_cb, ros::TransportHints().tcpNoDelay());
  traj_sub_ = nh.subscribe("/mavros/trajectory/generated", 10, traj_sub_cb, ros::TransportHints().tcpNoDelay());
  fov_sub_ = nh.subscribe("/fov", 10, fov_sub_cb, ros::TransportHints().tcpNoDelay());

  waypoint_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
  set_current_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("/mavros/mission/set_current");

  // if (bridge->register_callback(self_id_in_bridge_, "/takeoff_tcp", takeoff_command_bridge_cb))
  // {

  // }

  // if (bridge->register_callback(self_id_in_bridge_, "/land_tcp", land_command_bridge_cb))
  // {

  // }

  std::thread waypoint_thread([&]() {
    ros::Rate r(1); // 防止空循环占用CPU
    while (ros::ok()) {
      // 等待新航点到达
      {
        std::unique_lock<std::mutex> lock(wp_mutex);
        wp_cv.wait(lock, [] { return waypoint_received.load(); });
      }

      ROS_INFO("[Drone %d] Received new waypoint list, uploading...", self_id_);

      if (waypoint_client.call(waypoint_push_srv)) {
        if (waypoint_push_srv.response.success)
          ROS_INFO("[Drone %d] Waypoint upload success (%lu waypoints)",
                   self_id_, waypoint_push_srv.request.waypoints.size());
        else
          ROS_WARN("[Drone %d] Waypoint upload failed", self_id_);
      } else {
        ROS_ERROR("[Drone %d] Failed to call MAVROS waypoint push service", self_id_);
      }

      if (set_current_client.call(set_current_srv)) {
        if (set_current_srv.response.success)
          ROS_INFO("[Drone %d] set current 0 success", self_id_);
        else
          ROS_WARN("[Drone %d] set current 0 failed", self_id_);
      } else {
        ROS_ERROR("[Drone %d] Failed to call MAVROS set current service", self_id_);
      }

      if (set_mission_client.call(set_mission_srv)) {
        if (set_mission_srv.response.mode_sent)
          ROS_INFO("[Drone %d] set mission mode success", self_id_);
        else
          ROS_WARN("[Drone %d] set mission mode failed", self_id_);
      } else {
        ROS_ERROR("[Drone %d] Failed to call MAVROS set mode service", self_id_);
      }

      // ✅ 上传完成后清除标志以等待下一次
      waypoint_received = false;
      r.sleep();
    }
  });

  if(bridge->register_callback(drone_num_, "/wplist_"+std::to_string(self_id_), waypoint_list_bridge_cb))
  {
    ROS_INFO("[Drone %d] Waiting for waypoint list from bridge...", self_id_);
  }

  if(bridge->register_callback(drone_num_, "/ryCtrl_"+std::to_string(self_id_), ryCtrl_bridge_cb))
  {
    ROS_INFO("[Drone %d] Waiting for ruiyan control from bridge...", self_id_);
  }

  bridge->register_callback(drone_num_, "/takeoff_tcp", takeoff_command_bridge_cb);
  bridge->register_callback(drone_num_, "/land_tcp", land_command_bridge_cb);
  bridge->register_callback(drone_num_, "/mission_tcp", mission_mode_bridge_cb);
  bridge->register_callback(drone_num_, "/clear_tcp", clear_wp_bridge_cb);
  bridge->register_callback(drone_num_, "/ryCtrl_tcp", ryCtrl_bridge_cb);

  takeoff_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/trigger_drone", 10);
  land_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/trigger_land", 10);
  mission_mode_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/trigger_mission", 10);
  clear_wp_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/trigger_clear", 10);
  ryCtrl_pub_ = nh.advertise<ruiyan_ros_sdk::RuiyanControl>("/RuiyanControl", 10);

  ros::AsyncSpinner spinner(4); // 多线程处理回调
  spinner.start();
  ros::waitForShutdown();
  bridge->StopThread();
  return 0;
}

void takeoff_command_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Received takeoff command");
  takeoff_command_pub_.publish(cmd);
}

void land_command_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Received land command");
  land_command_pub_.publish(cmd);
}

void mission_mode_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("change to mission mode");
  mission_mode_pub_.publish(cmd);
}

void clear_wp_bridge_cb(int ID, ros::SerializedMessage &m) {
  geometry_msgs::PoseStamped cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Clearing wplist...");
  clear_wp_pub_.publish(cmd);
}

void waypoint_list_bridge_cb(int ID, ros::SerializedMessage &m) {
  mavros_msgs::WaypointList wp;
  ros::serialization::deserializeMessage(m, wp);
  ROS_INFO("Received waypoint list");
  {
    std::lock_guard<std::mutex> lock(wp_mutex);
    waypoint_push_srv.request.waypoints = wp.waypoints;
    set_current_srv.request.wp_seq = 0;
    set_mission_srv.request.custom_mode = "AUTO.MISSION";
    waypoint_received = true;  // ✅ 标记已收到航点
  }
  wp_cv.notify_all();  // ✅ 唤醒等待线程
}

void ryCtrl_bridge_cb(int ID, ros::SerializedMessage &m) {
  ruiyan_ros_sdk::RuiyanControl cmd;
  ros::serialization::deserializeMessage(m, cmd);
  ROS_INFO("Received Ruiyan Ctrl command");
  ryCtrl_pub_.publish(cmd);
}

void pose_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  pose_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/pose_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void vel_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  vel_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/vel_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void battery_sub_cb(const sensor_msgs::BatteryState::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  battery_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/battery_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void state_sub_cb(const mavros_msgs::State::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  state_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/state_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void waypoint_list_sub_cb(const mavros_msgs::WaypointList::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  waypoint_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/wplist_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void gps_sub_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  gps_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/gps_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void ryState_sub_cb(const ruiyan_ros_sdk::RuiyanState::ConstPtr &msg) {
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * ryState_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/ryState_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void alt_sub_cb(const std_msgs::Float64::ConstPtr &msg){
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  alt_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/alt_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void traj_sub_cb(const mavros_msgs::Trajectory::ConstPtr &msg){
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  traj_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/traj_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}

void fov_sub_cb(const visualization_msgs::Marker::ConstPtr &msg){
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() *  fov_freq_ < 1.0) return;
  t_last = t_now;
  std::string topic = "/fov_tcp_" + std::to_string(self_id_in_bridge_);
  send_to_all_groundstation_except_me(topic, *msg);
}