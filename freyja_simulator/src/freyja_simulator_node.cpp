#include <cstdio>
#include <thread>
#include <iostream>
#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <atomic>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"

#include <visualization_msgs/msg/marker_array.hpp>

#include "fast_approximate_math.hpp"

typedef geometry_msgs::msg::TransformStamped TFStamped;
typedef geometry_msgs::msg::Twist BodyTwist;
typedef nav_msgs::msg::Odometry   Odom;



struct GenericRobot
{
  double xpos_, ypos_, theta_;
  double xvel_, yvel_, omega_;
  double xvel_tgt_, yvel_tgt_, omega_body_;

  int unique_id_;
  std::string name_;
  int robot_type_;      // 0: diffdrive, 1: holonimic
  
  volatile bool is_ok_;
  std::atomic_flag keep_alive_;

  // custom robot properties
  const double MAX_LINEAR_SPD_=2.0;
  const double MAX_ANGULAR_SPD_=2.0;
  const double MAX_LINEAR_ACC_ = 1.0;

  public:
    GenericRobot( int, std::string, int );
    // forbid copy
    GenericRobot( const GenericRobot& ) = delete;
    GenericRobot& operator=( const GenericRobot& ) = delete;
    // allow move
    GenericRobot( GenericRobot&& d): unique_id_(d.unique_id_), name_(std::move(d.name_)), robot_type_(d.robot_type_)
    {
      xpos_ = d.xpos_; ypos_ = d.ypos_; theta_ = d.theta_;
      xvel_ = d.xvel_; yvel_ = d.yvel_; omega_ = d.omega_;
      xvel_tgt_ = d.xvel_tgt_;
      yvel_tgt_ = d.yvel_tgt_;
      omega_body_ = d.omega_body_;
    }
    GenericRobot& operator=( GenericRobot&& ) = default;
    // explicit destructor for handling shutdown
    ~GenericRobot();

    void initialise_stopped( const double& _x, const double& _y, const double& _th );
    void setVelocity( const double& _vx, const double& _vy, const double& _w );
    void setBodyVelocity( const double&, const double&, const double& );
    void getWorldVelocity( double& _vx, double& _vy, double& _w) const
      { _vx = xvel_; _vy = yvel_; _w = omega_; }
    void getWorldPosition( double& _px, double& _py, double& _th ) const
      { _px = xpos_; _py = ypos_; _th = theta_; }
    
    void alert_problem() { is_ok_ = false; }
    void terminate() { keep_alive_.clear(std::memory_order_release); }
    
    void moveRobot( const double& dt );
    void manager_process();

};

GenericRobot::~GenericRobot()
{
  terminate();
}

GenericRobot::GenericRobot( int _uid, std::string _n, int _type ) : 
                                    unique_id_(_uid),
                                    name_(std::move(_n)),
                                    robot_type_(_type)
{
  xpos_ = ypos_ = theta_ = std::nan("");
  xvel_ = yvel_ = omega_ = std::nan("");
  is_ok_ = false;
}
void GenericRobot::initialise_stopped( const double& _x, const double& _y, const double& _th )
{
  xpos_ = _x;
  ypos_ = _y;
  theta_ = _th;
  xvel_ = yvel_ = omega_ = 0.0;
  xvel_tgt_ = yvel_tgt_ = omega_body_ = 0.0;
  
  is_ok_ = true;
  keep_alive_.test_and_set();
  printf( "Robot instance: %s(%d), pos: [%0.3f, %0.3f, %0.3f], vel: [%0.2f, %0.2f, %0.2f]\n",
              name_.c_str(), unique_id_, xpos_, ypos_, theta_, xvel_, yvel_, omega_ );
}

void GenericRobot::setBodyVelocity( const double& body_vx, const double& body_vy, const double& body_w )
{
  xvel_tgt_ = std::min( MAX_LINEAR_SPD_, std::max(-MAX_LINEAR_SPD_, body_vx) );
  yvel_tgt_ = std::min( MAX_LINEAR_SPD_, std::max(-MAX_LINEAR_SPD_, body_vy) );
  omega_body_ = std::min( MAX_ANGULAR_SPD_, std::max(-MAX_ANGULAR_SPD_, body_w) );
}
void GenericRobot::moveRobot( const double& dt )
{
  double new_theta = 0.0;
  if( robot_type_ == 0 )
  {
    // diff-drive system using circle eqns.
    omega_ = omega_body_;
    new_theta = fast_approx::constrainAngleRad( theta_ + omega_*dt );
    if( fast_approx::fastabs(omega_) < 0.005 )
    {
      xvel_ = xvel_tgt_*fast_approx::cosine( new_theta );
      yvel_ = xvel_tgt_*fast_approx::sine( new_theta );
    }
    else
    {
      double r = (xvel_tgt_/omega_);
      xvel_ = -r*( fast_approx::sine(theta_) - fast_approx::sine(new_theta) )/dt;
      yvel_ = r * ( fast_approx::cosine(theta_) - fast_approx::cosine(new_theta) )/dt;
    }
  }
  else if( robot_type_ == 1 )
  {
    // holonomic robot point
    new_theta = fast_approx::constrainAngleRad( theta_ + omega_*dt );
    double ax = std::min( MAX_LINEAR_ACC_, std::max(-MAX_LINEAR_ACC_, (xvel_tgt_ - xvel_)/dt) );
    double ay = std::min( MAX_LINEAR_ACC_, std::max(-MAX_LINEAR_ACC_, (yvel_tgt_ - yvel_)/dt) );
    xvel_ += ax * dt;
    yvel_ += ay * dt;
  }
  xpos_ += xvel_*dt;
  ypos_ += yvel_*dt;
  theta_ = new_theta; 
}

void GenericRobot::manager_process()
{
  const double dt = 0.01;
  const int dt_ms = std::round(dt*1000.0);
  keep_alive_.test_and_set();
  printf( "Starting manager process for: %s(%d)\n", name_.c_str(), unique_id_ );

  while( keep_alive_.test_and_set() )
  {
    if( is_ok_ )
    {
      moveRobot(dt);
      std::this_thread::sleep_for( std::chrono::milliseconds(dt_ms) );
    }
    else
    {
      setBodyVelocity( 0.0, 0.0, 0.0 );
      std::this_thread::sleep_for( std::chrono::milliseconds(2*dt_ms) );
      is_ok_ = true;
    }
  }
  printf( "Terminating manager proces for: %s(%d)\n", name_.c_str(), unique_id_ );
}





/* Main simulation interface */

class FreyjaSimulator : public rclcpp::Node
{
  int num_robots_;
  std::vector<long int> robot_num_range_;
  std::vector<double> spawn_limits_xy_;
  
  double sim_step_;
  double topic_step_;


  // robots and managers
  std::vector<GenericRobot> robots_;
  std::vector<std::thread> robot_mgrs_;
  bool enable_collisions_;
  visualization_msgs::msg::Marker robot_markers_;

  // global visualization obstacles
  std::vector<double> obst_pos_list_;
  visualization_msgs::msg::MarkerArray obst_markers_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<BodyTwist>::SharedPtr> cmd_subs_;
  std::vector<rclcpp::Publisher<Odom>::SharedPtr> odom_pubs_;
  std::vector<TFStamped> all_tforms_;
  
  public:
    FreyjaSimulator();
    ~FreyjaSimulator();
    rclcpp::TimerBase::SharedPtr tf_timer_;
    void timerTfCallback() __attribute__((hot));

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_robotmarker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_obstmarker_pub_;
    void simulation_setup();
    
    void create_robots( int );
    void proper_shutdown();
    bool collides(const GenericRobot&, const GenericRobot& );

};

FreyjaSimulator::~FreyjaSimulator()
{
}

FreyjaSimulator::FreyjaSimulator() : Node( "freyja_sim" )
{
  declare_parameter<std::vector<long int>>( "robot_num_range", std::vector<long int>({3, 7}) );
  declare_parameter<std::vector<double>>( "spawn_limits_xy", std::vector<double>({-2.0, 2.0}) );
  declare_parameter<double>("sim_rate", 50.0);
  declare_parameter<double>("topic_rate", 30.0);
  declare_parameter<std::vector<double>>( "team_color", std::vector<double>({1.0, 0.0, 0.0}) );
  declare_parameter<int>( "team_id", 0 );
  declare_parameter<bool>( "enable_collisions", false );
  declare_parameter<std::string>( "robots_type", "diffdrive" );
  declare_parameter<std::vector<double>>( "obst_pos_list", std::vector<double>({-1.0}) );
  
  double refresh_rate, topic_rate;
  std::string robot_type_str;
  int robots_type = 0;
  get_parameter( "robot_num_range", robot_num_range_ );
  get_parameter( "spawn_limits_xy", spawn_limits_xy_ );
  get_parameter( "sim_rate", refresh_rate );
  get_parameter( "topic_rate", topic_rate );
  get_parameter( "enable_collisions", enable_collisions_ );
  get_parameter( "robots_type", robot_type_str );
  get_parameter( "obst_pos_list", obst_pos_list_ );

  // pre-setup
  num_robots_ = int( robot_num_range_[1] - robot_num_range_[0] + 1 );
  printf( "Number of robots: %d\n", num_robots_ );
  assert( num_robots_ > 0 );
  cmd_subs_.resize(num_robots_);
  all_tforms_.resize(num_robots_);
  odom_pubs_.resize(num_robots_);

  if( robot_type_str == "diffdrive" )
    robots_type = 0;
  else if( robot_type_str == "holonomic" )
    robots_type = 1;

  // instantiate all robots
  create_robots( robots_type );

  // set up broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // set up visualization markers
  rviz_robotmarker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/global_robot_markers", rclcpp::QoS(1));
  rviz_obstmarker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/global_obst_markers", rclcpp::QoS(1).transient_local());

  // set up extra pieces of simulation
  simulation_setup();

  // set up a timer process
  topic_step_ = 1.0/topic_rate;
  sim_step_ = 1.0/refresh_rate;
  tf_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<double>(sim_step_),
                                    std::bind(&FreyjaSimulator::timerTfCallback, this) );
  RCLCPP_INFO( get_logger(), "Simulator Ready!" );
}

void FreyjaSimulator::create_robots( int robots_type )
{
  std::random_device rd;
  std::default_random_engine rand_engine(rd());
  std::uniform_real_distribution<> xpos_distrib(spawn_limits_xy_[0], spawn_limits_xy_[1]);
  std::uniform_real_distribution<> ypos_distrib(spawn_limits_xy_[0], spawn_limits_xy_[1]);
  std::uniform_real_distribution<> theta_dist(-3.14, 3.14);
  std::uniform_int_distribution<> uid_dist(200,300);

  int idx = 0;
  for( int r=robot_num_range_[0]; r<=robot_num_range_[1]; r++, idx++ )
  {
    double x = xpos_distrib(rand_engine);
    double y = ypos_distrib(rand_engine);
    double th = theta_dist(rand_engine);
    int uid = uid_dist(rand_engine);
    std::string rname = "robot" + std::to_string(r);
    robots_.emplace_back( uid, rname, robots_type );
    robots_[idx].initialise_stopped( x, y, th );
    // create subscriber
    cmd_subs_[idx] = create_subscription<BodyTwist> ( rname + "/cmd_vel", 1,
                            [this,idx](const BodyTwist::ConstSharedPtr msg)
                            {robots_[idx].setBodyVelocity(msg->linear.x, msg->linear.y, msg->angular.z);} );
    // create publisher
    odom_pubs_[idx] = create_publisher<Odom> ( rname + "/odometry", 1 );
  }
  printf( "All robots created. Starting managers..\n" );
  for(int idx=0; idx<num_robots_; idx++ )
    robot_mgrs_.push_back( std::move(std::thread(&GenericRobot::manager_process, &robots_[idx])) );
}

void FreyjaSimulator::simulation_setup()
{
  int team_id;
  std::vector<double> marker_rgb;
  get_parameter( "team_color", marker_rgb );
  get_parameter( "team_id", team_id );

  // set up visualisation markers for robots
  robot_markers_.color.a = 0.42;
  robot_markers_.color.r = marker_rgb[0];
  robot_markers_.color.g = marker_rgb[1];
  robot_markers_.color.b = marker_rgb[2];
  robot_markers_.scale.x = robot_markers_.scale.y = robot_markers_.scale.z = 0.3;
  robot_markers_.header.frame_id = "map";
  robot_markers_.id = team_id;
  robot_markers_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  robot_markers_.lifetime = rclcpp::Duration(std::chrono::seconds(2));
  
  robot_markers_.points.resize(num_robots_);

  // set up visualization for obstacles
  if( obst_pos_list_.size()%2 != 0 )
    printf( "WARN: Obstacles list must be even sized: [x1 y1 x2 y2 ..]. Skipping!\n" );
  else
  {
    std::cout << std::endl;
    visualization_msgs::msg::Marker ob;
    ob.color.a = 0.42;
    ob.color.r = ob.color.g = ob.color.b = 0.8;
    ob.scale.x = ob.scale.y = ob.scale.z = 0.5;
    ob.header.frame_id = "map";
    ob.header.stamp = now();
    ob.type = visualization_msgs::msg::Marker::CYLINDER;
    ob.id = 100;
    for( unsigned int idx=0; idx<obst_pos_list_.size(); idx+=2 )
    {
      ob.pose.position.x = obst_pos_list_[idx];
      ob.pose.position.y = obst_pos_list_[idx+1];
      ob.pose.position.z = 0.25;
      ob.pose.orientation.w = 1.0;
      ob.id++;
      obst_markers_.markers.push_back( ob );
    }
    rviz_obstmarker_pub_ -> publish( obst_markers_ );
  }
}

bool FreyjaSimulator::collides( const GenericRobot& r1, const GenericRobot& r2 )
{
  static double r1x, r1y, r2x, r2y;
  static double th;
  r1.getWorldPosition( r1x, r1y, th );
  r2.getWorldPosition( r2x, r2y, th );
  return ( (r1x-r2x)*(r1x-r2x) + (r1y-r2y)*(r1y-r2y) < 0.25*0.25 );
}

void FreyjaSimulator::timerTfCallback()
{
  static rclcpp::Time t_topics_updated = now();
  static rclcpp::Time t_onehertz_update = now();
  static TFStamped t;
  static Odom odom;
  static double x, y, th, vx, vy, w;

  // make sure everyone is doing ok
  if( enable_collisions_ )
  {
    for( int i=0; i<num_robots_; i++ )
    {
      for( int j=0; j<num_robots_; j++ )
      {
        if( i!=j && collides(robots_[i], robots_[j]) )
        {
          robots_[i].alert_problem();
          robots_[j].alert_problem();
        }
      }
    }
  }

  // update topics/vis every so often
  rclcpp::Time t_now = now();
  if( (t_now - t_topics_updated).seconds() > topic_step_ )
  {
    // get everyone's poses
    t.header.frame_id = "map";
    for( int idx=0; idx < num_robots_; idx++ )
    {
      robots_[idx].getWorldPosition(x, y, th);
      robots_[idx].getWorldVelocity(vx, vy, w);
      // fill for tf
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, th);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      t.header.stamp = t_now;
      t.child_frame_id = robots_[idx].name_;
      all_tforms_[idx] = t;

      // fill for visualisation
      robot_markers_.points[idx].x = x;
      robot_markers_.points[idx].y = y;
      robot_markers_.points[idx].z = 0.0;

      // fill for odom
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.orientation = tf2::toMsg(q);
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = w;
      odom.child_frame_id = robots_[idx].name_;
      odom_pubs_[idx] -> publish( odom );
    }
    // publish tf
    tf_broadcaster_ -> sendTransform( all_tforms_ );
    // publish markers
    rviz_robotmarker_pub_ -> publish( robot_markers_ );
    t_topics_updated = t_now;
  }

  // slow updates
  if( (t_now - t_onehertz_update).seconds() > 1.0 )
  {
    rviz_obstmarker_pub_ -> publish( obst_markers_ );
    t_onehertz_update = t_now;
  }
}

void FreyjaSimulator::proper_shutdown()
{
  std::cout << "Requesting all robots shutdown .." << std::endl;
  for( auto& r: robots_ )
    r.terminate();
  
  // wait for threads to stop
  for( auto &t : robot_mgrs_ )
    t.join();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr sim = std::make_shared<FreyjaSimulator>();
  rclcpp::spin(sim);
  sim->proper_shutdown();

  rclcpp::shutdown();
  return 0;
}
