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

#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/controller_debug.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include "fast_approximate_math.hpp"

#include "generic_robot.hpp"
#include "generic_flyers.hpp"

typedef geometry_msgs::msg::TransformStamped  TFStamped;
typedef geometry_msgs::msg::Vector3           CmdVel3d;
typedef freyja_msgs::msg::ReferenceState      RefState;
typedef freyja_msgs::msg::ControllerDebug     CTRLDebug;


/* Main simulation interface */

class FreyjaSimulator : public rclcpp::Node
{
  int num_robots_;
  std::vector<long int> robot_num_range_;
  std::vector<double> init_positions_;
  
  double sim_step_;
  double topic_step_;


  // robots and managers
  std::vector<GenericFlyer> robots_;
  std::vector<std::thread> robot_mgrs_;
  bool enable_collisions_;
  visualization_msgs::msg::Marker robot_markers_;

  // global visualization obstacles
  std::vector<double> obst_pos_list_;
  visualization_msgs::msg::MarkerArray obst_markers_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<CTRLDebug>::SharedPtr> ctrl_subs_;

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
    bool collides( const GenericFlyer&, const GenericFlyer& );

};

FreyjaSimulator::~FreyjaSimulator()
{
}

FreyjaSimulator::FreyjaSimulator() : Node( "freyja_sim" )
{
  declare_parameter<std::vector<long int>>( "robot_num_range", std::vector<long int>({3, 7}) );
  declare_parameter<std::vector<double>>( "init_positions", std::vector<double>({-2.0, 2.0, -3.0}) );
  declare_parameter<double>("sim_rate", 50.0);
  declare_parameter<double>("topic_rate", 30.0);
  declare_parameter<std::vector<double>>( "team_color", std::vector<double>({1.0, 0.0, 0.0}) );
  declare_parameter<int>( "team_id", 0 );
  declare_parameter<bool>( "enable_collisions", false );
  declare_parameter<std::string>( "robots_type", "diffdrive" );
  declare_parameter<std::vector<double>>( "obst_pos_list", std::vector<double>({-1.0}) );
  
  double refresh_rate, topic_rate;
  std::string robot_type_str;
  get_parameter( "robot_num_range", robot_num_range_ );
  get_parameter( "init_positions", init_positions_ );
  get_parameter( "sim_rate", refresh_rate );
  get_parameter( "topic_rate", topic_rate );
  get_parameter( "enable_collisions", enable_collisions_ );
  get_parameter( "obst_pos_list", obst_pos_list_ );

  // pre-setup
  num_robots_ = int( robot_num_range_[1] - robot_num_range_[0] + 1 );
  printf( "Number of robots: %d\n", num_robots_ );
  assert( num_robots_ > 0 );
  assert( init_positions_.size()%3 == 0 );

  ctrl_subs_.resize(num_robots_);
  all_tforms_.resize(num_robots_);


  // instantiate all robots
  create_robots( 0 );

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
  //std::uniform_real_distribution<> xpos_distrib(spawn_limits_xy_[0], spawn_limits_xy_[1]);
  //std::uniform_real_distribution<> ypos_distrib(spawn_limits_xy_[0], spawn_limits_xy_[1]);
  //std::uniform_real_distribution<> theta_dist(-3.14, 3.14);
  std::uniform_int_distribution<> uid_dist(200,300);

  int idx = 0;
  for( int r=robot_num_range_[0]; r<=robot_num_range_[1]; r++, idx++ )
  {    
    int uid = uid_dist(rand_engine);
    std::string rname = "U" + std::to_string(r);
    
    robots_.emplace_back( uid, rname, 0.01 );
          std::cout << std::endl;
  std::this_thread::sleep_for( std::chrono::seconds(1) );
    Eigen::Vector3d p = { init_positions_[3*idx], init_positions_[3*idx+1], init_positions_[3*idx+2] };

    robots_[idx].initialise_stopped( p );
    // create subscriber
    ctrl_subs_[idx] = create_subscription<CTRLDebug> ( rname + "/controller_debug", 1,
                            [this,idx](const CTRLDebug::ConstSharedPtr msg)
                            {
                              Eigen::Map<const Eigen::Vector4f> u( msg->lqr_u.data() );
                              robots_[idx].setCtrlInput( u.cast<double>() );
                            } );
  }
  printf( "All robots created. Starting managers..\n" );

  for(int idx=0; idx<num_robots_; idx++ )
    robot_mgrs_.push_back( std::move(std::thread(&GenericFlyer::manager_process, &robots_[idx])) );
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

bool FreyjaSimulator::collides( const GenericFlyer& r1, const GenericFlyer& r2 )
{
  static Eigen::Vector3d r1pos, r2pos;
  r1.getWorldPosition( r1pos );
  r2.getWorldPosition( r2pos );
  return ( (r1pos-r2pos).squaredNorm() < 0.25*0.25 );
}

void FreyjaSimulator::timerTfCallback()
{
  static rclcpp::Time t_topics_updated = now();
  static rclcpp::Time t_onehertz_update = now();
  static TFStamped t;
  static Eigen::Vector3d robot_pos, robot_rpy;
  //static Odom odom;

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
    t.header.frame_id = "map_ned";
    for( int idx=0; idx < num_robots_; idx++ )
    {
      robots_[idx].getWorldPosition(robot_pos);
      robots_[idx].getAnglesRPY(robot_rpy);
      // fill for tf
      t.transform.translation.x = robot_pos.coeff(0);
      t.transform.translation.y = robot_pos.coeff(1);
      t.transform.translation.z = robot_pos.coeff(2);
      tf2::Quaternion q;
      q.setRPY(robot_rpy.coeff(0), robot_rpy.coeff(1), robot_rpy.coeff(2));
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      t.header.stamp = t_now;
      t.child_frame_id = robots_[idx].name_;
      all_tforms_[idx] = t;

      // fill for visualisation
      //robot_markers_.points[idx].x = x;
      //robot_markers_.points[idx].y = y;
      //robot_markers_.points[idx].z = 0.0;

    }
    // publish tf
    tf_broadcaster_ -> sendTransform( all_tforms_ );
    // publish markers
    //rviz_robotmarker_pub_ -> publish( robot_markers_ );
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
