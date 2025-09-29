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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
//#include "mavros_msgs/srv/command_bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include "tf2_ros/transform_broadcaster.h"

#include "freyja_msgs/msg/current_state.hpp"
#include "freyja_msgs/msg/reference_state.hpp"
#include "freyja_msgs/msg/controller_debug.hpp"
#include "freyja_msgs/msg/freyja_interface_status.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include "fast_approximate_math.hpp"

#include "generic_robot.hpp"

typedef Eigen::Matrix<double, 9, 1>           Vector9d;
typedef geometry_msgs::msg::TransformStamped  TFStamped;
typedef geometry_msgs::msg::Vector3           GeomVec3d;
typedef freyja_msgs::msg::ReferenceState      RefState;
typedef freyja_msgs::msg::ControllerDebug     CTRLDebug;
typedef freyja_msgs::msg::CurrentState        CurrentState;
typedef freyja_msgs::msg::FreyjaInterfaceStatus FreyjaIfaceStatus;
typedef visualization_msgs::msg::MarkerArray  RvizMarkerArray;
typedef geometry_msgs::msg::Vector3Stamped    GeomVec3Stamped;
typedef std_srvs::srv::SetBool                BoolServ;
//typedef mavros_msgs::srv::CommandBool         MavrosArming;


/* Main simulation interface */

class FreyjaSimulator : public rclcpp::Node
{
  int num_robots_;
  std::string platf_basename_;
  std::vector<long int> robot_num_range_;
  std::vector<double> init_positions_;
  
  double sim_step_;
  double tf_upd_interv_;


  // robots and managers
  std::vector<std::unique_ptr<GenericRobot>> robots_;
  std::vector<std::thread> robot_mgrs_;
  bool enable_collisions_;
  bool enable_downwash_;
  bool publish_gt_curstate_;
  bool publish_gt_extf_;
  bool publish_gt_iface_;

  // some robot params
  double dw_ellipse_a_;     // ellipse-a [m] for downwash 
  double dw_expo_ampl_;     // downwash amplitude scaling: A*exp(..)

  visualization_msgs::msg::MarkerArray robot_markers_;

  // global visualization obstacles
  std::vector<double> obst_pos_list_;
  visualization_msgs::msg::MarkerArray obst_markers_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<rclcpp::Subscription<CTRLDebug>::SharedPtr> ctrl_subs_;
  std::vector<rclcpp::Publisher<GeomVec3Stamped>::SharedPtr> extf_pubs_;
  std::vector<rclcpp::Publisher<CurrentState>::SharedPtr> simstate_pubs_;
  std::vector<rclcpp::Publisher<FreyjaIfaceStatus>::SharedPtr> iface_pubs_;
  std::vector<rclcpp::Service<BoolServ>::SharedPtr> idle_servers_;
  //std::vector<rclcpp::Service<MavrosArming>::SharedPtr> arm_servers_;

  rclcpp::CallbackGroup::SharedPtr reentr_subs_grp_; 

  std::vector<TFStamped> all_tforms_;
  
  public:
    FreyjaSimulator();
    ~FreyjaSimulator();
    rclcpp::TimerBase::SharedPtr tf_timer_;
    void simMainLoopTimer() __attribute__((hot));

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_robotmarker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_obstmarker_pub_;
    void rendering_setup();
    
    void create_robots(const std::string&);
    void create_diffdrives(int) {};
    void create_holo2d(int);
    void create_flyers(int);
    void start_managers();
    void proper_shutdown();
    bool collides( std::unique_ptr<GenericRobot> &r1,
                   std::unique_ptr<GenericRobot> &r2 );
    Eigen::Vector3d computeDownwash( std::unique_ptr<GenericRobot> &r1,
                                     std::unique_ptr<GenericRobot> &r2 );

};

FreyjaSimulator::~FreyjaSimulator()
{
}

FreyjaSimulator::FreyjaSimulator() : Node( "freyja_sim" )
{
  declare_parameter<std::vector<long int>>( "robot_num_range", std::vector<long int>({3, 7}) );
  declare_parameter<std::string>("platf_basename", "R");
  declare_parameter<std::vector<double>>( "init_positions", std::vector<double>({-2.0, 2.0, -3.0}) );
  declare_parameter<double>("sim_rate", 50.0);
  declare_parameter<double>("topic_rate", 30.0);
  declare_parameter<std::vector<double>>( "team_color", std::vector<double>({1.0, 0.0, 0.0}) );
  declare_parameter<int>( "team_id", 0 );
  declare_parameter<std::string>( "robot_type", "diffdrive" );
  declare_parameter<std::vector<double>>( "obst_pos_list", std::vector<double>({-1.0}) );
  declare_parameter<bool>( "enable_collisions", false );
  declare_parameter<bool>( "enable_downwash", false );
  declare_parameter<bool>( "publish_gt_curstate", false );
  declare_parameter<bool>( "publish_gt_extf", false );

  declare_parameter<double>( "dwash_ellipse_a", 0.3 );
  declare_parameter<double>( "dwash_expo_ampl", 5.0 );
  
  double refresh_rate, topic_rate;
  std::string robot_type_str;
  get_parameter( "robot_type", robot_type_str );
  get_parameter( "robot_num_range", robot_num_range_ );
  get_parameter( "platf_basename", platf_basename_ );
  get_parameter( "init_positions", init_positions_ );
  get_parameter( "sim_rate", refresh_rate );
  get_parameter( "topic_rate", topic_rate );
  get_parameter( "obst_pos_list", obst_pos_list_ );
  get_parameter( "enable_collisions", enable_collisions_ );
  get_parameter( "enable_downwash", enable_downwash_ );
  get_parameter( "publish_gt_curstate", publish_gt_curstate_ );
  get_parameter( "publish_gt_extf", publish_gt_extf_);

  get_parameter( "dwash_ellipse_a", dw_ellipse_a_ );
  get_parameter( "dwash_expo_ampl", dw_expo_ampl_ );


  // pre-setup
  num_robots_ = int( robot_num_range_[1] - robot_num_range_[0] + 1 );
  printf( "Number of robots: %d\n", num_robots_ );
  assert( num_robots_ > 0 );
  assert( init_positions_.size()%3 == 0 );

  ctrl_subs_.resize(num_robots_);
  all_tforms_.resize(num_robots_);
  extf_pubs_.resize(num_robots_);
  simstate_pubs_.resize(num_robots_);
  iface_pubs_.resize(num_robots_);
  idle_servers_.resize(num_robots_);
  //arm_servers_.resize(num_robots_);


  // instantiate all robots
  create_robots(robot_type_str);

  // set up broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // set up visualization markers
  rviz_robotmarker_pub_ = create_publisher<RvizMarkerArray>("/global_robot_markers", rclcpp::QoS(1));
  rviz_obstmarker_pub_ = create_publisher<RvizMarkerArray>("/global_obst_markers", rclcpp::QoS(1).transient_local());

  // set up extra pieces of simulation
  rendering_setup();

  // set up a timer process
  tf_upd_interv_ = 1.0/topic_rate;
  sim_step_ = 1.0/refresh_rate;
  tf_timer_ = rclcpp::create_timer( this, get_clock(), std::chrono::duration<double>(sim_step_),
                                    std::bind(&FreyjaSimulator::simMainLoopTimer, this) );
  RCLCPP_INFO( get_logger(), "Simulator Ready!" );
}

void FreyjaSimulator::create_robots(const std::string& _type)
{
  // Robots get a unique internal id. This is unlikely to be useful anymore, but is
  // kept for compatibility reasons (may come in handy though).
  std::random_device rd;
  std::default_random_engine rand_engine(rd());
  std::uniform_int_distribution<> uid_dist(100,1000);
  std::uniform_real_distribution<> position_dist(-1.0, 1.0);
  int uid_base = uid_dist(rand_engine);

  if(_type == "diffdrive")
    create_diffdrives(uid_base);
  else if(_type == "holo2d")
    create_holo2d(uid_base);
  else if(_type == "aerial")
    create_flyers(uid_base);
  else
    RCLCPP_ERROR(get_logger(), "Unknown robot type: %s ~ {diffdrive, holo2d, aerial}",
                 _type.c_str());
}

void FreyjaSimulator::create_holo2d(int uid_base)
{
  reentr_subs_grp_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = reentr_subs_grp_;

  int idx = 0;
  double dynstep_dt = 1.0/200.0;    // dynamics updated at this rate (must be >=1ms)
  for( int r=robot_num_range_[0]; r<=robot_num_range_[1]; r++, idx++ )
  {
    int uid = uid_base + r;
    std::string rname = platf_basename_ + std::to_string(r);

    robots_.emplace_back( std::make_unique<GenericWheeled>(uid, rname, dynstep_dt, RobotType::HOLO2D) );
    Eigen::Vector3d pos;
    if( (3*idx+2) < init_positions_.size() )
      pos << init_positions_[3*idx], init_positions_[3*idx+1], init_positions_[3*idx+2];
    else
      pos = Eigen::Vector3d::Random() + Eigen::Vector3d(0, 0, -10.0);

    robots_[idx]->initialise_stopped( pos );
    // create subscriber
    ctrl_subs_[idx] = create_subscription<CTRLDebug> ( rname + "/controller_debug", 1,
                              [this,idx](const CTRLDebug::ConstSharedPtr msg)
                              {//. reentrant cb-group; make sure this is thread-safe.
                                Eigen::Map<const Eigen::Vector4f> u( msg->lqr_u.data() );
                                robots_[idx]->setCtrlInput( u.cast<double>() );
                              },
                              options );
    publish_gt_iface_ = false;
    if(publish_gt_curstate_)
      simstate_pubs_[idx] = create_publisher<CurrentState>( rname + "/current_state_gt", 1 );
    if(publish_gt_extf_)
      extf_pubs_[idx] = create_publisher<GeomVec3Stamped>( rname + "/ext_forces_gt", 1 );
  }

  start_managers();
}

void FreyjaSimulator::create_flyers(int uid_base)
{
  reentr_subs_grp_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = reentr_subs_grp_;

  int idx = 0;
  double dynstep_dt = 1.0/200.0;    // dynamics updated at this rate (must be >=1ms)
  for( int r=robot_num_range_[0]; r<=robot_num_range_[1]; r++, idx++ )
  {    
    int uid = uid_base + r;
    std::string rname = platf_basename_ + std::to_string(r);
    
    robots_.emplace_back( std::make_unique<GenericFlyer>(uid, rname, dynstep_dt) );
    Eigen::Vector3d pos;
    if( (3*idx+2) < init_positions_.size() )
      pos << init_positions_[3*idx], init_positions_[3*idx+1], init_positions_[3*idx+2];
    else
      pos = Eigen::Vector3d::Random() + Eigen::Vector3d(0, 0, -10.0);

    robots_[idx]->initialise_stopped( pos );
    // create subscriber
    ctrl_subs_[idx] = create_subscription<CTRLDebug> ( rname + "/controller_debug", 1,
                            [this,idx](const CTRLDebug::ConstSharedPtr msg)
                            {//. reentrant cb-group; make sure this is thread-safe.
                              Eigen::Map<const Eigen::Vector4f> u( msg->lqr_u.data() );
                              robots_[idx]->setCtrlInput( u.cast<double>() );
                            },
                            options );
    // create clients
    idle_servers_[idx] = create_service<BoolServ> ( rname+"/set_onground_idle",
                            [this,idx](const BoolServ::Request::SharedPtr rq, const BoolServ::Response::SharedPtr rp )
                            {  rp->success = true; } );
    /*arm_servers_[idx] = create_service<MavrosArming> ( rname+"/mavros/cmd/arming",
                            [this,idx,rname](const MavrosArming::Request::SharedPtr rq, const MavrosArming::Response::SharedPtr rp )
                            {
                              robots_[idx]->set_arm(rq->value);
                              rp->success = true;
                              RCLCPP_WARN(get_logger(), "Arming: %s", rname.c_str());
                            } );         */
    publish_gt_iface_ = true;
    iface_pubs_[idx] = create_publisher<FreyjaIfaceStatus>
                              (rname + "/freyja_interface_status", 1);

    if(publish_gt_extf_)
      extf_pubs_[idx] = create_publisher<GeomVec3Stamped>
                              (rname + "/ext_forces_gt", 1);

    if( publish_gt_curstate_ )
      simstate_pubs_[idx] = create_publisher<CurrentState>
                              (rname + "/current_state_gt", 1);
  }
  printf( "All robots created. Starting managers..\n" );

  start_managers();
}

void FreyjaSimulator::start_managers()
{
  for(int idx=0; idx<num_robots_; idx++ )
    robot_mgrs_.push_back( std::move(
                              std::thread(&GenericRobot::manager_process,
                                          robots_[idx].get()
                                          )
                                     )
                          );
  // fake sleep to mimic simulator slow-startup
  //std::this_thread::sleep_for( std::chrono::seconds(1) );
}

void FreyjaSimulator::rendering_setup()
{
  int team_id;
  std::vector<double> marker_rgb;
  get_parameter( "team_color", marker_rgb );
  get_parameter( "team_id", team_id );

  if( enable_downwash_ )
  {
    float dw_length = 2.5;   // metres
    visualization_msgs::msg::Marker m;
    m.color.a = 0.31;
    m.color.g = 0.71;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.lifetime = rclcpp::Duration(std::chrono::seconds(2));
    m.pose.position.x = m.pose.position.y = 0.0;
    m.pose.position.z = dw_length/2.0;
    m.scale.x = m.scale.y = dw_ellipse_a_*2;
    m.scale.z = dw_length;
    for( const auto& r : robots_ )
    {
      m.header.frame_id = r->name_;
      m.id = r->unique_id_;
      robot_markers_.markers.push_back( m );
    }
  }
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
    ob.header.frame_id = "map_ned";
    ob.header.stamp = now();
    ob.type = visualization_msgs::msg::Marker::CYLINDER;
    ob.id = 100;
    for( unsigned int idx=0; idx<obst_pos_list_.size(); idx+=2 )
    {
      ob.pose.position.x = obst_pos_list_[idx];
      ob.pose.position.y = obst_pos_list_[idx+1];
      ob.pose.position.z = -0.25;
      ob.pose.orientation.w = 1.0;
      ob.id++;
      obst_markers_.markers.push_back( ob );
    }
    rviz_obstmarker_pub_ -> publish( obst_markers_ );
  }
}

bool FreyjaSimulator::collides( std::unique_ptr<GenericRobot> &r1,
                                std::unique_ptr<GenericRobot> &r2 )
{
  static Eigen::Vector3d r1pos, r2pos;
  r1->getWorldPosition( r1pos );
  r2->getWorldPosition( r2pos );
  return ( (r1pos-r2pos).squaredNorm() < 0.25*0.25 );
}

Eigen::Vector3d FreyjaSimulator::computeDownwash( std::unique_ptr<GenericRobot> &r1,
                                                  std::unique_ptr<GenericRobot> &r2 )
{
  // returns force applied on r1 due to r2's downwash
  static Eigen::Vector3d ext_f;
  static Eigen::Vector3d r1pos, r2pos, r1vel, r2vel;
  static std::function<double(Eigen::Vector3d&)> r2ellipse;
  static double dw_halflen = 2.5/2;
  
  ext_f.setZero();
  // only do anything if robots are different
  if( r1->unique_id_ != r2->unique_id_ )
  {  
    // calc distance vector to r1 from r2
    r1->getWorldPosition( r1pos );
    r1->getWorldVelocity( r1vel );
    r2->getWorldPosition( r2pos );
    r2->getWorldVelocity( r2vel );
    // find r2's ellipsoid
    double a = dw_ellipse_a_;
    double c = dw_halflen;
    r2ellipse = [&r2pos, &a, &c, &dw_halflen](Eigen::Vector3d& _pos)
      {
        // with _pos as some arbitrary point of interest, the equation is:
        //    sqnorm( (_pos - (r2 + [0,0,dw])) ./ [a,a,c] )
        // which is: {(x-r2x)/a}^2 + {(y-r2y)/a}^2 + {(z-(r2z+dw))/c}^2
        // which is the eqn of an ellipsoid centered at (r2x, r2y, r2z+dw).

        //return  (_pos.head<2>() - r2pos.head<2>()).squaredNorm()/(a*a)
        //        + fast_approx::square( (_pos.coeff(2) - (r2pos.coeff(2)+dw_halflen))/c );
        return ( _pos - (r2pos+Eigen::Vector3d(0,0,dw_halflen)) ).cwiseQuotient(Eigen::Vector3d(a,a,c)).squaredNorm();
      };
    bool insideEllipse = r2ellipse(r1pos) < 1.0;
    // where in the ellipse-peel is r1? 0=skin surface, 1=inner core
    float ellipsePeelFactor = r2ellipse(r1pos) - 1.50;
    if( insideEllipse )
    {
      Eigen::Vector3d r21 = r2pos-r1pos;
      Eigen::Vector3d v21 = r2vel-r1vel;
      double mag_r21 = r21.norm();
      double mag_v21 = std::min( 4.0, std::max( 0.05, v21.norm() ) );
      // apply negated exponential curve (pos) and inv-linear curve (vel)
      //ext_f = -8.0*std::exp(-mag_r21)*(r21/mag_r21).array() - 0.08/mag_v21;

      // the following is equivariant (contributed by H.Smith)
      double fd = dw_expo_ampl_*std::exp(-mag_r21);
      double R = 1.0/v21.head<2>().norm() 
              * ( v21.head<2>().dot(r21.head<2>())
                 / (v21.head<2>().norm()*r21.head<2>().norm())  )
              * std::exp(-r21.head<2>().norm())
              * std::exp(-r21.tail<1>().norm());
      double th = std::acos( r21.coeff(0)/(r21.head<2>().norm()) );
      th = r21.coeff(1) < 0 ? -th : th;
      ext_f <<  R*fast_approx::cosine(th),
                R*fast_approx::sine(th),
                fd;
      
    }
  }

  return ext_f;
}

void FreyjaSimulator::simMainLoopTimer()
{
  static rclcpp::Time t_state_topic_upd = now();
  static rclcpp::Time t_onehertz_upd = now();
  static rclcpp::Time t_iface_upd = now();
  static Eigen::Vector3d robot_pos, robot_rpy;
  static Eigen::Vector4d robot_extf;
  static Vector9d robot_pva;
  static TFStamped t;
  static GeomVec3Stamped ext_f_msg;
  static CurrentState cs_msg;
  static FreyjaIfaceStatus iface_msg;


  // make sure everyone is doing ok
  if( enable_collisions_ )
  {
    for( int i=0; i<num_robots_; i++ )
    {
      for( int j=0; j<num_robots_; j++ )
      {
        if( i!=j && collides(robots_[i], robots_[j]) )
        {
          robots_[i]->alert_problem();
          robots_[j]->alert_problem();
        }
      }
    }
  }

  // apply downwash forces
  if( enable_downwash_ )
  {
    Eigen::Vector4d ext_f;
    for( int i=0; i<num_robots_; i++ )
    {
      ext_f.setZero();
      for( int j=0; j<num_robots_; j++ )
        if(i != j)
          ext_f.head<3>() += computeDownwash( robots_[i], robots_[j] );
      robots_[i]->setExtForces( ext_f );
    }
  }

  // update topics/vis every so often
  rclcpp::Time t_now = now();
  if( (t_now - t_state_topic_upd).seconds() > tf_upd_interv_ )
  {
    // get everyone's poses
    t.header.frame_id = "map_ned";
    for( int idx=0; idx < num_robots_; idx++ )
    {
      // fetch data for this robot
      robots_[idx]->getBodyAngles(robot_rpy);
      robots_[idx]->getExtForces(robot_extf);
      robots_[idx]->getWorldPosVelAcc(robot_pva);
      robot_pos = robot_pva.head<3>();
      
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
      t.child_frame_id = robots_[idx]->name_;
      all_tforms_[idx] = t;

      if(publish_gt_extf_)
      {// publish known forces acting on the system
        ext_f_msg.header.stamp = t_now;
        ext_f_msg.vector.x = robot_extf.coeff(0);
        ext_f_msg.vector.y = robot_extf.coeff(1);
        ext_f_msg.vector.z = robot_extf.coeff(2);
        extf_pubs_[idx] -> publish(ext_f_msg);
      }

      // publish ground truth state-vector
      if( publish_gt_curstate_ )
      {
        for( int sv=0; sv<6; sv++ )
          cs_msg.state_vector[sv] = robot_pva.coeff(sv);
        cs_msg.state_vector[9] = robot_pva.coeff(6);
        cs_msg.state_vector[10] = robot_pva.coeff(7);
        cs_msg.state_vector[11] = robot_pva.coeff(8);
        cs_msg.header.stamp = t_now;
        simstate_pubs_[idx] -> publish( cs_msg );
      }
    }
    // publish tf
    tf_broadcaster_ -> sendTransform( all_tforms_ );
    // publish markers
    rviz_robotmarker_pub_ -> publish( robot_markers_ );
    t_state_topic_upd = t_now;
  }

  // slow updates
  if( (t_now - t_onehertz_upd).seconds() > 1.0 )
  {
    rviz_obstmarker_pub_ -> publish( obst_markers_ );
    t_onehertz_upd = t_now;
  }

  if( publish_gt_iface_ && (t_now - t_iface_upd).seconds() > 0.1 )
  {
    for( int idx=0; idx < num_robots_; idx++ )
    {
      iface_msg.armed = robots_[idx]->is_armed();
      iface_msg.connected = true;
      iface_msg.computer_ctrl = true;
      iface_msg.rtk_fix_ok = true;
      iface_msg.rtk_carrsol = 2;

      iface_pubs_[idx] -> publish(iface_msg);
    }
  }
}

void FreyjaSimulator::proper_shutdown()
{
  std::cout << "Requesting all robots shutdown .." << std::endl;
  for( auto& r: robots_ )
    r->terminate();
  
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
