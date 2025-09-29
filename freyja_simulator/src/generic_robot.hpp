#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> PosVelNED;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

enum class RobotType {
  DIFFDRIVE,
  HOLO2D,
  AERIAL
};

class GenericRobot {
public:
  Vector9d posvelacc_;
  Vector6d rpy_rpyrate_;

  Vector4d cur_extforces_;
  Vector4d cur_ctrlinput_;

  double total_mass_;
  double step_dt_;

  Eigen::Matrix<double, 9, 9> sys_A_;
  Eigen::Matrix<double, 9, 3> sys_B_, sys_Bext_;


  int unique_id_;
  std::string name_;
  RobotType robot_type_;
  bool is_armed_;    // true/unused for ground systems

  volatile bool is_ok_;
  std::atomic_flag keep_alive_;

  GenericRobot() { is_ok_ = false; };
  GenericRobot(int, std::string, double);
  // forbid copy
  GenericRobot(const GenericRobot&) = delete;
  GenericRobot& operator=(const GenericRobot&) = delete;

  // allow move
  GenericRobot(GenericRobot&& d):  total_mass_(d.total_mass_),
                                    step_dt_(d.step_dt_),
                                    unique_id_(d.unique_id_),
                                    name_(std::move(d.name_))
  {
    posvelacc_ = d.posvelacc_;
    rpy_rpyrate_ = d.rpy_rpyrate_;
    cur_extforces_ = d.cur_extforces_;
    cur_ctrlinput_ = d.cur_ctrlinput_;
    sys_A_ = d.sys_A_;
    sys_B_ = d.sys_B_;
    sys_Bext_ = d.sys_Bext_;
  }
  GenericRobot& operator=( GenericRobot&& ) = default;
  // explicit destructor for handling shutdown
  ~GenericRobot() { terminate(); };

  // some functions are provided for all child classes
  void getWorldPosition(Eigen::Vector3d& _p)  { _p = posvelacc_.segment<3>(0); }
  void getWorldVelocity(Eigen::Vector3d& _v)  { _v = posvelacc_.segment<3>(3); }
  void getWorldPosVelAcc(Vector9d& _pva)      { _pva = posvelacc_; }
  void getBodyAngStates(Vector6d& _a_ar)      { _a_ar = rpy_rpyrate_; }
  void getBodyAngles(Eigen::Vector3d& _a)     { _a = rpy_rpyrate_.head<3>(); }
  void getExtForces(Vector4d& _d)             { _d = cur_extforces_; }
  void setWorldVelocity(const Eigen::Vector3d& _v) { posvelacc_.segment<3>(3) = _v; }
  void arrestMotion()                         { posvelacc_.tail<6>().setZero(); }
  void alert_problem()                        { is_ok_ = false; }
  void terminate()            { keep_alive_.clear(std::memory_order_release); }
  void manager_process();

  // some are allowed to be overridden if needed
  virtual void set_arm(bool _ctg)             { is_armed_ = _ctg; }
  virtual bool is_armed()                    { return is_armed_; }


  // child classes must implement the following
  virtual void initialise_stopped(const Eigen::Vector3d& _pos) = 0;
  virtual void setCtrlInput(const Eigen::Vector4d& _u) = 0;
  virtual void setExtForces(const Eigen::Vector4d& _d) = 0;
  virtual void moveRobotStep(const double& dt) = 0;
};

void GenericRobot::manager_process()
{
  const double dt = step_dt_;
  const int dt_ms = std::round(dt*1000.0);
  keep_alive_.test_and_set();
  printf( "Starting manager process for: %s(%d) with dt: %dms\n",
          name_.c_str(), unique_id_, dt_ms );

  while( keep_alive_.test_and_set() )
  {
    if( is_ok_ )
    {
      moveRobotStep(dt);
      std::this_thread::sleep_for( std::chrono::milliseconds(dt_ms) );
    }
    else
    {
      setWorldVelocity( Eigen::Vector3d::Zero() );
      std::this_thread::sleep_for( std::chrono::milliseconds(2*dt_ms) );
      is_ok_ = true;
    }
  }
  printf( "Terminating manager proces for: %s(%d)\n", name_.c_str(), unique_id_ );
}

#include "generic_flyers.hpp"
#include "generic_wheeled.hpp"
