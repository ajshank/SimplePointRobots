#include <eigen3/Eigen/Dense>
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/common/quad_state.hpp"

typedef Eigen::Matrix<double, 6, 1> PosVelNED;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

enum class FlyingState
{
  DISARMED,
  ARMED_ONGROUND,
  FLYING
};

class GenericFlyer
{
  flightlib::Quadrotor robot_;
  flightlib::Command robot_cmd_;
  flightlib::QuadState quad_state_;

  public:
  Vector9d posvelacc_;
  Vector6d rpy_rpyrate_;
  
  Eigen::Vector3d cur_extforces_;
  Eigen::Vector4d cur_ctrlinput_;

  double total_mass_;

  Eigen::Matrix<double, 9, 9> sys_A_;
  Eigen::Matrix<double, 9, 3> sys_B_, sys_Bext_;

  double step_dt_;

  int unique_id_;
  std::string name_;
  
  volatile bool is_ok_;
  std::atomic_flag keep_alive_;
  FlyingState air_state_;

  // custom robot properties
  const double MAX_LINEAR_SPD_= 2.0;
  const double MAX_LINEAR_ACC_ = 1.0;

  public:
    GenericFlyer( int, std::string, double );
    // forbid copy
    GenericFlyer( const GenericFlyer& ) = delete;
    GenericFlyer& operator=( const GenericFlyer& ) = delete;
    
    // allow move
    GenericFlyer( GenericFlyer&& d):  total_mass_(d.total_mass_),
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
      air_state_ = d.air_state_;
    }
    GenericFlyer& operator=( GenericFlyer&& ) = default;
    // explicit destructor for handling shutdown
    ~GenericFlyer();

    void initialise_system();

    void initialise_stopped( Eigen::VectorXd _pos );
    void setVelocity( Eigen::Vector3d _tgt_vel ) { posvelacc_.segment<3>(3) = _tgt_vel; }
    void setCtrlInput( const Eigen::Vector4d _ctrl )
    {
      robot_cmd_.omega << _ctrl.head<2>().cast<float>(), float(_ctrl.coeff(3));
      robot_cmd_.collective_thrust = -static_cast<float>(_ctrl.coeff(2)) + fast_approx::accg;
    }
    void setExtForces( Eigen::Vector3d _extf ) { cur_extforces_ = _extf; }
    void arrestMotion() { posvelacc_.tail<6>().setZero(); }
    void arrestMotionGround() { posvelacc_(2) = posvelacc_(5) = 0.0; }
    bool onGround() { return posvelacc_.coeff(2) > -0.001 && posvelacc_.coeff(8) > 0; }
    
    void getPosVelAcc( Vector9d &_pva ) const
      { _pva = posvelacc_; }
    void getWorldVelocity( Eigen::Vector3d &_v ) const
      { _v = posvelacc_.segment<3>(3); }
    void getWorldPosition( Eigen::Vector3d &_p ) const
      { _p = posvelacc_.segment<3>(0); }
    void getAnglesRPY( Eigen::Vector3d &_a ) const
      { _a = rpy_rpyrate_.segment<3>(0); }
    void getExtForces( Eigen::Vector3d &_f ) const
      { _f = cur_extforces_; }
    
    void alert_problem() { is_ok_ = false; }
    void terminate() { keep_alive_.clear(std::memory_order_release); }
    void arm( bool req );
    
    void moveRobot( const double& dt );
    void manager_process();

    bool is_armed() { return air_state_!=FlyingState::DISARMED; }

};

GenericFlyer::~GenericFlyer()
{
  terminate();
}

GenericFlyer::GenericFlyer( int _uid, std::string _n, double dt ) : 
                                    unique_id_(_uid),
                                    name_(std::move(_n)),
                                    step_dt_(dt)
{
  posvelacc_.setZero();
  rpy_rpyrate_.setZero();
  cur_extforces_.setZero();
  cur_ctrlinput_.setZero();
  initialise_system();
  is_ok_ = false;
  air_state_ = FlyingState::DISARMED;
}
void GenericFlyer::initialise_stopped( Eigen::VectorXd _pos )
{
  posvelacc_.head<3>() = _pos;
  posvelacc_.tail<6>().setZero();
  cur_ctrlinput_.setZero();
  cur_extforces_.setZero();
  initialise_system();

  is_ok_ = true;
  keep_alive_.test_and_set();




  // hovering test
  quad_state_.setZero();
  quad_state_.p = _pos.cast<float>();
  robot_.reset(quad_state_);
  robot_cmd_.omega.setZero();
  robot_cmd_.collective_thrust = 1.05*fast_approx::accg;
  robot_cmd_.t = 0.0;

  printf( "Robot instance: %s(%d), pos: [%0.3f, %0.3f, %0.3f], cmd_valid: %d\n",
          name_.c_str(), unique_id_, robot_.getPosition().coeff(0), robot_.getPosition().coeff(1), robot_.getPosition().coeff(2),
         robot_cmd_.valid());
  std::cout << robot_cmd_.t << "[s], w=" << robot_cmd_.omega.transpose() << " thr="
            << robot_cmd_.collective_thrust << " rotors=" << robot_cmd_.thrusts.transpose()
            << std::endl;
}

void GenericFlyer::initialise_system()
{
  double dt = step_dt_;
  double dt2 = 0.5*dt*dt;
  sys_A_ << 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, dt2,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
            Eigen::MatrixXd::Zero(3,6), 0.001*Eigen::MatrixXd::Identity(3,3);
  sys_B_ << Eigen::MatrixXd::Zero(6,3),
            Eigen::MatrixXd::Identity(3,3);
  sys_Bext_ << sys_B_;
}

void GenericFlyer::arm( bool _req )
{
  switch( air_state_ )
  {
    case FlyingState::DISARMED:
      {
        air_state_ = (_req==true)? FlyingState::ARMED_ONGROUND : air_state_;
        break;
      }
    case FlyingState::ARMED_ONGROUND:
      {
        air_state_ = (_req==false)? FlyingState::DISARMED : air_state_;
        break;
      }
    case FlyingState::FLYING:
      {
        air_state_ = (_req==false)? FlyingState::DISARMED : air_state_;
        std::cout << "Allowing arm/disarm calls in the air!" << std::endl;
        break;
      }
  }
}

void GenericFlyer::moveRobot( const double& dt )
{
  // called by manager process, with dt as the step time
  static Eigen::Vector3d gvec = {0.0, 0.0, fast_approx::accg};
  
 posvelacc_ = sys_A_ * posvelacc_ 
            + sys_B_ * cur_ctrlinput_.head<3>()
            + sys_Bext_ * cur_extforces_
            + sys_Bext_ * gvec;
              //- 0.25 * sys_Bext_ * posvelacc_.segment<3>(3).cwiseAbs2().cwiseProduct(posvelacc_.segment<3>(3).cwiseSign());  
  
  /*
  posvelacc_.tail<3>() = -0.025*posvelacc_.tail<3>() + 0.8*cur_ctrlinput_.head<3>() + gvec + cur_extforces_;
  posvelacc_.segment<3>(3) += step_dt_*posvelacc_.tail<3>();
  //   posvelacc_.segment<3>(3) *= 1.0;
  posvelacc_.head<3>() += step_dt_*posvelacc_.segment<3>(3);
  */
  /*  if A.bottomRightCorner<3,3> is identity, B matrix can be
  *   removed and use this step separately:
  *     posvelacc_.tail<3>() = cur_ctrlinput_.head<3>() + cur_extforces_ + gvec;
  */
  
  // update angles naively using ang-rate, and overwrite yaw-rate from ctrl
  rpy_rpyrate_ = sys_A_.topLeftCorner<6,6>() * rpy_rpyrate_;
  rpy_rpyrate_.tail<1>() = cur_ctrlinput_.tail<1>(); 
}

void GenericFlyer::manager_process()
{
  const float dt = step_dt_;
  const int dt_ms = std::round(dt*1000.0);
  keep_alive_.test_and_set();
  printf( "Starting manager process for: %s(%d) with dt: %dms, pos: [%0.3f, %0.3f, %0.3f], valid: %d\n",
          name_.c_str(), unique_id_, dt_ms, robot_.getPosition().coeff(0), robot_.getPosition().coeff(1), robot_.getPosition().coeff(2),
         robot_.getDynamics().valid());
  //std::cout << robot_.getPosition().transpose() << std::endl;

  while( keep_alive_.test_and_set() )
  {
    if( is_ok_ )
    {
      if( !robot_.setCommand(robot_cmd_) ) std::cout << "Command invalid!" << std::endl;
      if( !robot_.run(dt) ) std::cout << "Could not step flyer!" << std::endl;
      if( onGround() || false )
        arrestMotion();
      // update state
      robot_.getState(&quad_state_);
      posvelacc_.segment<3>(0) = quad_state_.p.cast<double>();
      std::cout << robot_cmd_.collective_thrust << " Newtons, "
                << robot_.getPosition().transpose() << std::endl;
      std::this_thread::sleep_for( std::chrono::milliseconds(dt_ms) );
    }
    else
    {
      setVelocity( Eigen::Vector3d::Zero() );
      std::this_thread::sleep_for( std::chrono::milliseconds(2*dt_ms) );
      is_ok_ = true;
    }
  }
  printf( "Terminating manager proces for: %s(%d)\n", name_.c_str(), unique_id_ );
}
