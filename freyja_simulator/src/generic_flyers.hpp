#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> PosVelNED;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

enum class FlyingState
{
  DISARMED,
  ARMED_ONGROUND,
  FLYING
};

class GenericFlyer : public GenericRobot
{
public:
  FlyingState air_state_;

  // custom robot properties
  const double MAX_LINEAR_SPD_= 2.0;
  const double MAX_LINEAR_ACC_ = 1.0;

  public:
    using GenericRobot::GenericRobot;
    GenericFlyer( int, std::string, double );
    // forbid copy
    GenericFlyer( const GenericFlyer& ) = delete;
    GenericFlyer& operator=( const GenericFlyer& ) = delete;
    
    // allow move

    GenericFlyer& operator=( GenericFlyer&& ) = default;
    // explicit destructor for handling shutdown
    ~GenericFlyer() { terminate(); };

    // some internal functions this object needs (cannot be called from parent!)
    void initialise_system();
    void arrestMotionVertical() { posvelacc_(2) = posvelacc_(5) = 0.0; }
    bool onGround() { return posvelacc_.coeff(2) > -0.001 && posvelacc_.coeff(8) > 0; }
    
    // some fcns from parent we override on purpose
    void set_arm(bool _req) override;
    bool is_armed() override { return air_state_!=FlyingState::DISARMED; }
    
    // some functions from parent that this needs to implement
    void setCtrlInput(const Eigen::Vector4d& _u) override
      { cur_ctrlinput_ = _u; }
    void setExtForces(const Eigen::Vector4d& _d) override
      { cur_extforces_ = _d; }
    void initialise_stopped(const Eigen::Vector3d& _pos) override;
    void moveRobotStep(const double& dt) override;
};

GenericFlyer::GenericFlyer(int _uid, std::string _n, double dt)
{
  unique_id_  = _uid;
  name_ = std::move(_n);
  step_dt_ = dt;
  posvelacc_.setZero();
  rpy_rpyrate_.setZero();
  cur_extforces_.setZero();
  cur_ctrlinput_.setZero();
  initialise_system();
  is_ok_ = false;
  air_state_ = FlyingState::DISARMED;
  robot_type_ = RobotType::AERIAL;
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
  sys_Bext_ = sys_B_;
}

void GenericFlyer::set_arm( bool _req )
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
  // set this variable anyway
  is_armed_ = (air_state_ == FlyingState::ARMED_ONGROUND)
  || (air_state_ == FlyingState::FLYING);
}

void GenericFlyer::initialise_stopped(const Eigen::Vector3d& _pos)
{
  posvelacc_.head<3>() = _pos;
  posvelacc_.tail<6>().setZero();
  cur_ctrlinput_.setZero();
  cur_extforces_.setZero();
  initialise_system();

  is_ok_ = true;
  keep_alive_.test_and_set();
  printf( "Robot instance: %s(%d), pos: [%0.3f, %0.3f, %0.3f]\n",
                      name_.c_str(), unique_id_, posvelacc_(0), posvelacc_(1), posvelacc_(2) );
}

void GenericFlyer::moveRobotStep( const double& dt )
{
  // called by manager process, with dt as the step time
  // This function should handle ground collisions if needed.
  if(air_state_ == FlyingState::DISARMED)
  {
    printf("%d: Not armed!", unique_id_);
    return;
  }

  static Eigen::Vector3d gvec = {0.0, 0.0, fast_approx::accg};
  
  posvelacc_ = sys_A_ * posvelacc_
              + sys_B_ * cur_ctrlinput_.head<3>()
              + sys_Bext_ * cur_extforces_.head<3>()
              + sys_Bext_ * gvec;
  if(onGround())
    arrestMotionVertical();
  
  // update angles naively using ang-rate, and overwrite yaw-rate from ctrl
  rpy_rpyrate_ = sys_A_.topLeftCorner<6,6>() * rpy_rpyrate_;
  rpy_rpyrate_.tail<1>() = cur_ctrlinput_.tail<1>(); 

  /*  TRY CUSTOM "ERRORS" and state propagation effects here -------
      posvelacc_.tail<3>() = -0.025*posvelacc_.tail<3>()
                              + 0.8*cur_ctrlinput_.head<3>()
                              + gvec + cur_extforces_;
      posvelacc_.segment<3>(3) += step_dt_*posvelacc_.tail<3>();
      posvelacc_.head<3>() += step_dt_*posvelacc_.segment<3>(3);

  /*  if A.bottomRightCorner<3,3> is identity, B matrix can be
   *   removed and use this step separately:
   *     posvelacc_.tail<3>() = cur_ctrlinput_.head<3>() + cur_extforces_ + gvec;
   */
}
