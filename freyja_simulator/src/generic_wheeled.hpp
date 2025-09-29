class GenericWheeled : public GenericRobot
{
  RobotType robot_type_;      // 0: diffdrive, 1: holonimic


  // custom robot properties
  const double MAX_LINEAR_SPD_= 2.0;
  const double MAX_ANGULAR_SPD_= 2.0;
  const double MAX_LINEAR_ACC_ = 1.0;

  public:
    using GenericRobot::GenericRobot;
    GenericWheeled( int, std::string, double, RobotType );
    // forbid copy
    GenericWheeled( const GenericWheeled& ) = delete;
    GenericWheeled& operator=( const GenericWheeled& ) = delete;
    // allow move

    GenericWheeled& operator=( GenericWheeled&& ) = default;
    // explicit destructor for handling shutdown
    ~GenericWheeled();

    

    // some functions from parent that this needs to implement
    void setCtrlInput(const Eigen::Vector4d& _u) override
    { cur_ctrlinput_ = _u; }
    void setExtForces(const Eigen::Vector4d& _d) override
    { cur_extforces_ = _d; }
    void initialise_stopped(const Eigen::Vector3d& _pos) override;
    void moveRobotStep(const double& dt) override;

};

GenericWheeled::~GenericWheeled()
{
  terminate();
}

GenericWheeled::GenericWheeled( int _uid, std::string _n, double dt,  RobotType _type )
{
  unique_id_  = _uid;
  name_ = std::move(_n);
  step_dt_ = dt;
  is_ok_ = false;
  robot_type_ = _type;
}

void GenericWheeled::initialise_stopped(const Eigen::Vector3d& _pos)
{
  // NOTE, we are abusing 3-vec "pos" to mean 2dpos + theta
  posvelacc_.head<2>() = _pos.head<2>();
  rpy_rpyrate_.setZero();
  rpy_rpyrate_(2) = _pos(2);
  posvelacc_.tail<6>().setZero();

  is_ok_ = true;
  keep_alive_.test_and_set();
  printf( "Robot instance: %s(%d), pos: [%0.3f, %0.3f, %0.3f]\n",
          name_.c_str(), unique_id_, posvelacc_(0), posvelacc_(1), posvelacc_(2) );
}


void GenericWheeled::moveRobotStep(const double& dt)
{
  // NOTE, the 4-vec ctrl input represents [vx, vy, v_th, ~]
  if( robot_type_ == RobotType::DIFFDRIVE )
  {
    // diff-drive system using circle eqns.
    // omega_ = omega_body_;
    // new_theta = fast_approx::constrainAngleRad( theta_ + omega_*dt );
    // if( fast_approx::fastabs(omega_) < 0.005 )
    // {
    //   xvel_ = xvel_tgt_*fast_approx::cosine( new_theta );
    //   yvel_ = xvel_tgt_*fast_approx::sine( new_theta );
    // }
    // else
    // {
    //   double r = (xvel_tgt_/omega_);
    //   xvel_ = -r*( fast_approx::sine(theta_) - fast_approx::sine(new_theta) )/dt;
    //   yvel_ = r * ( fast_approx::cosine(theta_) - fast_approx::cosine(new_theta) )/dt;
    // }
  }
  else if( robot_type_ == RobotType::HOLO2D )
  {
    // holonomic robot point
    rpy_rpyrate_(2) = fast_approx::constrainAngleRad(rpy_rpyrate_(2)+cur_ctrlinput_(2) * dt);
    posvelacc_.head<2>() += (cur_ctrlinput_.head<2>()*dt);  // move position
    posvelacc_.segment<2>(3) = cur_ctrlinput_.head<2>();  // set velocity
  }

}


