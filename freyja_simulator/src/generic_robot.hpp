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
  const double MAX_LINEAR_SPD_= 2.0;
  const double MAX_ANGULAR_SPD_= 2.0;
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