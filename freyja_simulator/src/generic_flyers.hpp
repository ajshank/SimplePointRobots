#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> PosVelNED;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;

struct GenericFlyer
{
  Vector9d posvelacc_;
  Vector6d ang_angrate_;
  
  Eigen::Vector3d cur_extforces_;
  Eigen::Vector3d cur_ctrlinput_;

  double total_mass_;

  Eigen::Matrix<double, 9, 9> sys_A_;
  Eigen::Matrix<double, 9, 3> sys_B_, sys_Bext_;

  double step_dt_;

  int unique_id_;
  std::string name_;
  
  volatile bool is_ok_;
  std::atomic_flag keep_alive_;

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
      ang_angrate_ = d.ang_angrate_;
      cur_extforces_ = d.cur_extforces_;
      cur_ctrlinput_ = d.cur_ctrlinput_;
      sys_A_ = d.sys_A_;
      sys_B_ = d.sys_B_;
      sys_Bext_ = d.sys_Bext_;
    }
    GenericFlyer& operator=( GenericFlyer&& ) = default;
    // explicit destructor for handling shutdown
    ~GenericFlyer();

    void initialise_system();

    void initialise_stopped( Eigen::VectorXd _pos );
    void setVelocity( Eigen::Vector3d _tgt_vel ) { posvelacc_.segment<3>(3).setZero(); }
    void setCtrlInput( const Eigen::Vector4d _ctrl ) { cur_ctrlinput_ = _ctrl.head<3>(); }
    void setExtForces( Eigen::Vector3d _extf ) { cur_extforces_ = _extf; }
    void arrestMotion() { posvelacc_.tail<6>().setZero(); }
    void arrestMotionGround() { posvelacc_(2) = posvelacc_(5) = 0.0; }
    bool onGround() { return posvelacc_.coeff(2) > -0.01; }
    
    void getWorldVelocity( Eigen::Vector3d &_v ) const
      { _v = posvelacc_.segment<3>(3); }
    void getWorldPosition( Eigen::Vector3d &_p ) const
      { _p = posvelacc_.segment<3>(0); }
    
    void alert_problem() { is_ok_ = false; }
    void terminate() { keep_alive_.clear(std::memory_order_release); }
    
    void moveRobot( const double& dt );
    void manager_process();

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
  ang_angrate_.setZero();
  cur_extforces_.setZero();
  cur_ctrlinput_.setZero();
  initialise_system();
  is_ok_ = false;
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
  printf( "Robot instance: %s(%d), pos: [%0.3f, %0.3f, %0.3f]\n",
                      name_.c_str(), unique_id_, posvelacc_(0), posvelacc_(1), posvelacc_(2) );
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
            Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
  sys_B_ << Eigen::MatrixXd::Zero(6,3),
            Eigen::MatrixXd::Identity(3,3);
  sys_Bext_ << sys_B_;
}

void GenericFlyer::moveRobot( const double& dt )
{
  // called by manager process, with dt as the step time
  static Eigen::Vector3d gvec = {0.0, 0.0, fast_approx::accg};
  posvelacc_ = sys_A_ * posvelacc_ ;
  posvelacc_.tail<3>() = cur_ctrlinput_ + cur_extforces_ + gvec;
              //+ sys_B_ * cur_ctrlinput_
              //+ sys_Bext_ * cur_extforces_
              //+ sys_Bext_ * gvec;
  //printf( "current z posvel: %0.2f, %0.2f\n", posvelacc_(2), posvelacc_(5) );
  // std::cout.width(10); std::cout << posvelacc_.transpose() << std::endl;
}

void GenericFlyer::manager_process()
{
  const double dt = step_dt_;
  const int dt_ms = std::round(dt*1000.0);
  keep_alive_.test_and_set();
  printf( "Starting manager process for: %s(%d)\n", name_.c_str(), unique_id_ );

  while( keep_alive_.test_and_set() )
  {
    if( is_ok_ )
    {
      moveRobot(dt);
      //if( onGround() )
      //  arrestMotionGround();
      
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