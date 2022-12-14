namespace fast_approx
{
  constexpr double pii = 3.14159;
  constexpr double accg = 9.81;
  constexpr double fact3 = 3.0*2.0;
  constexpr double fact5 = 5.0*4.0*fact3;
  constexpr double fact7 = 7.0*6.0*fact5;
  constexpr inline double square(const double &x) { return x*x; }
  constexpr inline double cube(const double &x) { return x*x*x; }
  // fast sign function, returns {+1 or -1}
  #define posneg(x) ((x>0)*2.0-1)
  constexpr inline double fastabs( double &x ) noexcept { return x > 0? x: -x; }
  constexpr inline int fastround(double x) noexcept { return int(x+0.5*posneg(x)); }
  constexpr inline double fastfmod( double x, double y ) noexcept
  {
    return x - int(x/y)*y;
  }
  constexpr inline double constrainAngleRad( double x ) noexcept
  {
    // limit angle to [-pi,pi]
    x = fastfmod(x+pii,2*pii);
    return( x < 0 ? x+pii : x-pii );
  }
  constexpr inline double sine(const double &a) noexcept
    { return a - cube(a)/fact3 + a*a*cube(a)/fact5 - a*cube(a)*cube(a)/fact7; }
  constexpr inline double cosine(const double &a) noexcept
    { return 1.0 - a*a/2.0 + a*cube(a)/(4*fact3) - cube(a)*cube(a)/(6*fact5); }
}