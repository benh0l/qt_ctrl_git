/**
 ** @file  include/model/object.hpp
 **
 ** @brief Static mathematical methods.
 **
 ** @date  November 2018
 **/


#ifndef QTCTRL_OBJECT
#define QTCTRL_OBJECT

#include <iostream>   // to get ostream 
#include <math.h>     // to get M_PI and fabs

/** 
 ** @brief This class defines static mathematical methods.
 ** 
 ** @since 0.3.1
 */
class Object {
  /** @brief A small value under which doubles are considered as zero.
   ** 
   ** Double values should never be compared to zero, but considered 
   ** as zero if and only if their absolute value is smaller than 
   ** this value. 
   **/ 
  static const double smallDouble; 
   
public:
  /** @brief Method giving the angle between @f$- \pi@f$ 
   ** (excluded) and @f$\pi@f$ (included), which is equal 
   ** to a given angle modulo @f$2 \pi@f$. 
   **
   ** @param theta  an angle. 
   **
   ** @return   the given angle modulo @f$2 \pi@f$, between - 
   **           @f$\pi@f$ (excluded) and @f$\pi@f$ (included). 
   **/
  static double mod2pi(const double& theta) 
  { double res = theta; const double PIx2 = 2 * M_PI; 
    while  (res > M_PI)    res -= PIx2; 
    while (res <= - M_PI)  res += PIx2;  return res; } 

  /** @brief Method transforming an angle in gradian, @f$x \pi@f$ 
   **        with @f$x@f$ between -1 (excluded) and 1 (included), 
   **        into its equivalent in degree, i.e. @f$90 x@f$ . 
   ** @param theta  an angle in gradian. 
   ** @return   the angle in degree. 
   **/
  static double grad2deg(const double& theta) 
  { return theta * 180 / M_PI; } 

  /** @brief Method transforming an angle in degree, @f$90 x@f$ 
   **        with @f$x@f$ between -1 (excluded) and 1 (included), 
   **        into its equivalent in radian, i.e. @f$x \pi@f$ . 
   ** @param theta  an angle in degree. 
   ** @return   the angle in gradian. 
   **/
  static double deg2rad(const double& theta) 
  { return theta * M_PI / 180; } 

  /** @brief Method telling whether a double is strictly positive. 
   **
   ** @param x  the double checked. 
   **
   ** @return   true if the double is bigger than ISeeML' small 
   **           value. 
   **/
  static bool isPositive(const double& x) 
  { return( x >= smallDouble ); } 

  /** @brief Method telling whether a double is strictly negative. 
   ** 
   ** Computes the double's opposite, and checks if it is positive. 
   **
   ** @param x  the double checked. 
   **
   ** @return   true if the opposite of the double is positive. 
   **
   ** @see      isPositive. 
   **/
  static bool isNegative(const double& x) 
  { return( isPositive(- x) ); } 

  /** @brief Method comparing a double to zero. 
   **
   ** @param x  the double compared to zero. 
   **
   ** @return   true if the double's absolute value is not positive. 
   **
   ** @see      isPositive. 
   **/
  static bool isZero(const double& x) 
  { return( !isPositive( fabs(x) ) ); } 

  /** @brief Method giving the sign of a double. 
   **
   ** @param x  the double which sign is searched. 
   **
   ** @return   the sign of the double, as an integer in {-1, 0, 1}. 
   **
   ** @see      isZero.
   **/
  static int sign(const double& x) 
  { return( isZero(x) ? 0 : ( x < 0 ? -1 : 1) ); } 

  /// @brief Writes the object's description in an output stream.
  /// @param O  the output stream in which the description is written.
  /// @see ostream::operator<<(const Object&)
  virtual void writeTo(std::ostream& O) const = 0;
  
}; // end of class Object

/// @brief Writes an object's description in an output stream.
/// @param O  the output stream in which the description is written,
/// @param o  the object whose description is written.
/// @return   the output stream, after writing in it.
/// @see      Object::writeTo
inline std::ostream& operator<<(std::ostream& O, const Object& o) 
{ o.writeTo(O); return O; } 

#endif // QTCTRL_OBJECT
