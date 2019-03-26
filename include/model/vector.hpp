/**
 ** @file  include/model/vector.hpp
 **
 ** @brief 2D vector class.
 **
 ** @date  November 2018
 **/

#ifndef QTCTRL_VECTOR
#define QTCTRL_VECTOR


#include <model/object.hpp>

/** 
 ** @brief This class defines a vector in two dimension.
 ** 
 ** @since 0.3.1
 */
class Vector : public Object {
  double XCoord;   ///< First Cartesian coordinate. 
  double YCoord;   ///< Second Cartesian coordinate. 

public:
  /** @brief The default constructor, creating the zero vector 
   **        with Cartesian coordinates (0,0). 
   **/
  Vector() : XCoord(0), YCoord(0) {}


  /** @brief A simple constructor, creating the unit vector 
   **        of given orientation. 
   **
   ** The vector of Polar coordinates <tt>(rho, theta)</tt> 
   ** can thus be obtained using 
   ** <tt>iSeeML::geom::Vector(theta).multiply(rho)</tt>. 
   ** 
   ** @param theta the orientation of the vector. 
   **
   ** @see   orientation, multiply. 
   **/
  Vector(const double& theta) :
    XCoord( cos(theta) ), YCoord( sin(theta) ) {} 

  /** @brief The main constructor, creating a vector from 
   **        its Cartesian coordinates. 
   ** 
   ** @param x the first  Cartesian coordinate of the vector, 
   ** @param y the second Cartesian coordinate of the vector. 
   **
   ** @see   xCoord, yCoord.
   **/
  Vector(const double& x, const double& y) : 
    XCoord(x), YCoord(y) {}

  // The default copy constructor is OK. 

  /** @brief Gives the point's first coordinate. 
   **
   ** @return    the point's first coordinate. 
   **/
  const double& xCoord() const { return XCoord; } 

  /** @brief Gives the point's second coordinate. 
   **
   ** @return    the point's second coordinate. 
   **/
  const double& yCoord() const { return YCoord; } 

  /// @brief Writes the vector's coordinates in an output stream.
  /// @param O  the output stream in which the description is written.
  /// @see ostream::operator<<(const Object&)
  void writeTo(std::ostream& O) const 
  { O << '(' << xCoord() << ", " << yCoord() << ')'; }
  
  /** @brief Gives the vector's orientation.
   **
   ** The couple (@ref length, orientation) gives Polar 
   ** coordinates of the vector, while the couple (@ref xCoord, 
   ** @ref yCoord) gives Cartesian coordinates. 
   ** Orientation is always uniquely defined, except for zero 
   ** vector.
   ** In that case, we choose to return 0.
   **
   ** @return    the vector's orientation, between - @f$\pi@f$ 
   **            (excluded) and @f$\pi@f$ (included), if vector
   **            is not zero, or zero. 
   **
   ** @see   Object::isPositive, Object::isNegative, 
   **        Object::sign, atan.
   **/
  double orientation() const 
  { return
      // if XCoord is positive, direct computation using atan
      ( isPositive(XCoord) ? atan(YCoord / XCoord) : 
	// if XCoord is negative, atan value is corrected 
	// wrt YCoord precise sign 
	isNegative(XCoord) ? ( YCoord < 0 ? 
			       atan(YCoord / XCoord) - M_PI : 
			       atan(YCoord / XCoord) + M_PI ) : 
	// if XCoord is zero, result can be 0 or +/- pi / 2
	sign(YCoord) * M_PI_2 );
  }

  /** @brief Gives the vector's length.
   **
   ** The couple (length, @ref orientation) gives Polar 
   ** coordinates of the vector, while the couple (@ref xCoord, 
   ** @ref yCoord) gives Cartesian coordinates. 
   **
   ** @return    the vector's length. 
   **
   ** @see       sqrLength. 
   **/
  double length() const    { return sqrt( sqrLength() ); }

  /** @brief Gives the square of the vector's length.
   ** 
   ** Uses the scalar product. 
   **
   ** @return    the square of the vector's length. 
   **
   ** @see       operator*(const Vector&). 
   **/
  double sqrLength() const { return( (*this) * (*this) ); }

  /** @brief Changes the vector to a given value. 
   ** 
   ** The original vector is changed. 
   ** 
   ** @param x  the first coordinate of the new position, 
   ** @param y  the second coordinate of the new position. 
   ** 
   ** @return   the vector, after tranformation. 
   **
   ** @see      xCoord, yCoord. 
   **/
  Vector& moveTo(const double& x, const double& y) 
  { XCoord = x;  YCoord = y;  return *this; } 

  /** @brief Translates the vector along an other. 
   ** 
   ** The original vector is changed, addition without 
   ** modification is obtained using operator+(const Vector&) const. 
   ** 
   ** @param v  the vector of the translation.
   **
   ** @return   the new vector, after translation.
   **
   ** @see      add.
   **/
  Vector& translate(const Vector& v) { return( add(v) ); } 

  /** @brief Rotates the vector of a given angle. 
   ** 
   ** The original vector is changed. 
   ** 
   ** @param theta  the angle of the rotation.
   **
   ** @return       the new vector, after rotation.
   **/
  Vector& rotate(const double& theta) { 
    const double x = XCoord, y = YCoord, 
      ct = cos(theta), st = sin(theta); 
    moveTo(x * ct - y * st, x * st + y * ct); 
    return *this; } 

  /** @brief Adds a vector to the current one. 
   ** 
   ** The original vector is changed, addition without 
   ** modification is obtained using operator+(const Vector&) const. 
   ** 
   ** @param v  the vector of the translation.
   **
   ** @return   the new vector, after translation / addition.
   **/
  Vector& add(const Vector& v) 
  { XCoord += v.xCoord();  YCoord += v.yCoord();  return *this; } 

  /** @brief Multiplies a vector by a real factor. 
   ** 
   ** The original vector is changed, multiplication without 
   ** modification is obtained using operator*() const. 
   ** 
   ** @param f  a real factor.
   **
   ** @return   the new vector, after multiplication.
   **/
  Vector& multiply(const double& f) 
  { XCoord *= f;  YCoord *= f;  return *this; } 

  /** @brief Indicates wether a vector is null (both Cartesian 
   **        coordinates are zero). 
   **
   ** @return  whether the vector is null. 
   **
   ** @see     xCoord, yCoord, Object::isZero. 
   **/
  bool isNull() const 
  { return( isZero( xCoord() ) && isZero( yCoord() ) ); } 

  /** @brief Equality operator between vectors (differences between 
   **        vectors is null). 
   **
   ** @param   other   another vector. 
   **
   ** @return  whether the two vectors are equal. 
   **
   ** @see     operator-(const Vector&), isNull(). 
   **/
  bool operator==(const Vector& other) const 
  { return( (other - *this).isNull() ); } 

  /** @brief Sum operator between two vectors, giving the translation 
   **        of the first one by the other.
   **
   ** @param  v  the vector of the translation.
   **
   ** @return    the sum of the two vectors. 
   **
   ** @see       add.
   **/
  Vector operator+(const Vector& v) const
  { Vector res(*this); return res.add(v); } 

  /** @brief Difference operator between two vectors, giving the sum 
   **        of the first vector and of the second one's opposite. 
   **
   ** @param  v  a vector.
   **
   ** @return    the current vector minus the given one. 
   **
   ** @see       operator-(), operator+.
   **/
  Vector operator-(const Vector& v) const
  { return( *this + (-v) ); } 

  /** @brief Opposite operator for a vector, such that the sum of 
   **        the vector and of its opposite is the null vector. 
   **
   ** @return    the opposite of the current vector. 
   **
   ** @see       multiply.
   **/
  Vector operator-() const
  { Vector res(*this); return( res.multiply(-1) ); } 

  /** @brief Multiplication operator between a vector and a real.
   **
   ** @param  f  a real factor.
   **
   ** @return    the product of the current vector by the real 
   **            factor. 
   **
   ** @see       multiply.
   **/
  Vector operator*(const double& f) const
  { Vector res(*this); return res.multiply(f); } 

  /** @brief Multiplication operator between a real and a vector.
   **
   ** @param  f  the real factor, 
   ** @param  v  the vector.
   **
   ** @return    the product between the factor and the vector. 
   **
   ** @see       const operator*(const double&). 
   **/
  friend Vector operator*(const double& f, const Vector& v) 
  { return(v * f); } 

  /** @brief Scalar product operator between two vectors.
   **
   ** @param  v  the second vector of the product.
   **
   ** @return    the scalar product between the current vector 
   **            and the given one. 
   **/
  double operator*(const Vector& v) const
  { return( xCoord() * v.xCoord() + yCoord() * v.yCoord() ); }

  /** @brief Vectorial product operator between two vectors.
   **
   ** @param  v  the second vector of the product.
   **
   ** @return    the vectorial product between the current vector 
   **            and the given one. 
   **/
  double operator^(const Vector& v) const
  { return( xCoord() * v.yCoord() - yCoord() * v.xCoord() ); }

}; // end of class Vector

#endif  // QTCTRL_VECTOR
