/**
 ** @file  include/model/point.hpp
 **
 ** @brief 2D point class.
 **
 ** @date  November 2018
 **/

#ifndef QTCTRL_POINT
#define QTCTRL_POINT


#include <model/vector.hpp>

/** 
 ** @brief This class defines a point in two dimension.
 ** 
 ** @since 0.3.1
 */
class Point : public Object {
  double XCoord;   ///< First coordinate. 
  double YCoord;   ///< Second coordinate. 

public:
  /// @brief The default constructor, returning the frame origin (0,0). 
  Point() : XCoord(0), YCoord(0) {}

  /** @brief The main constructor, creating a point from 
   **        its Cartesian coordinates. 
   ** 
   ** The point of Polar coordinates <tt>(r, theta)</tt>
   ** can be obtained using the formula <tt>Point().translate(v)</tt>,
   ** where <tt>v</tt> is the vector of Polar coordinates <tt>(rho, 
   **  theta)</tt>.
   ** 
   ** @param x the first  Cartesian coordinate of the point, 
   ** @param y the second Cartesian coordinate of the point. 
   **
   ** @see     xCoord, yCoord, Vector. 
   **/
  Point(const double& x, const double& y) 
    : XCoord(x), YCoord(y) {}

  // Default copy constructor is OK.

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

  /// @brief Writes the point's coordinates in an output stream.
  /// @param O  the output stream in which the description is written.
  /// @see ostream::operator<<(const Object&)
  void writeTo(std::ostream& O) const 
  { O << '(' << xCoord() << ", " << yCoord() << ')'; }
  
  /** @brief Moves the point to a given position. 
   ** 
   ** The original point is changed. 
   ** 
   ** @param x  the first coordinate of the position, 
   ** @param y  the second coordinate of the position. 
   ** 
   ** @return   the point, after tranformation. 
   **
   ** @see      xCoord, yCoord. 
   **/
  Point& moveTo(const double& x, const double& y) 
  { XCoord = x;  YCoord = y;  return *this; } 

  /** @brief Translates the point along a vector. 
   ** 
   ** The original point is changed (addition without modification 
   ** is obtained using operator+(const iSeeML::geom::Vector&) const). 
   ** 
   ** @param v  the vector of the translation.
   ** 
   ** @return   the point, after translation. 
   **/
  Point& translate(const Vector& v)
  { XCoord += v.xCoord();  YCoord += v.yCoord();  return *this; } 

  /** @brief Equality operator between points. 
   **
   ** @param   other   another point. 
   **
   ** @return  whether the two points are equal. 
   **
   ** @see     operator-(const Point&), isNull(). 
   **/
  bool operator==(const Point& other) const 
  { return( (other - *this).isNull() ); } 


  /** @brief Sum operator between a point and a vector, giving 
   ** the translation of the current point along the vector. 
   ** 
   ** @param  v  the vector of the translation.
   ** 
   ** @return    the translated point. 
   **
   ** @see       translate. 
   **/
  Point operator+(const Vector& v) const
  { Point res(*this);  return res.translate(v); }

  /** @brief Difference operator between a point and a vector, giving 
   ** the translation of the current point along the opposite of 
   ** the vector. 
   ** 
   ** @param  v  the opposite vector of the translation.
   ** 
   ** @return    the translated point. 
   **
   ** @see       operator+, Vector::operator-(). 
   **/
  Point operator-(const Vector& v) const 
  { return( *this + (-v) ); }

  /** @brief Difference operator between two points, giving the vector 
   ** connecting these points. 
   **
   ** @param  other  another point. 
   **
   ** @return        the vector going from the second given point 
   **                to the first one (the current). 
   **/
  Vector operator-(const Point& other) const 
  { return( Vector( xCoord() - other.xCoord(), 
		    yCoord() - other.yCoord() ) ); } 

  /** @brief Gives the distance between two points. 
   **
   ** It simply computes the length of the connecting vector. 
   **
   ** @param  other  another point. 
   **
   ** @return        the distance between the two points. 
   **
   ** @see       operator-(const iSeeML::geom::Point&), 
   **            Vector::length. 
   **/
  double distance(const Point& other) const 
  { return( sqrt( sqrDist(other) ) ); } 

  /** @brief Gives the square of the distance between two points. 
   **
   ** It simply computes the square length of the connecting vector. 
   **
   ** @param  other  another point. 
   **
   ** @return        the square of the distance between the two points. 
   **
   ** @see       operator-(const iSeeML::geom::Point&), 
   **            Vector::sqrLength. 
   **/
  double sqrDist(const Point& other) const 
  { return( (other - *this).sqrLength() ); } 

}; // end of class Point

#endif // QTCTRL_POINT
