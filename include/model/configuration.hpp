/**
 ** @file  include/model/configuration.hpp
 **
 ** @brief Configuration class, i.e. oriented point in two dimension.
 **
 ** @date  November 2018
 **/

#ifndef QTCTRL_CONFIG
#define QTCTRL_CONFIG


#include <model/point.hpp>

/** 
 ** @brief This class defines a configuration, i.e. an oriented point 
 **        in two dimension.
 ** 
 ** @since 0.3.1
 */
class Config : public Object {
  Point pos;      ///< Position of the reference point.
  double orient;  ///< Orientation of the main axis.

public:
  /** @brief The default constructor should only be used for array 
   **        initializations:  it correspond to a default point with 
   **        zero orientation. 
   ** 
   ** @see  default constructor of Point. 
   **/
  Config() : pos(), orient(0) {} 

  /** @brief The main constructor. 
   ** 
   ** @param P     the position of the reference point, 
   ** @param theta the orientation of the main axis, taken 
   **              between - @f$\pi@f$ (excluded) and @f$\pi@f$
   **              (included) modulo 2 @f$\pi@f$. 
   **
   ** @see   Object::mod2pi.
   **/
  Config(const Point& P, const double& theta) 
    : pos(P), orient( mod2pi(theta) ) {}
	 
  // The default copy constructor is OK. 

  /** @brief Gives the position of the reference point 
   **        of this configuration. 
   **
   ** @return  the position of the reference point. 
   **/
  const Point& position() const { return pos; } 

  /** @brief Gives the orientation of this configuration. 
   **
   ** @return    the orientation of the main axis. 
   **/
  const double& orientation() const { return orient; }

  /// @brief Writes the configuration's values in an output stream.
  /// @param O  the output stream in which the description is written.
  /// @see ostream::operator<<(const Object&)
  void writeTo(std::ostream& O) const {
    O << '(' << position().xCoord() << ", "
      << position().yCoord() << ", " << orientation() << ')';
  }

  /** @brief Turns the current oriented point to its opposite:
   **        the position does not change but the orientation 
   **        is replaced by its opposite (@f$\pm@f$ @f$\pi@f$ 
   **        is added to it). 
   **
   ** @return  the new oriented point, one transformed 
   **          to its opposite. 
   **/
  Config& uTurn() { 
    // do not use rotate to avoid unusefull checks
    orient += (orient > 0 ? - M_PI : M_PI); 
    return *this; } 

  /** @brief Projects a point in the frame of this configuration.
   **
   ** @param  P  the point projected into this configuration's frame. 
   **
   ** @return  the new point, once projected. 
   **
   ** @see projection, Point(), Point::operator+(const Vector&)
   **/
  Point& project(Point& P) const
  { P = Point() + projection(P); return P; } 

  /** @brief Projects an oriented point in the frame of  
   **        this configuration.
   **
   ** @param  q  the configuration projected into 
   **            this configuration's frame. 
   **
   ** @return  the new configuration, once projected. 
   **/
  Config& project(Config& q) const { q = projection(q); return q; } 

  /** @brief Gives the projection of a point in the frame of 
   **        this configuration.
   **
   ** @param  P  the point projected into this configuration's frame. 
   **
   ** @return  the vector from the origin to the projected point.
   **
   ** @see Point::operator-(const Point&), Vector::rotate, 
   **      position, orientation
   **/
  Vector projection(const Point& P) const
  { return ( P - position() ).rotate( -orientation() ); } 

  /** @brief Projects a configuration in the frame of  
   **        this configuration.
   **
   ** @param  q  the configuration projected into 
   **            this configuration's frame. 
   **
   ** @return  the new configuration, once projected. 
   **
   ** @see Config(const Point&, const double&), position, 
   **      projection(const Point&), orientation
   **/
  Config projection(const Config& q) const
  { return Config( Point() + projection( q.position() ),
		   q.orientation() - orientation() ); } 

  /** @brief Equality operator between configurations. 
   **
   ** @param  other  another configuration. 
   **
   ** @return        whether the two configurations are equal 
   **                (same position and orientation). 
   **
   ** @see  position, Point::operator==, isParallelTo. 
   **/
  bool operator==(const Config& other) const { 
    return( ( position() == other.position() ) && 
	    isParallelTo(other) ); } 

  /** @brief Difference operator between configurations. 
   **
   ** @param  other  another configuration. 
   **
   ** @return        whether the two configurations are different. 
   **
   ** @see  operator==. 
   **/
  bool operator!=(const Config& other) const
  { return(! (*this == other) ); } 

  /** @brief Gives the opposite configuration of the current one:
   **        the opposite configuration has the same position
   **        but opposite orientation (current configuration's 
   **         orientation @f$\pm@f$ @f$\pi@f$). 
   **
   ** @return  the opposite configuration of the current one. 
   **
   ** @see  uTurn.
   **/
  Config opposite() const 
  { Config res(*this);  return res.uTurn(); } 

  /** @brief Checks whether two configurations are parallel, i.e. 
   **        whether their orientations are equal (modulo @f$2 pi@f$).
   **
   ** @param  other  another configuration. 
   **
   ** @return        whether the current configuration and the given 
   **                one have same orientation. 
   **
   ** @see  orientation, Object::mod2pi, Object::isZero.
   **/
  bool isParallelTo(const Config& other) const 
  { return( isZero( mod2pi( orientation() - 
			    other.orientation() ) ) ); } 
 
  /** @brief Checks whether two configurations are symmetric, i.e. 
   **        whether their orientations are symmetric wrt the line 
   **        segment connecting their position. 
   ** 
   ** The vector connecting the positions and the vector of average 
   ** orientation should be collinear: their vectorial product 
   ** should be zero. 
   **
   ** @param  other  another configuration. 
   **
   ** @return        whether the current configuration and the given 
   **                one are symmetric. 
   **
   ** @see  position, orientation, Vector::operator^, Object::isZero.
   **/
  bool isSymmetricTo(const Config& other) const
   { return( isZero( ( position() - other.position() ) ^  
		     Vector(( orientation() + 
			      other.orientation() ) / 2) ) ); } 

  /** @brief Checks whether two configurations are aligned: 
   **        their orientations should be equal, and be the same 
   **        as the orientation of the segment connecting 
   **        their positions. 
   ** 
   ** The configurations are aligned iff they are simultaneously 
   ** parallel and symmetric. 
   **
   ** @return        whether the current configuration and the given 
   **                one are aligned. 
   **
   ** @see  isParallelTo, isSymmetricTo. 
   **/
  bool isAlignedWith(const Config& other) const
  { return( isParallelTo(other) && isSymmetricTo(other) ); }
  
  /** @brief Checks whether a point is in the front half-plane of 
   **        the current configuration: the vector connecting 
   **        the current configuration's position to the given 
   **        point should make an angle between @f$- \pi / 2@f$ and 
   **        @f$\pi / 2@f$ with the current configuration's 
   **        orientation. 
   **
   ** The vector going from the configuration's position to the point 
   ** and the vector of same orientation as the configuration should 
   ** have a positive scalar product.
   **
   ** @param  point  a point. 
   **
   ** @return        whether the given point is in the front 
   **                half-plane of the current configuration. 
   **
   ** @see  position, orientation, Point::operator-, 
   **       Vector::operator*(const Vector&).
   **/
  bool hasInFront(const Point& point) const 
  { return( ( point - position() ) * Vector( orientation() ) >= 0 ); }
  
}; // end of class Config

#endif // QTCTRL_CONFIG
