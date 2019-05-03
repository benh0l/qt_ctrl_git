/**
 ** @file  include/model/CHSI.hpp
 **
 ** @brief Cubic Hermine Spline Interpolation connecting two points
 **        and respecting their derivative.
 **
 ** @date  May 2019
 **/

#ifndef QTCTRL_CHSI
#define QTCTRL_CHSI


#include <model/point.hpp>

/** 
 ** @brief This class defines a Cubic Hermine Spline Interpolation 
 **        connecting two points and respecting their derivative.
 ** 
 ** @since 0.3.2
 */
class CubicHermineSplineInterpolation : public Object {
  
public:
  /// @brief The constructor needs the extremal points and derivatives.
  /// @param firstPt      the first point, 
  /// @param firstDeriv   the derivative at the first point, 
  /// @param secondPt     the second point, 
  /// @param secondDeriv  the derivative at the second point.
  CubicHermineSplineInterpolation
  (const point& firstPt, const vector& firstDeriv,
   const point& secondPt, const vector& secondDeriv);

  /// @brief Gets a point at given parameter on the interpolation.
  /// @param u  the value of the parameter for which a point
  ///           is required.
  /// @return   the point on the interpolation corresponding 
  ///           to the given value of the parameter.
  point operator[](const double& u) const;

  /// @brief Gets a point at given parameter on the interpolation.
  /// @param u  the value of the parameter for which a point
  ///           is required.
  /// @return   the point on the interpolation corresponding 
  ///           to the given value of the parameter.
  /// @see operator[]
  point pointAt(const double& u) const { return operator[](u); }
  
  /// @brief Gets the derivative of the interpolation at a point 
  ///        corresponding to a given parameter.
  /// @param u  the value of the parameter for which the derivative 
  ///           is required.
  /// @return   the derivative of the interpolation at the point 
  ///           corresponding to the given value of the parameter.
  vector derivativeAt(const double& u) const;
  
  /// @brief Gets the point along the interpolation which is 
  ///        the nearest from a given point.
  /// @param P  the point from which a nearest one is searched.
  /// @return   the point along the interpolation nearest from P.
  point nearestFrom(const point& P) const;
  
  /// @brief Gets the distance from a point to the interpolation.
  /// @param P  the point from which distance is searched.
  /// @return   the distance between P and the interpolation.
  /// @see nearestFrom, point::distance
  double distance(const point& P) const
  { return nearestFrom(P).distance(P); }
  
}; // end of class CubicHermineSplineInterpolation

#endif
