/**
 * @file  include/model/motion.hpp
 *
 * @brief Motion model for <tt>qt_ctrl</tt>.
 *
 * @date  August 2018
 **/

#ifndef QTCTRL_MOTION
#define QTCTRL_MOTION

#include <iostream>
#include <string.h>  // To get memcpy definition


/** @brief This class defines a motion model, with the motion 
 **        limits.
 **
 ** Rotation velocity has symmetric limits: minimum value is 
 ** the opposite of the maximum value.
 ** Regarding both accelerations (translation and rotation), 
 ** maximum acceleration limits the increase of the absolute value 
 ** of the related velocity, while deceleration concerns the decrease 
 ** of the absolute value.
 **
 ** @since 0.2.2
 **/
class MotionModel {

  /// @brief These are the indexes
  ///        of @ref limits "the array of limits".
  enum Limits {  
    vMin,    ///< Index of the minimum translation velocity.
    vMax,    ///< Index of the maximum translation velocity.
    omMax,   ///< Index of the maximum rotation velocity.
    aMin,    ///< Index of the maximum translation deceleration.
    aMax,    ///< Index of the maximum translation acceleration.
    gmMin,   ///< Index of the maximum rotation deceleration.
    gmMax,   ///< Index of the maximum rotation acceleration.
    nbLimits,    ///< Number of limits in the array.
  };
  
  /// @brief The array of limits.
  double limits[nbLimits]; 

public:
  /** @brief Minimum constructor, supposing that minimum velocities 
   **        and accelerations is the opposite of the maxima.
   **
   ** @param max_v      the maximum translation velocity (and opposite 
   **                    of the minimum translation velocity),
   ** @param max_omega  the maximum rotation velocity,
   ** @param max_a      the maximum translation acceleration (and 
   **                    opposite of the minimum translation 
   **                    acceleration),
   ** @param max_gamma  the maximum rotation acceleration (and opposite
   **                    of the minimum rotation acceleration).
   **/
  MotionModel(const double& max_v, const double& max_omega,
	      const double& max_a, const double& max_gamma)
    : MotionModel(-max_v, max_v, max_omega,
		  -max_a, max_a, -max_gamma, max_gamma) {}
  
  /** @brief General constructor, supposing that minimum velocities 
   **        and accelerations are not the opposite of the maxima.
   **
   ** @param min_v      the minimum translation velocity,
   ** @param max_v      the maximum translation velocity,
   ** @param max_omega  the maximum rotation velocity,
   ** @param min_a      the minimum translation acceleration (or
   **                    maximum translation deceleration),
   ** @param max_a      the maximum translation acceleration,
   ** @param min_gamma  the minimum rotation acceleration (or
   **                    maximum rotation deceleration),
   ** @param max_gamma  the maximum rotation acceleration.
   **/
  MotionModel(const double& min_v, const double& max_v, 
	      const double& max_omega,
	      const double& min_a, const double& max_a, 
	      const double& min_gamma, const double& max_gamma) {
    double values[nbLimits] = { min_v, max_v, max_omega,
				min_a, max_a, min_gamma, max_gamma };
    set(values);
  } // end of MotionModel(nbLimits x const double&)

  /// @brief Descriptive method, giving the minimum translation 
  ///        velocity.  @see vMin.
  const double& minTranslVel() const { return limits[vMin]; }

  /// @brief Descriptive method, giving the maximum translation 
  ///        velocity.  @see vMax.
  const double& maxTranslVel() const { return limits[vMax]; }

  /// @brief Descriptive method, giving the maximum rotation 
  ///        velocity.  @see omMax.
  const double& maxRotVel()    const { return limits[omMax]; }

  /// @brief Descriptive method, giving the minimum translation 
  ///        acceleration (or maximum translation deceleration).
  /// @see aMin.
  const double& minTranslAcc() const { return limits[aMin]; }

  /// @brief Descriptive method, giving the maximum translation 
  ///        acceleration.  @see aMax.
  const double& maxTranslAcc() const { return limits[aMax]; }

  /// @brief Descriptive method, giving the minimum rotation 
  ///        acceleration (or maximum rotation deceleration).
  /// @see gmMin.
  const double& minRotAcc()    const { return limits[gmMin]; }

  /// @brief Descriptive method, giving the maximum rotation 
  ///        acceleration.  @see aMax.
  const double& maxRotAcc()    const { return limits[gmMax]; }

  /// @brief Descriptive method, giving the limit acceleration
  ///        which ensure to avoid reaching limit distance for
  ///        a given distance, velocity and leader velocity.
  /// @param  lim_dist  the limit (minimum) distance allowed,
  /// @param  dist      the current distance to the leader,
  /// @param  vel       the current velocity of the robot,
  /// @param  ldr_vel   the current velocity of the leader,
  /// @param  time_step  the time step of the motion.
  double limAcc(const double& lim_dist, const double& dist,
		const double& vel, const double& ldr_vel,
		const double& time_step) const;

  /** @brief Modify translation and rotation given velocities, 
   **        applying the extremal accelerations multiplied by 
   **        the given factors during the given time step.
   **
   ** A factor of the same sign than the velocity is multiplied 
   ** by the maximum acceleration to improve the velocity 
   ** in absolute value, while a factor whose sign is the opposite 
   ** of the velocity's is multiplied by the maximum deceleration
   ** to reduce the velocity in absolute value.
   ** Extrema of the velocity are taken into account, as well as 
   ** a possible change of sign of the velocity during the change
   ** (it changes the applied acceleration for the remaining time).
   **
   ** @param trans_vel   the translation velocity, to update,
   ** @param rot_vel     the  rotation   velocity, to update,
   ** @param trans_fact  the translation factor, multiplied by one 
   **                     of the translation acceleration extrema,
   ** @param rot_fact    the  rotation   factor, multiplied by one 
   **                     of the  rotation   acceleration extrema,
   ** @param time_step   the duration of the accelerations.
   **/
  void applyAccelerationFactors(double& trans_vel, double& rot_vel,
				const double& trans_fact,
				const double& rot_fact, 
				const double& time_step) const;

  /** @brief Modify translation and rotation given velocities, 
   **        applying the given accelerations during the given 
   **        time step.
   **
   ** Extrema of the velocity and of the acceleration are taken 
   ** into account, as well as a possible change of sign of 
   ** the velocity during the change (it changes the acceleration 
   **  extrema for the remaining time).
   **
   ** @param trans_vel   the translation velocity, to update,
   ** @param rot_vel     the  rotation   velocity, to update,
   ** @param trans_acc   the translation acceleration,
   ** @param rot_acc     the  rotation   acceleration,
   ** @param time_step   the duration of the accelerations.
   **/
  void applyAccelerations(double& trans_vel, double& rot_vel,
			  const double& trans_acc,
			  const double& rot_acc, 
			  const double& time_step) const;
  
  /// @brief Set the motion's limits.
  /// @param values  The motion's limits in an array of @ref nbLimits
  ///                doubles, in the order defined by @ref Limits.
  /// @see limits.
  void set(const double values[nbLimits]) 
  { memcpy(limits, values, sizeof(double) * nbLimits); }
  
}; // end of class MotionModel

#endif // QTCTRL_MOTION
