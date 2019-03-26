/**
 * @file  src/model/motion.cpp
 *
 * @brief Motion model for <tt>qt_ctrl</tt>.
 *
 * @date  August 2018
 **/

#include <math.h>                   // to get fabs() declaration
#include <model/motion.hpp>


// Descriptive method, giving the limit acceleration which ensure 
// to avoid reaching limit distance for a given distance, velocity 
// and leader velocity.
// Parameter lim_dist   the limit (minimum) distance allowed,
//           dist       the current distance to the leader,
//           vel        the current velocity of the robot,
//           ldr_vel    the current velocity of the leader,
//           time_step  the time step of the motion.
double MotionModel::limAcc(const double& lim_dist, const double& dist,
			   const double& vel, const double& ldr_vel,
			   const double& time_step) const {
  const double ts2 = time_step * time_step,
    min_acc = limits[aMin], max_acc = limits[aMax], 
    delta_acc = (min_acc - max_acc),
    // worst case evaluation of next time step values
    next_dist = dist + (ldr_vel - vel) * time_step
    + delta_acc * ts2 / 2,
    next_ldr_vel = ldr_vel + min_acc * time_step,
    next_vel = vel + max_acc * time_step,
    next_vel_inc = next_vel - min_acc * time_step / 2,
    next_vel_mod = next_vel_inc + max_acc * time_step,
    next_crit = next_dist - lim_dist
    + (next_vel * next_vel - next_ldr_vel * next_ldr_vel)
    / (2 * min_acc),
    // worst case evaluation of 2 time step later value
    next_next_crit = std::max(0., next_crit - delta_acc
			      * (next_vel + max_acc * time_step / 2)
			      / min_acc - delta_acc * ts2);

  return std::min( min_acc + 2 * (next_dist - lim_dist
				  + (next_ldr_vel - next_vel)
				  * time_step) / (3 * ts2),
		   std::min( sqrt( next_vel_inc * next_vel_inc
				   - 2 * min_acc * next_crit
				   - next_vel_inc
				   + min_acc * time_step) / time_step,
			     ( sqrt( next_vel_mod * next_vel_mod
				     - 2 * min_acc * next_next_crit ) 
			       - next_vel) / time_step + delta_acc
			     + min_acc / 2 ) );  
} // end of double MotionModel::limAcc(const double&, ...) -----------

/* Modify translation and rotation given velocities, applying 
 * the extremal accelerations multiplied by the given factors 
 * during the given time step.
 *
 * A factor of the same sign than the velocity is multiplied 
 * by the maximum acceleration to improve the velocity 
 * in absolute value, while a factor whose sign is the opposite 
 * of the velocity's is multiplied by the maximum deceleration
 * to reduce the velocity in absolute value.
 * Extrema of the velocity are taken into account, as well as 
 * a possible change of sign of the velocity during the change
 * (it changes the applied acceleration for the remaining time).
 *
 * Parameter trans_vel   the translation velocity, to update,
 *           rot_vel     the  rotation   velocity, to update,
 *           trans_fact  the translation factor, multiplied by one 
 *                        of the translation acceleration extrema,
 *           rot_fact    the  rotation   factor, multiplied by one 
 *                        of the  rotation   acceleration extrema,
 *           time_step   the duration of the accelerations.
 */
void MotionModel::applyAccelerationFactors
(double& trans_vel, double& rot_vel, const double& trans_fact,
 const double& rot_fact, const double& time_step) const {
  static const int nbVal = 2; // two velocities:
  //  similar computations, using a for(){} with arrays
  static const double min_vel[nbVal] = {limits[vMin], -limits[omMax]},
    max_vel[nbVal] = {limits[vMax], limits[omMax]},
    acc[nbVal]     = {limits[aMax], limits[gmMax]},
    dec[nbVal]     = {limits[aMin], limits[gmMin]};
  const double fact[nbVal] = {trans_fact, rot_fact}; 
  double *vel[nbVal] = {&trans_vel, &rot_vel};
    
  for(int id = 0; id < nbVal; id++) {
    double delta_t = time_step; 
    do { // two passes may be needed (if deccelerating)
      // is acceleration or deceleration applied?
      const bool isAcc = (*vel[id] * fact[id] >= 0);
      const double& delta_a
	= fact[id] * ( isAcc ? acc[id] : fabs(dec[id]) );
      *vel[id] += delta_a * delta_t;
      delta_t = 0;  // remaining time
      if (isAcc) // acceleration: is max/min velocity respected?
	if (fact[id] >= 0) {
	  if (*vel[id] > max_vel[id])  *vel[id] = max_vel[id];
	} else {
	  if (*vel[id] < min_vel[id])  *vel[id] = min_vel[id];
	}
      else // deceleration: did velocity change of sign?
	if (*vel[id] * fact[id] > 0) { // it did! Second pass needed!
	  *vel[id] -= delta_a * delta_t;  // restore velocity
	  // remaining time after change of sign (vel[id] / acc < 0)
	  delta_t = time_step + *vel[id] / delta_a; // delta_t > 0
	  *vel[id] = 0; 
	} // end of if (Second pass needed, with the other acc!)
    } while (delta_t > 0);
  } // end of for (each velocity)
} // end of void MotionModel::applyAccelerationFactors(...) ----------

/* Modify translation and rotation given velocities, applying 
 * the given accelerations during the given time step.
 *
 * Extrema of the velocity and of the acceleration are taken 
 * into account, as well as a possible change of sign of 
 * the velocity during the change (it changes the acceleration 
 *  extrema for the remaining time).
 *
 * Parameter trans_vel   the translation velocity, to update,
 *           rot_vel     the  rotation   velocity, to update,
 *           trans_acc   the translation acceleration,
 *           rot_acc     the  rotation   acceleration,
 *           time_step   the duration of the accelerations.
 */
void MotionModel::applyAccelerations
(double& trans_vel, double& rot_vel, const double& trans_acc,
 const double& rot_acc, const double& time_step) const {
  static const int nbVal = 2; // two velocities:
  //  similar computations, using a for(){} with arrays
  static const double min_vel[nbVal] = {limits[vMin], -limits[omMax]},
    max_vel[nbVal] = {limits[vMax], limits[omMax]},
    max_acc[nbVal] = {limits[aMax], limits[gmMax]},
    max_dec[nbVal] = {limits[aMin], limits[gmMin]};
  const double acc[nbVal] = {trans_acc, rot_acc}; 
  double *vel[nbVal] = {&trans_vel, &rot_vel};
    
  for(int id = 0; id < nbVal; id++) {
    double delta_t = time_step; 
    do { // two passes may be needed (if decelerating)
      // is acceleration or deceleration applied?
      const bool isAcc = (*vel[id] * acc[id] >= 0);
      const double
	sgnAcc = (acc[id] < 0 ? -1 : 1), absAcc = sgnAcc * acc[id], 
	&delta_a = sgnAcc *
	( isAcc ? (absAcc > max_acc[id] ? max_acc[id] : absAcc)
	  : (absAcc > - max_dec[id] ? - max_dec[id] : absAcc) );
      *vel[id] += delta_a * delta_t;
      delta_t = 0;  // remaining time
      if (isAcc) // acceleration: is max/min velocity respected?
	if (delta_a >= 0) {
	  if (*vel[id] > max_vel[id])  *vel[id] = max_vel[id];
	} else {
	  if (*vel[id] < min_vel[id])  *vel[id] = min_vel[id];
	}
      else // deceleration: did velocity change of sign?
	if (*vel[id] * sgnAcc > 0) { // it did! Second pass needed!
	  *vel[id] -= delta_a * delta_t;  // restore velocity
	  // remaining time after change of sign (vel[id] / acc < 0)
	  delta_t = time_step + *vel[id] / delta_a; // delta_t > 0
	  *vel[id] = 0; 
	} // end of if (Second pass needed, with the other acc!)
    } while (delta_t > 0);
  } // end of for (each velocity)
} // end of void MotionModel::applyAccelerations(...) ----------------
