/**
 * @file  src/ctrl/PID.cpp
 *
 * @brief Path following controller class, using a PID.
 *
 * @date  November 2018
 **/


#include <ctrl/PID.hpp>


// Cf Controller::chooseVelocities(...)
void PIDCtrl::chooseVelocities(double& trans_vel, double& rot_vel,
			       std::ostringstream& log_str) {
  static double integr_dist = 0, integr_angle_diff = 0; 
  // updates the goal to be one time step after the robot's date
  searchGoal(time_step);  
  // estimate the distance and angle difference to goal's config,
  // as well as its derivative and its integration
  const Config& goal_config = (*goal)->configuration(),
    config = state.configuration();
  const Vector v = goal_config.position() - config.position(); 
  const double dist = v.length(), dir = v.orientation(),
    // === Following formula may be changed ==========================
    angle_diff = Object::mod2pi( dir + goal_config.orientation()
				 - config.orientation() ),
    trans_vel_diff = (*goal)->translationVelocity()
    - state.translationVelocity(),
    rot_vel_diff = (*goal)->rotationVelocity()
    - state.rotationVelocity();
  // =================================================================
  integr_dist += dist * time_step;
  integr_angle_diff += angle_diff * time_step; 
  // computes the accelerations
  const double trans_acc = trans_prop_coef * dist + trans_deriv_coef 
    * trans_vel_diff + trans_integ_coef * integr_dist,
    rot_acc = rot_prop_coef * angle_diff + rot_deriv_coef 
    * rot_vel_diff + rot_integ_coef * integr_angle_diff;
  // gets the goal's velocities
  motion_model.applyAccelerations
    (moving_velocity, turning_velocity,
     trans_acc, rot_acc, time_step);
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void PIDCtrl::chooseVelocities(double&, double&, ...) ----
