/**
 ** @file  src/ctrl/analytic.cpp
 **
 ** @brief Path following controller class, with an analytic approach.
 **
 ** @date  November 2018
 **/


#include <ctrl/analytic.hpp>


// Cf Controller::chooseVelocities(...)
void AnalyticCtrl::chooseVelocities(double& trans_vel, double& rot_vel,
				    std::ostringstream& log_str) {
  // === Following formula have to be changed ========================
  const Config& goal_config = (*goal)->configuration(),
    config = state.configuration();
  const Vector v = goal_config.position() - config.position(); 
  const double dist = v.length(), vel = state.translationVelocity(),
    goal_vel = (*goal)->translationVelocity(),
    max_curv = 1,  // === get a better value !!! ===
    trans_acc = motion_model.limAcc(0, dist, vel,
				    goal_vel, time_step),
    rot_acc = 0;   // === get a better value !!! ===
  // =================================================================
  // gets the goal's velocities
  motion_model.applyAccelerations
    (moving_velocity, turning_velocity,
     trans_acc, rot_acc, time_step);
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void AnalyticCtrl::chooseVelocities(double&, ...) --------
