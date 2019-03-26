/**
 * @file  src/ctrl/imitate.cpp
 *
 * @brief Path following controller class, copying the velocities.
 *
 * @date  November 2018
 **/


#include <ctrl/imitate.hpp>


// Cf Controller::chooseVelocities(...)
void ImitateCtrl::chooseVelocities(double& trans_vel, double& rot_vel,
				   std::ostringstream& log_str) {
  searchGoal();  // updates the goal to be after the robot's date
  // gets the goal's velocities
  moving_velocity = (*goal)->translationVelocity(); 
  turning_velocity = (*goal)->rotationVelocity(); 
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void ImitateCtrl::chooseVelocities(double&, double&, ...)-
