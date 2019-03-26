/**
 * @file  src/ctrl/file.cpp
 *
 * @brief File controller (teleoperation) class.
 *
 * @date  October 2018
 **/


#include <model/object.hpp>
#include <ctrl/file.hpp>


// Cf Controller::chooseVelocities(...)
void FileCtrl::chooseVelocities(double& trans_vel, double& rot_vel,
				std::ostringstream& log_str) {
  static double remain_dur = 0, duration, moving_acc, turning_acc; 
  if ( inputOK() ) {
    if ( Object::isZero(remain_dur) ) {
      // read new line in input stream
      *input >> duration >> moving_acc >> turning_acc;
      if ( inputEnded() )
	{ moving_acc = turning_acc = 0; duration = 10; }
      remain_dur += duration;
    }
    motion_model.applyAccelerations
      (moving_velocity, turning_velocity,
       moving_acc, turning_acc, time_step); /*
    log_str << "+(" << moving_acc << ", " << turning_acc
    << ", " << time_step << "/" << remain_dur << ")"; */
    updateVelocities(trans_vel, rot_vel); 
    remain_dur -= time_step;
  } // end of if (input OK)
} // end of void FileCtrl::chooseVelocities(double&, double&, ...) ---
