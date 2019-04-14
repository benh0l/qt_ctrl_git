/**
 ** @file  include/ctrl/adrc.hpp
 **
 ** @brief Path following controller class, copying the velocities.
 **
 ** @date  November 2018
 **/


#ifndef QTCTRL_ADRC_CTRL
#define QTCTRL_ADRC_CTRL

#include <ctrl/track.hpp>

/** 
 ** @brief ImitateCtrl copies the trajectory's velocities.
 **
 ** As velocities are directly copied, any error in the beginning
 ** will gets bigger and bigger.
 **
 ** @since 0.3.1
 */
class ADRCCtrl : public TrackingCtrl {
public:
  /** @brief The constructor needs a motion model, a file name
   **        giving the path to track and an initial configuration.
   ** 
   ** States from the path are projected into the initial 
   ** configuration's frame.
   ** 
   ** @param model            the motion model,
   ** @param ts               the time step of the controller,
   ** @param input_file_name  the input file name of the path,
   ** @param init_config      the initial configuration.
   ** 
   ** @see TrackingCtrl::TrackingCtrl
   **/
  ADRCCtrl(const MotionModel& model, const double& ts,
	      const char* input_file_name, const Config& init_config)
    : TrackingCtrl(model, ts, input_file_name, init_config) {}
  
  /// @brief The destructor needs to be explicitely redefined.
  virtual ~ADRCCtrl() {}
  
  // Cf Controller::choose_velocities(double&, double&, ...)
  virtual void chooseVelocities(double& trans_vel, double& rot_vel,
				std::ostringstream& log_str);
  
}; // end of class ImitateCtrl

#endif // QTCTRL_ADRC_CTRL
