/**
 ** @file  include/ctrl/PID.hpp
 **
 ** @brief Path following controller class, using a PID.
 **
 ** @date  November 2018
 **/


#ifndef QTCTRL_PID_CTRL
#define QTCTRL_PID_CTRL

#include <ctrl/track.hpp>


/** 
 ** @brief PIDCtrl uses a PID to compute the trajectory's velocities.
 **
 ** You need to find the right coefficients.
 **
 ** @since 0.3.1
 */
class PIDCtrl : public TrackingCtrl {
  double trans_prop_coef;   ///< Translation proportional coefficient.
  double trans_integ_coef;  ///< Translation integration coefficient.
  double trans_deriv_coef;  ///< Translation derivative coefficient.
  double rot_prop_coef;     ///< Rotation proportional coefficient.
  double rot_integ_coef;    ///< Rotation integration coefficient.
  double rot_deriv_coef;    ///< Rotation derivative coefficient.
  
public:
  /** @brief The constructor needs a motion model, a file name
   **        giving the path to track, an initial configuration and 
   **        the PID coefficients.
   ** 
   ** @param model            the motion model,
   ** @param ts               the time step of the controller,
   ** @param input_file_name  the input file name of the path,
   ** @param init_config      the initial configuration,
   ** @param coef             the coefficient array.
   ** 
   ** @see TrackingCtrl::TrackingCtrl
   **/
  PIDCtrl(const MotionModel& model, const double& ts,
	  const char* input_file_name, const Config& init_config, 
	  const double coef[6])
    : TrackingCtrl(model, ts, input_file_name, init_config),
      trans_prop_coef(coef[0]), trans_integ_coef(coef[1]),
      trans_deriv_coef(coef[2]), rot_prop_coef(coef[3]), 
      rot_integ_coef(coef[4]), rot_deriv_coef(coef[5]) {}
  
  /// @brief The destructor needs to be explicitely redefined.
  virtual ~PIDCtrl() {}
  
  // Cf Controller::choose_velocities(double&, double&, ...)
  virtual void chooseVelocities(double& trans_vel, double& rot_vel,
				std::ostringstream& log_str);
  
}; // end of class PIDCtrl

#endif // QTCTRL_PID_CTRL
