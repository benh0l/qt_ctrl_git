/**
 ** @file  include/ctrl/analytic.hpp
 **
 ** @brief Path following controller class, with an analytic approach.
 **
 ** @date  November 2018
 **/


#ifndef QTCTRL_ANALYTIC_CTRL
#define QTCTRL_ANALYTIC_CTRL

#include <ctrl/track.hpp>


/** 
 ** @brief AnalyticCtrl compute the trajectory's velocities 
 **        using analytic geometry and mecanic.
 **
 ** You need to finish the computation.
 **
 ** @since 0.3.1
 */
class AnalyticCtrl : public TrackingCtrl {
  // === Adding fields may be useful =================================

public:
  /** @brief The constructor needs a motion model, a file name
   **        giving the path to track and an initial configuration.
   ** 
   ** @param model            the motion model,
   ** @param ts               the time step of the controller,
   ** @param input_file_name  the input file name of the path,
   ** @param init_config      the initial configuration.
   ** 
   ** @see TrackingCtrl::TrackingCtrl
   **/
  AnalyticCtrl(const MotionModel& model, const double& ts,
	       const char* input_file_name, const Config& init_config)
    : TrackingCtrl(model, ts, input_file_name, init_config)
      /* === add fields initialisation if needed === */ {}
  
  /// @brief The destructor needs to be explicitely redefined.
  virtual ~AnalyticCtrl() {}
  
  // Cf Controller::choose_velocities(double&, double&, ...)
  virtual void chooseVelocities(double& trans_vel, double& rot_vel,
				std::ostringstream& log_str);
  
}; // end of class AnalyticCtrl

#endif // QTCTRL_ANALYTIC_CTRL

