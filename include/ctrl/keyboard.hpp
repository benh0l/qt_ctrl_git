/**
 ** @file  include/ctrl/keyboard.hpp
 **
 ** @brief Keyboard controller (teleoperation) class.
 **
 ** @date  October 2018
 **/


#ifndef QTCTRL_KEYBOARD_CTRL
#define QTCTRL_KEYBOARD_CTRL

#include <ctrl/controller.hpp>

/** 
 ** @brief KbdCtrl allows to control the robot with the keyboard.
 **
 ** Most of fields and methods were initially defined
 ** in @ref ROSnode "ROS node".
 **
 ** @since 0.3.0
 */
class KbdCtrl : public Controller {
  /// Increment factor of translation velocity, in [-1, 1].  @since 0.2.0
  double moving_factor;
  /// Increment factor of rotation velocity, in [-1, 1].  @since 0.2.0
  double turning_factor;
   
public:
  /// @brief The constructor needs a motion model.
  /// @param model  The motion model,
  /// @param ts     the time step of the controller.
  /// @see Controller(const MotionModel&, const double&)
  KbdCtrl(const MotionModel& model, const double& ts)
    : Controller(model, ts) {}
  
  /// @brief The destructor needs to be explicitely redefined.
  virtual ~KbdCtrl() {}
  
  /// @brief  Stops the robot's rotation.  @since 0.2.1
  void stopRotation()  { turning_velocity = 0; }

  /// @brief  Stops the robot's translation.  @since 0.2.2
  void stopTranslation()  { moving_velocity = 0; }

  /** @brief  Changes both velocities (translation and rotation)
   **         by adding or removing the related increment
   **         (acceleration times loop duration).
   **
   ** @param transFact  The translation factor multiplied to 
   **                   the translation increment (the factor 
   **                   is considered as 1 if > 1, and -1 if < -1).
   ** @param rotFact    The rotation factor multiplied to 
   **                   the rotation increment (the factor 
   **                   is considered as 1 if > 1, and -1 if < -1).
   **/
  void acc(const int transFact, const int rotFact)  {
    turning_factor = (rotFact > 1 ? 1 : rotFact < -1 ? -1 : rotFact);
    moving_factor = (transFact > 1 ? 1 : transFact < -1 ? -1
		     : transFact);
  }

  // Cf Controller::chooseVelocities(...)
  void chooseVelocities(double& trans_vel, double& rot_vel,
			std::ostringstream& log_str) {
    motion_model.applyAccelerationFactors
      (moving_velocity, turning_velocity,
       moving_factor, turning_factor, time_step); /*
    log_str << "+(" << moving_factor << " x, " << turning_factor 
    << " x, " << time_step << ")"; */
    updateVelocities(trans_vel, rot_vel);
  }
  
}; // end of class KbdCtrl

#endif // QTCTRL_KEYBOARD_CTRL
