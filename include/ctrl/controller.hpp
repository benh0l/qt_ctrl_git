/**
 ** @file  include/ctrl/controller.hpp
 **
 ** @brief Controller abstract class.
 **
 ** @date  October 2018
 **/


#ifndef QTCTRL_CONTROLLER
#define QTCTRL_CONTROLLER

#include <QObject>
#include <model/motion.hpp>
#include <model/state.hpp>

/** 
 ** @brief Controller is the abstract class inherited by all 
 **        the implemented controllers.
 **
 ** It declares the abstract methods needed 
 ** by the @ref ROSnode "ROS node" for the control to work.
 ** Most of fields and methods were initially defined
 ** in @ref ROSnode "ROS node".
 **
 ** @since 0.3.0
 */
class Controller : public QObject {
  Q_OBJECT  // This macro is needed to handle signals
  
protected:  // for the inheriting controllers...
  /// @brief The model of the motion.  @since 0.2.2
  const MotionModel& motion_model;

  /// @brief The time step of the controller.  @since 0.3.0
  const double time_step;

  /// @brief The first date of the odometry.  @since 0.3.1
  double initial_date;

  /// Translation velocity desired for the robot.  @since 0.2.0
  double moving_velocity;
  /// Rotation velocity desired for the robot.  @since 0.2.0
  double turning_velocity;  
  
  /** @brief This constructor (for the inheriting controllers)
   **        sets the motion model to the given one.
   ** @param model  the motion model,
   ** @param ts     the time step of the controller.
   ** @warning  The time step should be strictly positive (it is taken 
   **           in absolute value, with a minimum value of 1E-9).
   **/
  Controller(const MotionModel& model, const double& ts)
    : motion_model(model),
      time_step( fabs(ts) >= 1E-9 ? fabs(ts) : 1E-9 ) {}
  
  /** @brief Update the velocities from the fields and 
   **        send the update signal.
   ** @param trans_vel  the translation velocity, 
   ** @param rot_vel    the rotation velocity.
   ** @see  moving_velocity, turning_velocity, commandsUpdated.
   **/
  void updateVelocities(double& trans_vel, double& rot_vel) {
    trans_vel = moving_velocity;
    rot_vel   = turning_velocity;
    // signal the GUI to update command's display 
    // low frequency and small data: no copy nor mutex needed!
    commandsUpdated(trans_vel, rot_vel); 
  }
  
public:
  /// @brief The destructor needs to be defined as virtual.
  virtual ~Controller() {}
  
  /// @brief  Stops the robot (i.e. sets both velocities to zero).
  /// @since 0.2.1
  const double& timeStep() const { return time_step; }
  
  /** @brief Handles new state of the robot.
   ** @param state  the new state of the robot, from odometry.
   **
   ** This method can be overwritten by inheritors.
   **/
  virtual void newState(const State& state);

  /// @brief  Stops the robot (i.e. sets both velocities to zero).
  /// @since 0.2.1
  void stopMotion()  { moving_velocity = turning_velocity = 0; }

  /** @brief Computes new velocities for @ref ROSnode "ROS node".
   **
   ** This method has to be defined by inheritors.
   ** It should use @ref updateVelocities.
   **
   ** @param trans_vel  the translation velocity, sent 
   **                   by the @ref ROSnode "ROS node" to ROS,
   ** @param rot_vel    the rotation velocity, sent to ROS,
   ** @param log_str    the log stream, allowing to log debug data.
   **/
  virtual void chooseVelocities(double& trans_vel, double& rot_vel,
				std::ostringstream& log_str) = 0;

  
Q_SIGNALS:  // this requires the Q_OBJECT macro

  /// @brief Update the display of the state.
  /// @param state  The state of the robot.  @since 0.3.1
  void stateUpdated(const State& state);

  /// @brief Update the display of the controller's commands.
  /// @param trans_vel  the translation velocity sent to ROS,
  /// @param rot_vel    the rotation velocity sent to ROS.
  /// @since 0.3.1
  void commandsUpdated(const double& trans_vel,
		       const double& rot_vel);

}; // end of class Controller


/// @brief Default controller, which does not move.  @since 0.3.0
class NoCtrl : public Controller {
public:
  /// @brief The constructor only sets the motion model.
  /// @param model  The motion model.
  NoCtrl(const MotionModel& model) : Controller(model, 1) {}

  /// @brief The destructor needs to be explicitely redefined.
  virtual ~NoCtrl() {}
  
  // Cf Controller::choose_velocities(geometry_msgs::Twist&)
  void chooseVelocities(double& trans_vel, double& rot_vel,
			std::ostringstream&)
  { stopMotion(); updateVelocities(trans_vel, rot_vel); }
  
}; // end of class NoCtrl

#endif // QTCTRL_CONTROLLER
