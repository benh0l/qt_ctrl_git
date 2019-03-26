/**
 * @file /include/gui/ctrlWdgt.hpp
 *
 * @brief Qt based control widget for <tt>qt_ctrl</tt>.
 *
 * @date October 2018
 **/

#ifndef QTCTRL_GUI_CONTROL
#define QTCTRL_GUI_CONTROL

#include <QWidget>
#include <QKeyEvent>
#include <QApplication>   // to get qApp definition
#include <ctrl/controller.hpp>

/** @brief This abstract class is inherited by all those containing 
 **        a @ref refs_qt_widget "Qt widget" intended to show 
 **        the @ref Controller "controller"'s behaviour (current and 
 **        aimed velocities).
 **
 ** @since 0.3.0
 **/
class ControlWidget {

protected:
  /// @brief This widget is the GUI of a @ref Controller "controller".
  /// @since 0.2.1
  Controller&  ctrl;
  /// @brief The motion model and its limits values.  @since 0.2.1
  const MotionModel& model; 
  
  /// @brief The constructor is used by inheritors.
  /// @param controller    The controller whose behaviour is shown,
  /// @param motion_model  its motion model (with its limits values).
  ControlWidget(Controller& controller,
		const MotionModel& motion_model)
    : ctrl(controller), model(motion_model) {}
  
public:
  /// @brief The destructor needs to be defined as virtual.
  virtual ~ControlWidget() {}
  
  /// @brief Returns the widget used to display the behaviour.
  /// @since 0.3.0
  ///
  /// See @ref refs_qt_widget "Qt widget".
  virtual QWidget& widget() = 0;

  /// @brief Update the display of the odometry.
  /// @param state  The state of the robot.  @since 0.3.1
  virtual void updateState(const State& state) = 0;

  /// @brief Update the display of the ROS node.
  /// @param trans_vel  the translation velocity sent to ROS,
  /// @param rot_vel    the rotation velocity sent to ROS. 
  /// @since 0.3.1
  virtual void updateCommands(const double& trans_vel,
			      const double& rot_vel) = 0;
  
  /** @brief Handles Quit/eXit shortcuts.
   ** @param event  The event associated to a key pression.
   **
   ** The application can be stopped either by pressing 'q' (Quit) 
   ** or 'x' (eXit).
   **
   ** See @ref refs_qt_keyevent.
   **/
  void keyPressed(QKeyEvent* event) {
  const int key = event->key(); // get the event's key
  if ( (key == Qt::Key_Q) || (key == Qt::Key_X) )  qApp->quit();
  }

  /// @brief Key release can be handled by inheritors.
  void keyReleased(QKeyEvent*) {}
  
}; // end of class ControlWidget

#endif // QTCTRL_GUI_CONTROL
