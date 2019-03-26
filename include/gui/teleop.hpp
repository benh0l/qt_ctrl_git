/**
 * @file /include/gui/teleop.hpp
 *
 * @brief Qt based teleoperation widget for <tt>qt_ctrl</tt>.
 *
 * @date July 2018
 **/

#ifndef QTCTRL_GUI_TELEOP
#define QTCTRL_GUI_TELEOP


#include <QGroupBox>
#include <QSignalMapper>
#include <ctrl/keyboard.hpp>
#include <gui/viewCtrl.hpp>

/** @brief This class is a @ref refs_qt_widget "Qt widget" which 
 **        allows to control the @ref KbdCtrl "keyboard controller".
 **
 ** This widget reacts to @ref keyPressed "key pression", 
 ** and modify accordingly the @ref Controller "controller" velocities
 ** (with the @ref KbdCtrl "keyboard controller" @ref KbdCtrl::acc() 
 ** "acceleration" or @ref Controller::stopMotion() "stop" methods).
 **
 ** Due to a bug (key pressions were not detected), buttons were 
 ** added.  A more precise method could be used later.
 **
 ** @since 0.2.1
 **/
class TeleopWidget : private QGroupBox, public ControlWidget {
  Q_OBJECT  // This macro is needed to handle graphical events

  /// @brief This widget draws the velocities' evolutions.
  /// @since 0.2.2
  ViewCtrlWdgt vel_drawing;
  
  /// @brief The @ref refs_qt_signalmapper "Qt object" mapping
  ///        the widget's buttons with the parameter transmitted
  ///        to react().
  QSignalMapper mapper;
  
public:
  /** @brief The constructor needs the main window containing 
   **        this widget, and the ROS node to control.
   ** @param parent        The containing graphical object,
   ** @param control       the controller whose behaviour is shown,
   ** @param motion_model  the motion model.
   **/
  TeleopWidget(QWidget& parent, KbdCtrl& control,
	       const MotionModel& motion_model);

  /// @brief The destructor needs to be redefined.
  ~TeleopWidget() {}
  
  /// @brief Returns the widget used to display the behaviour.
  /// @since 0.3.0
  ///
  /// See @ref refs_qt_widget "Qt widget".
  QWidget& widget() { return *this; }

  // Cf ControlWidget::updateState(const State&)
  void updateState(const State& state)
  { vel_drawing.updateState(state); }

  // Cf ControlWidget::updateCommands(const double&, const double&)
  void updateCommands(const double& trans_vel, const double& rot_vel)
  { vel_drawing.updateCommands(trans_vel, rot_vel); }

protected Q_SLOTS:
  /** @brief This widget reacts to key pression.
   **
   ** The key associated to actions are the following:
   ** the application can be stopped either by pressing 'q' (Quit) 
   ** or 'x' (eXit).
   ** The other possibilities are those of @ref react "react method".
   **
   ** @param event  The event associated to a key pression.
   **
   ** See @ref refs_qt_keyevent.
   **/
  void keyPressEvent(QKeyEvent* event)
  // either react to event's key, or use ControlWidget::keyPressed
  { react( event->key() ); ControlWidget::keyPressed(event); } 

  /// @brief When a key is released, both accelerations stop.
  void keyReleaseEvent(QKeyEvent*) { stopAcc(); }

private Q_SLOTS:
  /// @brief Stops the acceleration of the ROS node.  @since 0.2.2
  void stopAcc() { ((KbdCtrl&)(ctrl)).acc(0,0); }

  /** @brief This widget reacts to commands, which are Qt key values.
   **
   ** The commands are currently fixed, there will be later a dialog 
   ** to change the values associated to each action.
   ** These commands modify the @ref Controller "controller" 
   ** velocities (with the @ref KbdCtrl "keyboard controller" 
   ** @ref KbdCtrl::acc() "acceleration", 
   ** @ref Controller::stopMotion() "stop", 
   ** @ref KbdCtrl::stopTranslation() "stop translation" or 
   ** @ref KbdCtrl::stopRotation() "stop rotation" methods).
   ** The robot can be stopped (zero velocities) by pressing either
   ** '0' or 'End'.
   ** The robot can otherwise change its velocity in two ways:
   ** <table border='0'> <tr> <td> 
   ** <table> <tr> <td> 7</td><td>  8 </td><td> 9</td> </tr> 
   **         <tr> <td> 4</td><td>  5 </td><td> 6</td> </tr> 
   **         <tr> <td> 1</td><td>  2 </td><td> 3</td> </tr> 
   ** </table></td><td> &nbsp; or &nbsp; </td><td>
   ** <table><tr><td></td><td align="center">&uarr;</td><td></td></tr>
   **        <tr>   <td>&larr;</td><td>Home</td><td>&rarr;</td>  </tr> 
   **        <tr><td></td><td align="center">&darr;</td><td></td></tr> 
   ** </table></td> </tr> </table>
   ** Directions are in the robot's frame, up meaning forward
   ** (acceleration) while down means backward (deceleration).
   ** Center ('5' or 'Home') stops the robot's rotation.
   **
   ** @param key  The command, one of Qt key values.
   **
   ** See @ref refs_qt_keyevent.
   **/
  void react(const int& key);
  
}; // end of class TeleopWidget

#endif // QTCTRL_GUI_TELEOP
