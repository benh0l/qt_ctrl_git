/**
 * @file /include/gui/viewCtrl.hpp
 *
 * @brief Qt control widget showing current and aimed velocities.
 *
 * @date October 2018
 **/

#ifndef QTCTRL_GUI_CTRL_VIEW
#define QTCTRL_GUI_CTRL_VIEW

#include <gui/ctrlWdgt.hpp>

/** @brief This class is a @ref refs_qt_widget "Qt widget" which 
 **        allows to show the @ref Controller "controller"'s
 **        behaviour (current and aimed velocities).
 **
 ** This widget takes the code of 
 ** the @ref TeleopWidget "teleoperation widget" 
 ** which can be useful to other control widgets.
 **
 ** @since 0.3.0
 **/
class ViewCtrlWdgt : private QWidget, public ControlWidget {
  Q_OBJECT  // This macro is needed to handle graphical events

protected:
  QPointF velocities;        ///< The velocities from the odometry.
  QPointF commands;          ///< The velocities sent as commands.

  /// @brief  Returns the graphical point corresponding to 
  ///         a couple of velocities. 
  /// @param   velocities  The velocities, a couple of double.
  /// @return  The graphical point corresponding to the velocities. 
  QPoint plot(const QPointF& velocities) const;
    
  /// @brief  Returns the graphical region drawn around velocities. 
  /// @param   vel  The velocities, a couple of double.
  /// @return  The graphical region drawn around the point. 
  QRect  region(const QPointF& vel) const {
    static const int offset = 2;  // +/- 2 px, diameter = 5 px
    const QPoint intVel = plot(vel);
    return QRect(intVel, intVel).adjusted(-offset, -offset,
					  offset,   offset);
  }
    
  /// @brief Memorizes a new value in a variable and redraw.
  /// @param variable  either @ref velocities or @ref commands,
  /// @param value     the new value for this variable.
  void modify(QPointF& variable, const QPointF& value) {
    // region modified in the drawing (union of two regions)
    const QRect  change  = region(variable) | region(value);
    variable = value;    // update the variable
    update(change);      // update the drawing
  }

protected Q_SLOTS:
  /// @brief Draws the @ref Controller "controller"'s velocities 
  ///        and their evolution. 
  /// @param event  The drawing event, giving the region
  ///               which should be redrawn.
  void paintEvent(QPaintEvent *event);

public:
  /// @brief The constructor needs the containing graphical object,
  ///        the motion model and the related controller.
  /// @param parent        The containing graphical object,
  /// @param control       the controller whose behaviour is shown,
  /// @param motion_model  the motion model.
  /// 
  /// See @ref refs_qt_widget.
  ViewCtrlWdgt(QWidget& parent, Controller& control,
	       const MotionModel& motion_model);

  /// @brief The destructor needs to be redefined.
  ~ViewCtrlWdgt() {}
  
  /// @brief Returns the widget used to display the behaviour.
  /// @since 0.3.0
  ///
  /// See @ref refs_qt_widget "Qt widget".
  QWidget& widget() { return *this; }
  
  // Cf ControlWidget::updateState(const State&)
  void updateState(const State& state)
  { modify( velocities, QPointF( state.translationVelocity(),
				 state.rotationVelocity() ) ); }

  // Cf ControlWidget::updateCommands(const double&, const double&)
  void updateCommands(const double& trans_vel, const double& rot_vel)
  { modify( commands, QPointF(trans_vel, rot_vel) ); }

}; // end of class ViewCtrlWdgt

#endif // QTCTRL_CTRL_VIEW
