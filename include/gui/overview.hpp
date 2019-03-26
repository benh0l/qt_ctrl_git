/**
 * @file /include/gui/overview.hpp
 *
 * @brief Qt based overview of the robot's motion.
 *
 * @date November 2018
 **/

#ifndef QTCTRL_GUI_MOTION
#define QTCTRL_GUI_MOTION

#include <QTabWidget>
#include <QGraphicsScene>
#include <list> 
#include <model/state.hpp>

/** @brief This class shows the motion of the robot.
 **
 ** It uses a tab window, one tab displaying the motion's path and
 ** the other the velocity profiles.
 **
 ** @since 0.3.1
 **/
class QtMotionGUI : public QObject {
  Q_OBJECT  // This macro is needed to handle graphical events

  QTabWidget motion_window;  ///< The window containing the displays.
  QGraphicsScene  path_scene;     ///< The scene containing the path.
  /// @brief The scene containing the translation velocity profile.
  QGraphicsScene  trans_vel_scene; 
  /// @brief The scene containing the rotation velocity profile.
  QGraphicsScene  rot_vel_scene;
  
  bool not_first_state;
  State last_state;
  std::list<State*> trajectory_states;
  std::list<State*>::const_iterator trajectory_iterator;

public:
  QtMotionGUI();

  void addTrajectory(const std::list<State*>& trajectory); 

  void addState(const State& state,
		const Qt::GlobalColor motion_color = Qt::blue);

  void addStateAverage(const State& state,
		const Qt::GlobalColor motion_color = Qt::red);

public Q_SLOTS:
  void showHide()
  { motion_window.setVisible( !motion_window.isVisible() ); } 
  
}; // end of class QtMotionGUI

#endif // QTCTRL_GUI_MOTION
