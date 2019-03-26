/**
 * @file /include/gui/window.hpp
 *
 * @brief Qt based GUI for <tt>@ref index "qt_ctrl"</tt> package.
 *
 * @date July 2018
 **/

#ifndef QTCTRL_GUI
#define QTCTRL_GUI


#include <QMainWindow>
#include <QMenuBar>
#include <QToolBar>
#include <QAction>
#include <QProcess>
#include <QListView>
#include <QEvent>
#include <gui/ctrlWdgt.hpp>
#include <gui/display.hpp>

// State class is used as parameters in this GUI's slots.
// This macro is needed before the call to qRegisterMetaType in
// the constructor.
Q_DECLARE_METATYPE(State) 

/** @brief This class is the @ref refs_qt_mainwindow "Qt main window" 
 **        of <tt>@ref index "qt_ctrl"</tt> package.
 **
 ** It contains (from left to right and top to bottom ) a @ref 
 ** ControlWidget "control widget" (connected to a @ref Controller 
 ** "controller"), a @ref DataWidget "data widget" displaying odometry 
 ** and commands' values, and a @ref refs_qt_listview "Qt list view" 
 ** showing the logs of the @ref refs_ros_console "console outputs" 
 ** sent to ROS by the @ref ROSnode "ROS node".
 **
 ** @since 0.2.0
 **/
class QtCtrlGUI : public QMainWindow {
  Q_OBJECT  // This macro is needed to handle graphical events

  /** @brief This class is optionally used to start a ROS server.
   ** 
   ** It is a @ref refs_qt_thread "Qt thread", to run in a separate
   ** thread.  @since 0.2.1
   **/
  class ROSserver : public QThread {
    // Q_OBJECT macro should not be needed (no connect call)
    
    /// @brief Should the ROS server continue to run?  
    bool ROSrun;  // solution 2

    /// @brief Is the  empty workspace selected?  @since 0.2.4
    bool empty_workspace;  

    /// @brief The main method of this thread.
    void run() {  // no exec() call = no event handling
      // seperate the terminal command from what it should start
      const QString termCmd("xterm -e %1"),
	workspace(empty_workspace ? "qt_ctrl void" :
		  "turtlebot_gazebo turtlebot"), 
	ROScmd("roslaunch %1_world.launch");;
      QProcess ROSprocess;
      ROSprocess.start( termCmd.arg( ROScmd.arg(workspace) ) );
      //exec();            // solution 1, does not stop ROSprocess
      while (ROSrun);      // wait for a ROSserver::stop() (sol. 2)
      ROSprocess.close();  // OK with solution 2
    }  // end of void run()

  public:
    /// @brief Starts a ROS server in a separate thread.
    /// @param parent the process which starts a ROS server.
    ROSserver(QObject* parent) : QThread(parent), ROSrun(true) {}

    /** @brief Set the workspace to be either empty or default.
     ** @param empty  True if the empty workspace is selected, 
     **               false if the default one is chosen.  @since 0.2.4
     **/
    void setEmptyWorkspace(const bool& empty)
    { empty_workspace = empty; }

    /** @brief Waits for a given duration, in seconds.
     ** @param seconds  the duration in seconds.
     ** @see QThread::sleep().
     **/
    void sleep(const long& seconds) const { QThread::sleep(seconds); }

    /// @brief Stops the ROS server (and the thread's execution).
    void stop() { 
      ROSrun = false;   // ask to the run() method to stop  (sol. 2)
      quit();  wait();  // when stopped, stop the thread
    } // end of void stop()
    
  }; // end of class ROSserveur

  /// @brief The default time step.  @since 0.3.1
  static const double time_step; 

  /// @brief The @ref refs_qt_menubar "menu bar" of this GUI.
  /// @since 0.2.1
  QMenuBar menu_bar;
  
  /// @brief The actions' @ref refs_qt_menu "menu" of this GUI.
  /// @since 0.2.1
  QMenu app_menu;
  
  /// @brief The help @ref refs_qt_menu "menu" of this GUI.
  /// @since 0.2.1
  QMenu help_menu;
  
  /** @brief The @ref refs_qt_toolbar "tool bar" of this GUI.
   **
   ** Even though it is not used by this class, it is by Qt
   ** and has to exist as long as the window does.  @since 0.2.1
   **/
  QToolBar tool_bar;
  
  /// @brief The @ref refs_qt_action "Qt action" starting 
  ///        the @ref ROSnode "ROS node".  @since 0.2.1
  QAction  start_stop_action;
  
  /// @brief The @ref refs_qt_action "Qt action" starting 
  ///        a dialog box about Qt Ctrl.  @since 0.2.1
  QAction  about_action;

  /// The instance used to start a ROS server, if needed.
  ROSserver ROS_server;
  
  /// @brief The @ref refs_qt_action "Qt action" starting 
  ///        a dialog box about Qt.  @since 0.2.1
  QAction  about_Qt_action;

  
  /// @brief The motion model of the controlled robot.  @since 0.2.2
  MotionModel   motion_model;

  /// @brief The controller of the @ref ROSnode "ROS node" can change.
  /// @since 0.3.0
  Controller* controller;
  
  /// @brief The ROS node at the center of this GUI.  @since 0.2.2
  ROSnode       ctrl_node;
  
  /// @brief The widget showing the controller's velocities 
  ///        can also change. @since 0.2.1
  ControlWidget*  ctrl_wdgt;

  /// @brief The data display widget of the @ref ROSnode "ROS node".
  /// @since 0.2.2
  DataWidget     display;

  /// @brief The @ref refs_qt_listview "Qt list view" showing 
  ///        the logs of the @ref ROSnode "ROS node".
  QListView     logs;
  
  void readSettings();   ///< Load up Qt program settings at startup.
  void writeSettings();  ///< Save Qt program settings when closing.
  
  /// @brief Connects the controller signals to this class' methods.
  ///
  /// This has to be donne each time the controller is changed.
  void connectCtrl() {
    connect( controller, SIGNAL( stateUpdated(const State&) ),
	     this, SLOT( updateState(const State&) ) );
    connect( controller,
	     SIGNAL( commandsUpdated(const double&, const double&) ),
	     this, SLOT( updateCommands(const double&, const double&)
			 ) );
  }
  
  // @brief Asks in a dialog for a set of doubles, and returns it.
  // @param nb_lines  the number of lines in the dialog,
  // @param nb_col    the number of columns in the dialog,
  // @param nameVal   the array of names displayed,
  // @param isGreek   the array indicating wether name is in Greek,
  // @param val       the array of doubles to get from the user.
  // @pre  the arrays need to have at least nb_lines x nb_col
  //       elements, or a segmentation violation may arrise.
  /// @since 0.3.1
  void getDoubleArray(const int nb_lines, const int nb_col, 
		      const char* nameVal[], 
		      const bool isGreek[], double val[]);
public:
  /** @brief The constructor requires <tt>main()</tt>'s arguments 
   **        (they can be forwarded by the @ref ROSnode "ROS node" 
   **         to ROS).
   **
   ** @param argc    the arguments' count,
   ** @param argv    the arguments' value,
   ** @param parent  the main widget parent, if any (optional).
   **
   ** @see ROSnode::ROSnode().
   **/
  QtCtrlGUI(const int argc, char** argv, QWidget *parent = 0);

  /// @brief Freeing the allocated pointer. 
  //  Freeing the Qt widget pointer is OK, the other one generates
  //  a segmentation fault (?!?).
  ~QtCtrlGUI() { if ( ROS_server.isRunning() )  ROS_server.stop();
    delete ctrl_wdgt; /* delete controller; */ }

  /// @brief The motion model of the controlled robot.  @since 0.2.2
  const MotionModel&  motionModel() const { return motion_model; }
  
protected Q_SLOTS:
  /** @brief The closing method of @ref refs_qt_mainwindow 
   **        is surcharged to save settings before closing.
   **
   ** @param event  the close event, sent to the closing method 
   **               of @ref refs_qt_mainwindow.
   **
   ** See  writeSettings(), @ref refs_qt_mainwindow.
   **/
  void closeEvent(QCloseEvent* event) {
    if ( ctrl_node.connected() )
      // simplified version of buttonStartStop()
      { ctrl_node.end(); ROS_server.stop(); }
   writeSettings(); QMainWindow::closeEvent(event);
   qApp->closeAllWindows();
  }

  /** @brief All key press events are transmitted 
   **        to the @ref ControlWidget "control widget".
   ** @param event  the key press event, sent to @ref 
   **               ControlWidget::keyPressed "the related method"
   **               of the @ref ControlWidget "control widget".
   ** @since 0.2.1
   **/
  void keyPressEvent(QKeyEvent* event)
  { ctrl_wdgt->keyPressed(event); }

  /** @brief When a key is released, it is transmitted
   **        to the @ref ControlWidget "control widget".
   ** @param event  the key release event, sent to @ref 
   **               ControlWidget::keyReleased "the related method"
   **               of the @ref ControlWidget "control widget".
   ** @since 0.2.1
   **/
  void keyReleaseEvent(QKeyEvent* event)
  { ctrl_wdgt->keyReleased(event); }

private Q_SLOTS:
  /// @brief Method activated by the "About" menu item (not yet used).
  void actionAbout();

  /// @brief Method activated by the "Start/Stop" button. @since 0.2.1
  void buttonStartStop();

  /// @brief Method activated when this GUI's log view is updated.
  void updateLogging() { logs.scrollToBottom(); }

  /// @brief Update the display of the state.
  /// @param state  The state of the robot.  @since 0.3.1
  void updateState(const State& state)
  { ctrl_wdgt->updateState(state); display.updateState(state); } 

  /// @brief Update the display of the controller's commands.
  /// @param trans_vel  the translation velocity sent to ROS,
  /// @param rot_vel    the rotation velocity sent to ROS.
  /// @since 0.3.1
  void updateCommands(const double& trans_vel, const double& rot_vel)
  { ctrl_wdgt->updateCommands(trans_vel, rot_vel);
    display.updateCommands(trans_vel, rot_vel); } 

}; // end of class QtCtrlGUI

#endif // QTCTRL_GUI
