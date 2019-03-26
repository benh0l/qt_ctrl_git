/**
 ** @file  include/model/ROS_node.hpp
 **
 ** @brief ROS communication node.
 **
 ** @date  July 2018
 **/

#ifndef QTCTRL_ROS_NODE
#define QTCTRL_ROS_NODE

#include <string>
#include <fstream>
#include <QThread>
#include <QStringListModel>
#include <ctrl/controller.hpp>
// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <tf/tf.h>
//#include <boost/python.hpp>
#endif
#include <nav_msgs/Odometry.h>     // odometry


using namespace ::ros::console::levels;

/**
 ** @brief ROS node, getting sensors data and sending commands.
 **
 ** This class is the one in charge of the communication with ROS.
 ** It only sends commands through a publisher, for the moment.
 ** It is a @ref refs_qt_thread, i.e. a process which runs 
 ** its @ref run "main loop" in a separate thread.
 ** This class is a slight modification of the one generated 
 ** by <tt><a href="http://wiki.ros.org/qt_create"
 ** >catkin_create_qt_pkg</a></tt>.
 **
 ** @since 0.2.0
 */
class ROSnode : public QThread {
  Q_OBJECT  // This macro is needed to handle signals

  int    ROS_argc;  ///< Main arguments' count, forwarded to ROS.
  char** ROS_argv;  ///< Main arguments' values, forwarded to ROS.

  /** @brief A pointer to the controller of the motion. 
   **
   ** A NULL value indicates the node is not connected to ROS.
   **
   ** @since 0.3.0
   **/
  Controller* motion_ctrl;
    
  /** @brief ROS object used to send the velocities.
   **
   ** See @ref refs_ros_publisher.
   **/
  ros::Publisher   cmd_publisher;
  
  /** @brief ROS object used to get the odometry.
   **
   ** While this object is not used outside of the constructor, 
   ** it has to remain defined as long as the ROS node exists, 
   ** as it is in charge of transmitting the odometry's data 
   ** to @ref Controller::newState "the method which handle them".
   **
   ** See @ref refs_ros_subscriber.
   **/
  ros::Subscriber   odom_subscriber;
  
  /** @brief Qt object used to show the logs 
   **        in the @ref QtCtrlGUI "GUI".
   **
   ** See  @ref refs_qt_stringlistmodel.
   **/
  QStringListModel log_model;

  /** @brief Common part of the two initialisation methods.
   ** @param ctrl  the motion controller.
   ** See  @ref refs_ros_ros, @ref refs_ros_publisher, 
   **      @ref refs_qt_thread.
   ** @see init(), init(const std::string&, const std::string&).
   **/
  void ROSsetup(Controller& ctrl);

  /// @brief Sends the odometry data to the controller.
  /// @param odom  the new odometry data.
  /// @see Controller::newState().
  void newOdometry(const nav_msgs::Odometry::ConstPtr& odom);

protected:
  /** @brief  Main loop of the process (on a separate thread).
   **
   ** See @ref refs_qt_thread, @ref refs_ros_rate, stop(), 
   **     @ref refs_ros_twist, @ref refs_ros_ros, 
   **     log(const LogLevel&, const std::string&), 
   **     @ref refs_ros_publisher, ROSshutdown().
   **/
  void run();
  
public:
  /// @brief Log severity levels, comming from @ref refs_ros_console.
  /// @since 0.2.1
  typedef ::ros::console::Level LogLevel; 

  /** @brief The constructor requires <tt>main()</tt>'s arguments 
   **        (they can be forwarded to ROS).
   **
   ** @param argc  the arguments' count,
   ** @param argv  the arguments' value.
   **
   ** @see init().
   **/
  ROSnode(const int argc, char** argv) 
    : ROS_argc(argc), ROS_argv(argv), motion_ctrl(NULL) {}
  
  /** @brief Stops ROS if it has been started.
   **
   ** See  @ref refs_ros_ros, @ref refs_qt_thread.
   **/
  void end();

  /** @brief   Indicates whether the node is connected to ROS.
   ** @return  True if connected to ROS, false otherwise.
   ** @since 0.2.1
   **/
  bool connected() const { return (motion_ctrl != NULL); } 
  
  /** @brief  Connects to ROS server with <tt>main</tt> arguments.
   ** @param  ctrl  the motion controller.
   ** See @ref refs_ros_ros, ROSsetup().
   **/
  void init(Controller& ctrl)
  { ros::init(ROS_argc, ROS_argv, "qt_ctrl"); ROSsetup(ctrl); }
  
  /** @brief  Connects to ROS server with parameters.
   **
   ** @param master_url  the URI of the ROS master,
   ** @param host_url    the URI of the host,
   ** @param ctrl        the motion controller.
   ** 
   ** See @ref refs_ros_ros, ROSsetup().
   **/
  void init(const std::string &master_url,
	    const std::string &host_url, Controller& ctrl);

  /// @brief   The Qt object used to show the logs.
  /// @return  A reference to the @ref refs_qt_stringlistmodel 
  ///          "Qt object" used to show the logs.
  QStringListModel& loggingModel() { return log_model; }

  /** @brief  Writes a message at given level in the log.
   **
   ** @param level  the log's severity level,
   ** @param msg    the message.
   **
   ** See  @ref refs_ros_console, @ref refs_ros_time, 
   **      @ref refs_qt_stringlistmodel, connected(), 
   **      loggingModel(), loggingUpdated().
   **/
  void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:  // this requires the Q_OBJECT macro
  /// @brief   Signals the @ref QtCtrlGUI "GUI" for log's update.
  ///
  /// See @ref refs_qt_gui-thread.
  void loggingUpdated();

}; // end of class ROSnode

#endif // QTCTRL_ROS_NODE
