/**
 * @file  src/model/ROS_node.cpp
 *
 * @brief ROS communication central!
 *
 * @date  July 2018
 **/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // Publication format
#include <sstream>
#include <model/ROS_node.hpp> 


// Stops ROS if it has been started.
// See ROS Namespace, QThread.
void ROSnode::end() {
  if ( connected() ) {
    motion_ctrl->stopMotion();
    if ( ros::isStarted() ) {
      ros::shutdown();         // stops ROS, and thus the run() method
      ros::waitForShutdown();  // waits for ROS to stop
    }
    wait();  // waits for the run() method (other thread) to stop
    motion_ctrl = NULL;
  } // end of if (connected)
} // end of ROSnode::~ROSnode() --------------------------------------

/* Common part of the two initialisation methods
 * Parameter: ctrl        the motion controller.
 * See ROS Namespace, ROS::Publisher, QThread, 
 *     init(), init(const std::string&, const std::string&). 
 */
void ROSnode::ROSsetup(Controller& ctrl) {
  if ( ros::master::check() ) {
    motion_ctrl = &ctrl;
    ros::NodeHandle nh;
    // explicitly needed since our nodehandle is going out of scope
    ros::start();
    // set the publisher used to send the commands (velocities)
    cmd_publisher = nh.advertise<geometry_msgs::Twist>
      ("cmd_vel_mux/input/teleop", 10);
    // subscrive to the odometry topic, 
    // and start new_odom method with each new data
    odom_subscriber = nh.subscribe("/odom", 1000,
				   &ROSnode::newOdometry, this);
    start();  // starts the process, and calls run()
  } // end of if (is_connected)
} // end of void ROSnode::ROSsetup() ---------------------------------

// Sends the odometry data to the controller.
// Parameter: odom  the new odometry data.
// See Controller::newState().
void ROSnode::newOdometry(const nav_msgs::Odometry::ConstPtr& odom) {
  const ros::Time& t = odom->header.stamp; 
  const double date  = t.sec + 1E-9 * t.nsec;
  static const double initial_date = date;
  const geometry_msgs::Pose&  pose = odom->pose.pose;
  const geometry_msgs::Point& point = pose.position; 
  const geometry_msgs::Quaternion& orient = pose.orientation;
  const tf::Quaternion tfQuat(orient.x, orient.y, orient.z, orient.w);
  const tf::Matrix3x3  tfMatrix(tfQuat);
  const geometry_msgs::Twist& twist = odom->twist.twist; 
  double roll, pitch, yaw;
  tfMatrix.getRPY(roll, pitch, yaw);
  const State
    state(date - initial_date, Config(Point(point.x, point.y), yaw),
	  twist.linear.x, twist.angular.z);
  motion_ctrl->newState(state); 
} // end of void ROSnode::newOdometry(const nav_msgs:...:ConstPtr&) --

/* Main loop of the process (in a separate thread).
 *
 * See QThread, ROS::Rate, stop(), geometry_msgs::Twist, 
 *     ROS Namespace, log(const LogLevel&, const std::string&), 
 *     ROS::Publisher, ROSshutdown(). 
 */
void ROSnode::run() {  
  if ( connected() ) {
    const double loop_frequ = 1 / motion_ctrl->timeStep();
    // defined from input stream, if any
    ros::Rate loop_rate(loop_frequ);
    geometry_msgs::Twist vel;
  
    motion_ctrl->stopMotion();  // Sets the velocities to zero.
    while ( ros::ok() ) {
      std::ostringstream log_msg;
      motion_ctrl->chooseVelocities(vel.linear.x, vel.angular.z,
				    log_msg);
      log( Info, log_msg.str() ); 
      cmd_publisher.publish(vel);  // publish the commands
      // spin ROS once, and wait for next time step
      ros::spinOnce();
      loop_rate.sleep();
    } // end of while (ROS OK)
  } // end of if (connected)
} // end of void ROSnode::run() --------------------------------------

/* Connects to ROS server with parameters.
 *
 * Parameters: master_url  the URI of the ROS master,
 *             host_url    the URI of the host,
 *             ctrl        the motion controller.
 *
 * See ROS Namespace, ROSsetup().
 */
void ROSnode::init(const std::string &master_url,
		   const std::string &host_url, Controller& ctrl) {
  std::map<std::string,std::string> remappings;
  remappings["__master"]   = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"qt_ctrl");
  ROSsetup(ctrl);
} // end of void ROSnode::init(const std::string&, ..., Controller&) -

/* Writes a message at given level in the log.
 *
 * Parameters: level  the log's severity level,
 *             msg    the message.
 *
 * See ROS::console, ROS::Time, QStringListModel,
 *     connected(), loggingModel(), loggingUpdated(). 
 */
void ROSnode::log(const LogLevel &level, const std::string &msg) {
  static const char* levelName[Count] =
    {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
  std::stringstream log_model_msg;

  if ( connected() && (level < Count) && ( !msg.empty() ) ) {
    // fills ROS log at the requested level
    ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, msg);
    // change the GUI log: insert a line in the GUI log
    log_model.insertRows(log_model.rowCount(),1);
    // write the formated (add level and date) message 
    log_model_msg << "[" << levelName[(int)(level)] << "] ["
		  << ros::Time::now() << "]: " << msg;
    // transform the string stream into a QVariant (= union)
    QVariant new_row( QString( log_model_msg.str().c_str() ) );
    // add it as the last line of the log
    log_model.setData(log_model.index(log_model.rowCount()-1), new_row);
    Q_EMIT loggingUpdated(); // ask the GUI to show its log
  } // end of if (connected, valid level & message non empty)
} // end of void ROSnode::log(const LogLevel&, const std::string &) --
