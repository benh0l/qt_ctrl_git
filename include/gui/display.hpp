/**
 * @file /include/gui/display.hpp
 *
 * @brief Qt widget displaying data for <tt>qt_ctrl</tt>.
 *
 * @date August 2018
 **/

#ifndef QTCTRL_DISPLAY
#define QTCTRL_DISPLAY

#include <QWidget>
#include <QLabel>
#include <QCloseEvent>
#include <model/ROS_node.hpp>
#include <gui/overview.hpp>


/** @brief This class is a @ref refs_qt_widget "Qt widget" which 
 **        displays the data (odometry and commands) 
 **        of the @ref Controller "controller".
 **
 ** @since 0.2.2
 **/
class DataWidget : public QWidget {
  Q_OBJECT  // This macro is needed to handle graphical events

  QPointF last_vel;       ///< The last command published. @since 0.2.3
  std::ofstream *output;  ///< The output stream, if any. @since 0.2.3
  
  /// @brief These are the indexes
  ///        of @ref labels "the array of labels".
  enum Labels {  
    tOdom,   ///< Index of the odometry's time.
    xOdom,   ///< Index of the odometry's position abscissa.
    yOdom,   ///< Index of the odometry's position ordinate.
    thOdom,  ///< Index of the odometry's orientation.
    vOdom,   ///< Index of the odometry's translation velocity.
    omOdom,  ///< Index of the odometry's rotation velocity.
    vCmd,    ///< Index of the commands' translation velocity.
    omCmd,   ///< Index of the commands' rotation velocity.
    nbLabels,                     ///< Number of labels in the array.
    nbOdoms = vCmd,               ///< Number of odometry indexes.
    nbCmds  = nbLabels - nbOdoms  ///< Number of commands indexes.
  };

  /// @brief Labels shown before the values, and headers 
  ///        in the ouput file.  @since 0.2.3
  static const char *names[];
    
  /// @brief Array of labels displaying @ref Controller "controller"'s
  ///        data.
  QLabel labels[nbLabels];

  /// @brief Window showing the motion's path and velocities.
  /// @since 0.3.1
  QtMotionGUI motion_GUI;

  /** @brief Transfer data into the labels.
   ** 
   ** This is the common part of updateOdometry and updateCommands.
   **
   ** @param offset  The initial index in the label array,
   ** @param nbData  the number of data to transfer,
   ** @param data    the array of data to display.
   **
   ** @see labels.
   **/
  void setLabels(const Labels& offset, const Labels& nbData,
		 const double *data);
  
  /// @brief Is the output stream OK for writting?
  /// @return Yes if OK, no otherwise.  @since 0.2.3
  bool outputOK() const
  { return (output != NULL) && ( output->is_open() ); }

  /// @brief Close and terminate the output stream.  @since 0.2.3
  void outputEnd() 
  { if ( outputOK() ) { output->close(); delete output; } }

public:
  /** @brief The constructor needs the ROS node whose data will be 
   ** displayed.
   **
   ** @param parent  The windget containing this widget.
   **/
  DataWidget(QWidget& parent);
  
  /// @brief The destructor closes the output stream, if any.
  /// @since 0.2.3
  ~DataWidget() { outputEnd(); }
  
  /// @brief Change the output stream to a new file.
  /// @param output_file_name  The output file name.  @since 0.2.3
  void setOutput(const char* output_file_name) {
    outputEnd();  // if the stream was open, close it...
    output = new std::ofstream(output_file_name); 
    if ( outputOK() )  // Write header line in the ouput stream
      for(int idx = 0; idx < nbLabels; idx++)
	*output << names[idx] << (idx < nbLabels - 1 ? '\t' : '\n');
  }
  
  /// @brief Update the display of the state.
  /// @param state  The state of the robot.  @since 0.3.1
  void updateState(const State& state);
  
  /// @brief Update the display of the controller's commands.
  /// @param trans_vel  the translation velocity sent to ROS,
  /// @param rot_vel    the rotation velocity sent to ROS.
  /// @since 0.3.1
  void updateCommands(const double& trans_vel, const double& rot_vel);
  
  /// @brief Adds a trajectory to the motion display.
  /// @param trajectory  the trajectory to track.
  /// @see motion_GUI, QtMotionGUI::addTrajectory  @since 0.3.1
  void addTrajectory(const std::list<State*>& trajectory)
  { motion_GUI.addTrajectory(trajectory); }
  
}; // end of class DataWidget

#endif // QTCTRL_DISPLAY
