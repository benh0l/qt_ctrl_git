/**
 ** @file  include/ctrl/file.hpp
 **
 ** @brief File controller (teleoperation) class.
 **
 ** @date  October 2018
 **/


#ifndef QTCTRL_FILE_CTRL
#define QTCTRL_FILE_CTRL

//#include <iostream>
#include <fstream>               // to get ifstream definition
#include <ctrl/controller.hpp>

/** 
 ** @brief FileCtrl allows to control the robot from a file.
 **
 ** Most of fields and methods were initially defined
 ** in @ref ROSnode "ROS node".
 **
 ** @since 0.3.0
 */
class FileCtrl : public Controller {
protected:
  std::ifstream *input;  ///< The input stream, if any.  @since 0.2.4

  /// @brief Is the input stream OK for reading?
  /// @return Yes if OK, no otherwise.  @since 0.2.4
  bool inputOK() const
  { return (input != NULL) && ( input->is_open() ); }

  /// @brief Is the input stream at its end?
  /// @return Yes if at the end, no otherwise.  @since 0.3.1
  bool inputEnded() const
  { return (input != NULL) && ( input->eof() ); }
    
  /// @brief Close and terminate the input stream.  @since 0.2.4
  void inputEnd() 
  { if ( inputOK() ) { input->close(); delete input; } }
  
public:
  /// @brief The constructor needs a motion model and 
  ///        an optional input file name.
  /// @param model            the motion model,
  /// @param ts               the time step of the controller,
  /// @param input_file_name  the input file name.
  /// @see Controller(const MotionModel&)
  FileCtrl(const MotionModel& model, const double& ts,
	   const char* input_file_name = NULL)
    : Controller(model, ts), input( input_file_name == NULL ? NULL :
				    new std::ifstream(input_file_name)
				    ){}
  
  /// @brief The destructor closes the input stream, if any.
  /// @since 0.2.4
  virtual ~FileCtrl() { inputEnd(); }
    
  /// @brief Change the input stream to a new file.
  /// @param input_file_name  The input file name.  @since 0.2.4
  void setInput(const char* input_file_name) {
    inputEnd();  // if the stream was open, close it...
    input = new std::ifstream(input_file_name); 
  }
  
  // Cf Controller::chooseVelocities(...)
  void chooseVelocities(double& trans_vel, double& rot_vel,
			std::ostringstream& log_str);
  
}; // end of class FileCtrl

#endif // QTCTRL_FILE_CTRL
