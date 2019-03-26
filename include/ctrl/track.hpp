/**
 ** @file  include/ctrl/track.hpp
 **
 ** @brief Path following controller abstract class.
 **
 ** @date  October 2018
 **/


#ifndef QTCTRL_TRACK_CTRL
#define QTCTRL_TRACK_CTRL

#include <fstream>               // to get ifstream definition
#include <list> 
#include <model/state.hpp>
#include <ctrl/file.hpp>

/** 
 ** @brief TrackingCtrl aims at following a trajectory with the robot.
 **
 ** @since 0.3.1
 */
class TrackingCtrl : public FileCtrl {
protected:
  /// @brief The trajectory which should be followed.
  std::list<State*> traject;
  /// @brief The number of states in the followed trajectory.
  int          nb_states;
  /// @brief The trajectory's state which is aimed.
  std::list<State*>::const_iterator goal;
  /// @brief The current state of the robot.
  State state;

  /** @brief Moves the goal forward until it gets after the robot's 
   **        date plus a given amount of time.
   ** @param forwardTime  the amount of time forward.
   **/
  void searchGoal(const double& forwardTime = 0) {
    while ( ( goal != traject.end() )
	    && ( (*goal)->date() < state.date() + forwardTime ) )
      goal++;  // do not go over the last element:
    if ( goal == traject.end() )  goal--;
  } // end of void searchGoal(const double&)
   
public:
  /** @brief The constructor needs a motion model, a file name
   **        giving the path to track and an initial configuration.
   ** 
   ** States from the path are projected into the initial 
   ** configuration's frame.
   ** 
   ** @param model            the motion model,
   ** @param ts               the time step of the controller,
   ** @param input_file_name  the input file name of the path,
   ** @param init_config      the initial configuration.
   ** 
   ** @see FileCtrl(const MotionModel&, const double&, const char*), 
   **      Config::projection(const Config&), list::push_back
   **/
  TrackingCtrl(const MotionModel& model, const double& ts,
	       const char* input_file_name,
	       const Config& init_config);
  
  /// @brief The destructor needs to be explicitely redefined.
  virtual ~TrackingCtrl() {}

  // Cf Controller::newState(const State&)
  virtual void newState(const State& state)
  { this->state = state; Controller::newState(state); }
  
  /// @brief Gives the tracked trajectory.
  /// @return  the tracked trajectory.
  const std::list<State*>& trajectory() const { return traject; }

}; // end of class TrackingCtrl

#endif // QTCTRL_TRACK_CTRL
