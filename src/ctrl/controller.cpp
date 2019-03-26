/**
 * @file  src/ctrl/controller.cpp
 *
 * @brief Controller abstract class.
 *
 * @date  October 2018
 **/

#include <ctrl/controller.hpp>
#include <QPointF>

/* Handle new state of the robot.
 * Parameter state  the new state of the robot, from odometry.
 * This method can be overwritten by inheritors.
 */
void Controller::newState(const State& state) {
  static State last_state(state); 
  // new update: moved of 1 cm or .1 second elapsed
  if ( (state.configuration().position()
	.sqrDist( last_state.configuration().position() ) > 0.0001)
       || (state.date() - last_state.date() > .1) ) { 
    // signal the GUI for state display (replace logging)
    stateUpdated(state);  // send for display
    last_state = state; 
  } // end of if (new update)
} // end of void Controller::newOdometry(const Odometry::ConstPtr&) --
