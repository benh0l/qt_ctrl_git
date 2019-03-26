/**
 ** @file  src/ctrl/track.cpp
 **
 ** @brief Path following controller abstract class.
 **
 ** @date  October 2018
 **/


#include <ctrl/track.hpp>

/* The constructor needs a motion model, a file name giving the path 
 * to track and an initial configuration.
 * 
 * States from the path are projected into the initial 
 * configuration's frame..
 * 
 * Parameter model            the motion model,
 *           ts               the time step of the controller,
 *           input_file_name  the input file name,
 *           init_config      the initial configuration.
 * 
 * See FileCtrl(const MotionModel&, const char*), 
 *     Config::projection(const Config&), list::push_back
 */
TrackingCtrl::TrackingCtrl(const MotionModel& model,
			   const double& ts,
			   const char* input_file_name,
			   const Config& init_config)
  : FileCtrl(model, ts, input_file_name), nb_states(0),
    goal(NULL), state() {
  if ( inputOK() ) {
    double t, x, y, th, v, om; 
    std::string endLine;
    std::getline(*input, endLine);  // skip first line
    do {
      *input >> t >> x >> y >> th >> v >> om;
      if ( !inputEnded() ) {
	Config q = init_config.projection( Config(Point(x, y), th) );
	traject.push_back( new State(t, q, v, om) ); 
	nb_states++;
	std::getline(*input, endLine);  // skip the rest of the line
      } // end of if (input not ended)
    } while ( !inputEnded() );
    goal = traject.begin();
  }  // end of if (input OK)
} // end of TrackingCtrl::TrackingCtrl(const MotionModel&, ...) ------
  
