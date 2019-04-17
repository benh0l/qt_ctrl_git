/**
 * @file  src/ctrl/adrc.cpp
 *
 * @brief Path following controller class, copying the velocities.
 *
 * @date  November 2018
 **/


#include <ctrl/adrc.hpp>


// Cf Controller::chooseVelocities(...)
void ADRCCtrl::chooseVelocities(double& trans_vel, double& rot_vel,
				   std::ostringstream& log_str) {
  searchGoal();  // updates the goal to be after the robot's date
  // gets the goal's velocities
  //moving_velocity = (*goal)->translationVelocity(); 
  //turning_velocity = (*goal)->rotationVelocity(); 



  moving_velocity +=0.1; 
  turning_velocity += 0.1; 

double xi = state.configuration().position().xCoord();
double yi = state.configuration().position().yCoord();

double xiplus1 = (*goal)->configuration().position().xCoord();
double yiplus1 = (*goal)->configuration().position().yCoord();

double xiPrim = state.translationVelocity() * cos(state.rotationVelocity());
double yiPrim = state.translationVelocity() * sin(state.rotationVelocity());

double xiplus1Prim = (*goal)->translationVelocity() * cos((*goal)->rotationVelocity());
double yiplus1Prim = (*goal)->translationVelocity() * sin((*goal)->rotationVelocity());

double delta_t = (*goal)->date() - state.date();

double six = (xiplus1 - xi)/delta_t;
double siy = (yiplus1 - yi)/delta_t;

double cx0 = xi;
double cy0 = yi;

double cx1 = xiPrim;
double cy1 = yiPrim;


double cx2 = (3 * six - xiplus1Prim - 2*xiPrim)/delta_t;
double cy2 = (3 * siy - yiplus1Prim - 2*yiPrim)/delta_t;

double cx3 = (xiplus1Prim + xiPrim - 2 * six)/(delta_t * delta_t);
double cy3 = (yiplus1Prim + yiPrim - 2 * siy)/(delta_t * delta_t);



  //moving_velocity = 2.; 
  //turning_velocity = 3.; 
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void ImitateCtrl::chooseVelocities(double&, double&, ...)-
