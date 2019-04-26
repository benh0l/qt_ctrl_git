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
  //searchGoal(0.05);  // updates the goal to be after the robot's date
searchGoal(0.2);  // updates the goal to be after the robot's date
  // gets the goal's velocities
  moving_velocity = (*goal)->translationVelocity(); 
  //turning_velocity = (*goal)->rotationVelocity(); 



  //moving_velocity +=0.1; 
 // turning_velocity += 0.1; 

double xi = state.configuration().position().xCoord();
double yi = state.configuration().position().yCoord();

double xiplus1 = (*goal)->configuration().position().xCoord();
double yiplus1 = (*goal)->configuration().position().yCoord();

double xiPrim = state.translationVelocity() * cos(state.rotationVelocity());
double yiPrim = state.translationVelocity() * sin(state.rotationVelocity());

double xiplus1Prim = (*goal)->translationVelocity() * cos((*goal)->rotationVelocity());
double yiplus1Prim = (*goal)->translationVelocity() * sin((*goal)->rotationVelocity());

double ti = state.date();
double tiplus1 = (*goal)->date();

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


double xd0 = cx3 * pow(0.05 * delta_t, 3) + cx2 * pow(0.05 * delta_t, 2) + cx1 * 0.05 + cx0;
double yd0 = cy3 * pow(0.05 * delta_t, 3) + cy2 * pow(0.05 * delta_t, 2) + cy1 * 0.05 + cy0;

double xd00 = cx3 * pow(0.1 * delta_t, 3) + cx2 * pow(0.1 * delta_t, 2) + cx1 * 0.1 + cx0;
double yd00 = cy3 * pow(0.1 * delta_t, 3) + cy2 * pow(0.1 * delta_t, 2) + cy1 * 0.1 + cy0;

//double xd1 = cx3 * pow(0.49 * delta_t, 3) + cx2 * pow(0.49 * delta_t, 2) + cx1 * 0.49 + cx0;
//double yd1 = cy3 * pow(0.49 * delta_t, 3) + cy2 * pow(0.49 * delta_t, 2) + cy1 * 0.49 + cy0;

//double xd2 = cx3 * pow(0.5 * delta_t, 3) + cx2 * pow(0.5 * delta_t, 2) + cx1 * 0.5 + cx0;
//double yd2 = cy3 * pow(0.5 * delta_t, 3) + cy2 * pow(0.5 * delta_t, 2) + cy1 * 0.5 + cy0;

//double xd3 = cx3 * pow(0.51 * delta_t, 3) + cx2 * pow(0.51 * delta_t, 2) + cx1 * 0.51 + cx0;
//double yd3 = cy3 * pow(0.51 * delta_t, 3) + cy2 * pow(0.51 * delta_t, 2) + cy1 * 0.51 + cy0;



double xd1 = cx3 * pow(0.09 * delta_t, 3) + cx2 * pow(0.09 * delta_t, 2) + cx1 * 0.09 + cx0;
double yd1 = cy3 * pow(0.09 * delta_t, 3) + cy2 * pow(0.09 * delta_t, 2) + cy1 * 0.09 + cy0;

double xd2 = cx3 * pow(0.1 * delta_t, 3) + cx2 * pow(0.1 * delta_t, 2) + cx1 * 0.1 + cx0;
double yd2 = cy3 * pow(0.1 * delta_t, 3) + cy2 * pow(0.1 * delta_t, 2) + cy1 * 0.1 + cy0;

double xd3 = cx3 * pow(0.11 * delta_t, 3) + cx2 * pow(0.11 * delta_t, 2) + cx1 * 0.11 + cx0;
double yd3 = cy3 * pow(0.11 * delta_t, 3) + cy2 * pow(0.11 * delta_t, 2) + cy1 * 0.11 + cy0;

double t049 = 0.49 + ti;


double xdPrim = cx3 * 3 * t049 * t049 
	- 3 * cx3 * ti * 2 * t049
	+ cx2 * 2 * t049
	+ 3*cx3 * ti * ti  
	- cx2 *2 * ti 
	+ cx1;

double theta1_2 =  atan((xd2-xd1) / (yd2-yd1));
double theta2_3 =   atan((xd3-xd2) / (yd3-yd2));

double theta00 =  atan((xd0-xi) / (yd0-yi));

double theta0 =  atan((xd00-xd0) / (yd00-yd0));

 //Que faire en cas de div par zero?

//double omega = (theta0 - theta00) / 0.05;
//double omega = (theta2_3 - theta1_2) / 0.01;
double omega = (theta2_3 - theta1_2) / 0.01;
//double omega = (theta1_2 - state.rotationVelocity()) / 0.5;
//double omega = (theta0 - state.rotationVelocity()) / 0.1;

std::cout << xdPrim  << " au lieu de "<< theta1_2  << "\n";
//std::cout << omega  << " au lieu de "<< state.rotationVelocity()  << "\n";

//std::cout <<  theta1_2 << " et "<<  theta2_3  << " contre "<< state.rotationVelocity()  << "\n";
//std::cout << state.rotationVelocity() << " puis "<< omega << " puis "<< (*goal)->rotationVelocity()<< "\n";

//std::cout << (*goal)->rotationVelocity()*100/(*goal)->translationVelocity() << "\n";


//std::cout <<  acos(((xd2 - xd1)/0.01)/(*goal)->translationVelocity()) << "  " << asin(((yd2 - yd1)/0.01)/(*goal)->translationVelocity())<< "  " <<(*goal)->translationVelocity() << "\n";

//std::cout << xi << " puis "<< xd1 << " puis "<< xiplus1<< "\n";

//double vitesse = sqrt(pow(xiplus1 - xi, 2)+ pow(yiplus1 - yi, 2));

//std::cout << delta_t << ": "<< vitesse << " au lieu de "<<(*goal)->translationVelocity()<< "\n";

//moving_velocity = vitesse;
turning_velocity = omega;
  //moving_velocity = 2.; 
  //turning_velocity = 3.; 
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void ImitateCtrl::chooseVelocities(double&, double&, ...)-
