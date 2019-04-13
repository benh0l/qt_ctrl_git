/**
 * @file /include/gui/overview.hpp
 *
 * @brief Qt based overview of the robot's motion.
 *
 * @date November 2018
 **/

#ifndef QTCTRL_GUI_MOTION
#define QTCTRL_GUI_MOTION

#include <QTabWidget>
#include <QGraphicsScene>
#include <list> 
#include <model/state.hpp>

//#include <gui/hermite2.hpp>
//#include <gui/hermite2.cpp>

/** @brief This class shows the motion of the robot.
 **
 ** It uses a tab window, one tab displaying the motion's path and
 ** the other the velocity profiles.
 **
 ** @since 0.3.1
 **/
class QtMotionGUI : public QObject {
  Q_OBJECT  // This macro is needed to handle graphical events

  QTabWidget motion_window;  ///< The window containing the displays.
  QGraphicsScene  path_scene;     ///< The scene containing the path.
  /// @brief The scene containing the translation velocity profile.
  QGraphicsScene  trans_vel_scene; 
  /// @brief The scene containing the rotation velocity profile.
  QGraphicsScene  rot_vel_scene;
  
  bool not_first_state;
  State last_state;

  bool average_not_first_state;
  //State average_last_state;
  int trajectory_nb_pts;
  int average_nb_pts;
  int average_delay_count;
  float average_lastX, average_lastY;

  //std::list<State*> trajectory_states;
  std::list<State*>::const_iterator trajectory_iterator;

public:
void dif_deriv ( int nd, double xd[], double yd[], int *ndp, double xdp[], 
  double ydp[] );
void dif_shift_x ( int nd, double xd[], double yd[], double xv );
void dif_shift_zero ( int nd, double xd[], double yd[] );
void dif_to_r8poly ( int nd, double xd[], double yd[], double c[] );
double *dif_vals ( int nd, double xd[], double yd[], int nv, double xv[] );
double hermite_basis_0 ( int n, double x[], int i, double xv );
double hermite_basis_1 ( int n, double x[], int i, double xv );
void hermite_demo ( int n, double x[], double y[], double yp[] );
void hermite_interpolant ( int n, double x[], double y[], double yp[], 
  double xd[], double yd[], double xdp[], double ydp[] );
double *hermite_interpolant_rule ( int n, double a, double b, double x[] );
void hermite_interpolant_value ( int nd, double xd[], double yd[], double xdp[], 
  double ydp[], int nv, double xv[], double yv[], double yvp[] );
double r8_abs ( double x );
double r8_max ( double x, double y );
double r8poly_ant_val ( int n, double poly_cof[], double xval );
int r8poly_degree ( int na, double a[] );
void r8poly_print ( int n, double a[], std::string title );
double *r8vec_chebyshev_new ( int n, double a_first, double a_last );
double *r8vec_linspace_new ( int n, double a_first, double a_last );
void r8vec_print ( int n, double a[], std::string title );
double r8vec_product ( int n, double a[] );
void r8vec_uniform_01 ( int n, int *seed, double r[] );
void timestamp ( );

  QtMotionGUI();

  void addTrajectory(const std::list<State*>& trajectory); 

  void addState(const State& state,
		const Qt::GlobalColor motion_color = Qt::blue);


  void addStateAverage(const State& state,
		const Qt::GlobalColor motion_color = Qt::red);

  double lagrange(const int n, const double* X,const double*  Y, double x);




public Q_SLOTS:
  void showHide()
  { motion_window.setVisible( !motion_window.isVisible() ); } 
  
}; // end of class QtMotionGUI

#endif // QTCTRL_GUI_MOTION
