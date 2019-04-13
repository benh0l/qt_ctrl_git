void QtMotionGUI::dif_deriv ( int nd, double xd[], double yd[], int *ndp, double xdp[], 
  double ydp[] );
void QtMotionGUI::dif_shift_x ( int nd, double xd[], double yd[], double xv );
void QtMotionGUI::dif_shift_zero ( int nd, double xd[], double yd[] );
void QtMotionGUI::dif_to_r8poly ( int nd, double xd[], double yd[], double c[] );
double *QtMotionGUI::dif_vals ( int nd, double xd[], double yd[], int nv, double xv[] );
double QtMotionGUI::hermite_basis_0 ( int n, double x[], int i, double xv );
double QtMotionGUI::hermite_basis_1 ( int n, double x[], int i, double xv );
void QtMotionGUI::hermite_demo ( int n, double x[], double y[], double yp[] );
void QtMotionGUI::hermite_interpolant ( int n, double x[], double y[], double yp[], 
  double xd[], double yd[], double xdp[], double ydp[] );
double *QtMotionGUI::hermite_interpolant_rule ( int n, double a, double b, double x[] );
void QtMotionGUI::hermite_interpolant_value ( int nd, double xd[], double yd[], double xdp[], 
  double ydp[], int nv, double xv[], double yv[], double yvp[] );
double QtMotionGUI::r8_abs ( double x );
double QtMotionGUI::r8_max ( double x, double y );
double QtMotionGUI::r8poly_ant_val ( int n, double poly_cof[], double xval );
int QtMotionGUI::r8poly_degree ( int na, double a[] );
void QtMotionGUI::r8poly_print ( int n, double a[], std::string title );
double *QtMotionGUI::r8vec_chebyshev_new ( int n, double a_first, double a_last );
double *QtMotionGUI::r8vec_linspace_new ( int n, double a_first, double a_last );
void QtMotionGUI::r8vec_print ( int n, double a[], std::string title );
double QtMotionGUI::r8vec_product ( int n, double a[] );
void QtMotionGUI::r8vec_uniform_01 ( int n, int *seed, double r[] );
void QtMotionGUI::timestamp ( );
