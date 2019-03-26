/**
 * @file /src/gui/viewCtrl.cpp
 *
 * @brief Qt control widget showing current and aimed velocities.
 *
 * @date October 2018
 **/

#include <gui/viewCtrl.hpp> 
#include <QPainter> 
#include <QPaintEvent>


// Returns the graphical point corresponding to a couple of velocities. 
// Parameters  vel  The velocities, a couple of real values.
// Returns     The graphical point corresponding to the velocities.
QPoint ViewCtrlWdgt::plot(const QPointF& vel) const {
  const double &v = vel.x(), &om = vel.y(),
    &minV = model.minTranslVel(),
    &maxV = model.maxTranslVel(), &maxOm = model.maxRotVel();
  const QSize &sz = size();
  // both axes are reversed: left rotation is positive but is
  // on the left side of the drawing which is negative, and front is
  // on top which has low pixel ordinate values
  return QPoint( (int)( (1 - om / maxOm) / 2 * sz.width() ),
		 (int)( (maxV - v) / (maxV - minV) * sz.height() ) );
} // end of QPoint ViewCtrlWdgt::plot(const QPointF&) const ----------

// Draws the ROS node's velocities and their evolution. 
// Parameters event  The drawing event, giving the region
//                   which should be redrawn.
// See  vel_drawing  Since 0.2.2.
void ViewCtrlWdgt::paintEvent(QPaintEvent *event) {
  static const int nb_points = 2;
  static const Qt::GlobalColor colors[nb_points]
    = {Qt::green, Qt::blue};
  QPainter     painter(this); 
  const QRect& drawing_rect = event->rect();
  const QSize& sz      = size();
  const QPoint origin  = plot( QPointF() );
  const QPointF *points[nb_points] = {&velocities, &commands};
  const double &xOrig  = origin.x(), &yOrig = origin.y();
  
  painter.eraseRect(drawing_rect);  // clean the region
  // draw the axes
  painter.drawLine( xOrig, 0, xOrig, sz.height() );
  painter.drawLine( 0, yOrig, sz.width(), yOrig  );
  // draw the velocities (current = odometry and aimed = command)
  for(int i = 0; i < nb_points; i++) {
    painter.setPen(colors[i]);         // set the border color
    painter.setBrush(colors[i]);       // set the fill color
    painter.drawPie(region(*points[i]), 0, 360 * 16);  // fill the disk
  } // end of for (each point = velocities)
} // end of void ViewCtrlWdgt::paintEvent(QPaintEvent*) --------------

/* The constructor need the main window containing this widget, 
 * and main()'s arguments (they can be forwarded by the ROS node 
 *  to ROS).
 *
 * Parameters: parent        The containing graphical object,
 *             motion_model  the motion model,
 *             control       the controller whose behaviour is shown.
 *
 * See QWidget. 
 */
ViewCtrlWdgt::ViewCtrlWdgt(QWidget& parent, Controller& control,
			   const MotionModel& motion_model)
  : ControlWidget(control, motion_model), QWidget(&parent) {
  // setting up the widget's background color to white
  setBackgroundRole(QPalette::Base); 
  // setting up the widget's minimum size
  setMinimumSize(200, 200);
} // end of ViewCtrlWdgt::ViewCtrlWdgt(QWidget&, Controller&, ...) ---
