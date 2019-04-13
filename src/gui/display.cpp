/**
 * @file /src/gui/display.cpp
 *
 * @brief Qt widget displaying data for <tt>qt_ctrl</tt>.
 *
 * @date August 2018
 **/

#include <gui/display.hpp>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QPushButton>


// Labels shown before the values, and headers in the ouput file.
// Since 0.2.3
const char *DataWidget::names[] = 
  {"t", "x", "y", "theta", "v", "omega", "v", "omega"}; // -----------

/* The constructor needs the ROS node whose data will be displayed.
 *
 * Parameter parent  The main window containing this widget,
 *           node    the ROS node whose data will be displayed.
 */
DataWidget::DataWidget(QWidget& parent)
  : QWidget(&parent), output(NULL), motion_GUI() {
  // the widget is split vertically in two group boxes
  QVBoxLayout *layout = new QVBoxLayout(this);
  QGroupBox *odom_box = new QGroupBox( tr("Odometry"), this),
    *cmd_box = new QGroupBox( tr("Commands"), this), *a_box;
  layout->addWidget(odom_box);
  layout->addWidget(cmd_box);
  QGridLayout *odom_layout = new QGridLayout(odom_box),
    *cmd_layout = new QGridLayout(cmd_box), *a_layout;
  static const char *init_val = "----.--";
  int idx;  // for each label
  for(idx = 0, a_layout = odom_layout; idx < nbLabels; idx++) {
    // update the pointers when needed
    if (idx == nbOdoms)  a_layout = cmd_layout; 
    // Boxes contains lines of 3 columns with 2 labels
    const int nbCols = 3, dId = (idx == tOdom ? nbOdoms - 1 :
				 idx < nbOdoms ? idx - 1 : idx),
      // dId = display index, index are displayed with tOdom
      // as last odometry data
      line = dId / nbCols, col = (2 * dId) % (2 * nbCols);
    // Additions to the basic labels
    const bool isGreek =
      (idx == thOdom) || (idx == omOdom) || (idx == omCmd);
    char name[25];
    sprintf(name, "%s%s%s", isGreek ? "<span>&" : "<em>",
	    names[idx], isGreek ? ";</span>:" : "</em>:");
    a_layout->addWidget(new QLabel( tr(name) ), line, col);
    labels[idx].setText(init_val); 
    a_layout->addWidget(&(labels[idx]), line, col + 1);
  } // end of for (each label)
  QPushButton *motionBtn = new QPushButton("Historic");     // History
  layout->addWidget(motionBtn);       // button is added at the bottom
  connect( motionBtn, SIGNAL( clicked() ),         // and connected to
	   &motion_GUI, SLOT( showHide() ) ); // motion's GUI showHide
} // end of DataWidget::DataWidget(QtCtrlGUI&, ROSnode&) -------------

/* Transfer data into the labels.  See labels.
 * This is the common part of updateOdometry and updateCommands.
 *
 * Parameters offset  The initial index in the label array,
 *            nbData  the number of data to transfer,
 *            data    the array of data to display.
 */
void DataWidget::setLabels(const Labels& offset, const Labels& nbData,
			   const double *data) {
  static const QString val("%1");
  int idx;
  const double *data_prt  = data;
  QLabel *label_prt = &(labels[offset]);
  for(idx = 0; idx < nbData; idx++, data_prt++, label_prt++)
    // show the angle in degree, and rotation velocities in degree/s
    label_prt->setText
      ( (idx == thOdom) || (idx == omOdom) || (idx == omCmd) 
	? val.arg(Object::grad2deg(*data_prt), 0, 'f', 0)
	: val.arg(*data_prt, 0, 'f', 2) );
} // end of DataWidget::setLabels(const Labels&, ..., const double*) -

// Update the display of the odometry.
// Parameters odom  The odometry data.  Since 0.2.2.
void DataWidget::updateState(const State& state) {
  const Config& q = state.configuration();
  const Point&  P = q.position();
  const double data[] =
    {state.date(), P.xCoord(), P.yCoord(), q.orientation(), 
     state.translationVelocity(), state.rotationVelocity()};
  setLabels(tOdom, nbOdoms, data);
  motion_GUI.addState(state);
  if ( outputOK() ) {
    int idx;
    for(idx = 0; idx < nbOdoms; idx++)
      *output << data[idx] << '\t';
    *output << last_vel.x() << '\t' << last_vel.y() << std::endl;
  } // end of if (ouput OK)
} // end of void DataWidget::updateOdometry(const Odometry::ConstPtr&)

// Update the display of the controller's commands.
// Parameters trans_vel  the translation velocity sent to ROS,
//            rot_vel    the rotation velocity sent to ROS.
// Since 0.3.1
void DataWidget::updateCommands(const double& trans_vel,
				const double& rot_vel) {
  const double data[] = {trans_vel, rot_vel};
  last_vel.setX(data[0]); 
  last_vel.setY(data[1]); 
  setLabels(vCmd, nbCmds, data);
} // end of void DataWidget::updateCommands(const Twist&) ------------
