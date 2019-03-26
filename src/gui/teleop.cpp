/**
 * @file /src/gui/teleop.cpp
 *
 * @brief Qt based teleoperation widget for <tt>qt_ctrl</tt>.
 *
 * @date July 2018
 **/

#include <gui/teleop.hpp> 
#include <QGridLayout>
#include <QToolButton>
#include <QIcon>
#include <QPainter>


/* The constructor need the main window containing this widget, 
 * and main()'s arguments (they can be forwarded by the ROS node 
 *  to ROS).
 *
 * Parameters: parent        The containing graphical object,
 *             control       the controller whose behaviour is shown,
 *             motion_model  the motion model.
 */
TeleopWidget::TeleopWidget(QWidget& parent, KbdCtrl& control,
			   const MotionModel& motion_model)
  : ControlWidget(control, motion_model), QGroupBox(&parent), 
    vel_drawing(*this, control, motion_model), mapper(this) {
  setTitle( tr("Teleoperation") );
  // numbers of buttons added, number of direction buttons,
  // number of shortcuts for each button
  static const int nb_buttons = 7, nb_dir_btns = 4, nb_shortcuts = 2;
  // array of the shortcuts
  static const Qt::Key shortcuts[nb_buttons][nb_shortcuts] =
    { {Qt::Key_8, Qt::Key_Up},    {Qt::Key_Period, Qt::Key_PageDown},
      {Qt::Key_2, Qt::Key_Down},  {Qt::Key_0, Qt::Key_End},
      {Qt::Key_4, Qt::Key_Left},  {Qt::Key_5, Qt::Key_Home},
      {Qt::Key_6, Qt::Key_Right} };
  // array of the tooltips
  static const char *tips[nb_buttons] =
    { "Accelerate motion", "Stop motion", "Decelerate motion", 
      "Stop all", "Accelerate rotation toward left", 
      "Stop rotation", "Accelerate rotation toward right" };
  // array of the Qt arrow types for the direction buttons
  static const Qt::ArrowType arrows[nb_dir_btns] =
    { Qt::UpArrow, Qt::DownArrow, Qt::LeftArrow, Qt::RightArrow };
  // base file name for the other buttons
  static const QString file(":/files/%1.svg");
  // array of the icons' name for the other buttons
  static const char *names[] = {"pause", "stop", "eject"};
  // velocities' drawing's size
  static const int disp_size = 3;
  // setting up the widget
  setMinimumSize(200, 200);
  // create a layout and set it to this object
  QGridLayout *layout = new QGridLayout(this);
  // for each button
  for(int id = 0; id < nb_buttons; id++) {
    QToolButton *button = new QToolButton(this);  // define it
    if (id % 2 == 0)  // if it is a direction button
      button->setArrowType(arrows[id / 2]);  // set the Qt arrow type
    else  // other buttons have an icon 
      button->setIcon( QIcon( file.arg(names[id / 2]) ) );
    // add it to the widget at the right position
    layout->addWidget(button, id < disp_size ? id : disp_size,
		      id < disp_size ? 0 : id - disp_size);
    // for each shortcut
    for(int short_id = 0; short_id < nb_shortcuts; short_id++)
      button->setShortcut(shortcuts[id][short_id]);  // set it
    // add tooltip
    button->setToolTip(tips[id]);
    // map the button to its key
    mapper.setMapping(button, shortcuts[id][0]);
    connect( button, SIGNAL( pressed() ), &mapper, SLOT( map() ) );
    // releasing the button stops the acceleration
    connect( button, SIGNAL( released() ), this, SLOT( stopAcc() ) );
  } // end of for (each button)
  // connect the mapper action to the widget's react method
  connect( &mapper, SIGNAL( mapped(const int&) ), 
	   this, SLOT( react(const int&) ) );
  // add the velocities' drawing
  layout->addWidget(&( vel_drawing.widget() ),
		    0, 1, disp_size, disp_size);
} // end of TeleopWidget::TeleopWidget(QWidget&, KbdCtrl&, ...) ------

/* This widget reacts to commands, which are Qt key values.
 *
 * The commands are currently fixed, there will be later a dialog 
 * to change the values associated to each action. These commands 
 * modify the ROS node velocities (with ROS node acceleration,
 * stop or stop rotation methods). 
 * The robot can be stopped (zero velocities) by pressing either 
 * '0' or 'End'. 
 * The robot can otherwise change its velocity in two ways:
 * +---+---+---+        +---+-----+---+
 * | 7 | 8 | 9 |        |   |  ↑  |   |
 * +---+---+---+        +---+-----+---+
 * | 4 | 5 | 6 |   or   | ← | Home| → |
 * +---+---+---+        +---+-----+---+
 * | 1 | 2 | 3 |        |   |  ↓  |   |
 * +---+---+---+        +---+-----+---+
 *
 * Directions are in the robot's frame, up meaning forward 
 * (acceleration) while down means backward (deceleration). 
 * Center ('5' or 'Home') stops the robot's rotation, while
 * '.' or 'Pg Down' stops the robot's translation.
 *
 * Parameters key  The command, one of Qt key values.
 *
 * See QKeyEvent. 
 */
void TeleopWidget::react(const int& key) {
  KbdCtrl& kbd_ctrl = (KbdCtrl&)(ctrl);
  switch (key) { 
  case Qt::Key_0 : case Qt::Key_End :  kbd_ctrl.stopMotion(); break; 
  case Qt::Key_1 :                       kbd_ctrl.acc(-1,1);  break; 
  case Qt::Key_2 : case Qt::Key_Down  :  kbd_ctrl.acc(-1,0);  break;  
  case Qt::Key_3 :                       kbd_ctrl.acc(-1,-1); break;
  case Qt::Key_4 : case Qt::Key_Left  :  kbd_ctrl.acc(0,1);   break; 
  case Qt::Key_5: case Qt::Key_Home: kbd_ctrl.stopRotation(); break; 
  case Qt::Key_6 : case Qt::Key_Right :  kbd_ctrl.acc(0,-1);  break; 
  case Qt::Key_7 :                       kbd_ctrl.acc(1,1);   break; 
  case Qt::Key_8 : case Qt::Key_Up    :  kbd_ctrl.acc(1,0);   break; 
  case Qt::Key_9 :                       kbd_ctrl.acc(1,-1);  break;
  case Qt::Key_Period: case Qt::Key_PageDown:
    kbd_ctrl.stopTranslation();
  }
} // end of void TeleopWidget::react(const int&) ---------------------
