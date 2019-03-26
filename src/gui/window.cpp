/**
 * @file /src/gui/window.cpp
 *
 * @brief Qt based GUI for qt_ctrl.
 *
 * @date July 2018
 **/

#include <ctrl/file.hpp>
#include <ctrl/imitate.hpp>
#include <ctrl/PID.hpp>
#include <ctrl/analytic.hpp>
#include <gui/teleop.hpp>
#include <gui/window.hpp>
#include <QDialog>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLineEdit>
#include <QDoubleValidator>
#include <QPushButton>
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QDialogButtonBox>

const double QtCtrlGUI::time_step = .2;

// Asks in a dialog for a set of doubles, and returns it.
// Parameter  nb_lines  the number of lines in the dialog,
//            nb_col    the number of columns in the dialog,
//            nameVal   the array of names displayed,
//            isGreek   the array indicating wether name is in Greek,
//            val       the array of doubles to get from the user.
// Precondition  the arrays need to have at least nb_lines x nb_col
//               elements, or a segmentation violation may arrise.
void QtCtrlGUI::getDoubleArray(const int nb_lines, const int nb_col,
							   const char* nameVal[],
							   const bool isGreek[], double val[]) {
	const int nbVal = nb_lines * nb_col;
	QDialog dialog(this);                     // the dialog
	dialog.setModal(true);
	QWidget values_wdgt(&dialog);             // the values widget
	QVBoxLayout dialog_layout(&dialog);       // dialog layout (vertic.)
	QGridLayout values_layout(&values_wdgt);  // values layout (grid)
	QLineEdit *valBox = new QLineEdit[nbVal];    // its value boxes
	QDoubleValidator doubleValid(&values_wdgt);  // they take doubles
	int idx;
	for(idx = 0; idx < nbVal; idx++) {  // fills the widget
		const int lg = idx / nb_col, col = 2 * (idx % nb_col);
		char name[25];  // text label: emphasize if not greek
		sprintf(name, "%s%s%s", isGreek[idx] ? "<span>" : "<em>",
				nameVal[idx], isGreek[idx] ? "</span>:" : "</em>:");
		values_layout.addWidget(new QLabel( tr(name) ), lg, col);
		valBox[idx].setText( QString("%L1").arg(val[idx]) );
		valBox[idx].setValidator(&doubleValid);  // accepts only double
		values_layout.addWidget(&valBox[idx], lg, col + 1);
	} // end of for (each value)
	dialog_layout.addWidget(&values_wdgt);  // add the values widget
	QPushButton ok_btn( tr("Ok") );  // OK button closes the dialog
	connect( &ok_btn, SIGNAL( clicked() ), &dialog, SLOT( close() ) );
	dialog_layout.addWidget(&ok_btn);  // adds it at the bottom
	dialog.exec();  // shows the dialog and waits until it closes
	for(idx = 0; idx < nbVal; idx++)             // extracts the values
		val[idx] = valBox[idx].text().toDouble();  // from the boxes
	delete[] valBox;
} // end of Config QtCtrlGUI::getConfig() const ----------------------

/* The constructor requires <tt>main()</tt>'s arguments (they can 
 *  be forwarded by the @ref ROSnode "ROS node" to ROS).
 *
 * Parameter argc    the arguments' count,
 *           argv    the arguments' value,
 *           parent  the main widget parent, if any (optional).
 *
 * See ROSnode::ROSnode().
 */
QtCtrlGUI::QtCtrlGUI(const int argc, char** argv, QWidget* parent)
		: QMainWindow(parent),   menu_bar(this),  app_menu(&menu_bar),
		  help_menu(&menu_bar), tool_bar(this), start_stop_action(this),
		  about_action(this), about_Qt_action(this), ROS_server(this),
		  motion_model(-.5, .5, M_PI/4, -.6, .5, -M_PI/8, M_PI/8),
		  controller( new NoCtrl(motion_model) ), ctrl_node(argc, argv),
		  ctrl_wdgt( new ViewCtrlWdgt(*this, *controller, motion_model) ),
		  display(*this), logs(this) { // setup the UI
	setWindowTitle( tr("ROS - Qt Controlers Benchmark") );
	setWindowIcon( QIcon(":/files/icon.png") );
	// set the Start, "About Qt Ctrl" and "About Qt" actions
	start_stop_action.setText( tr("Start Ctrl") );
	start_stop_action.setShortcut(Qt::Key_S);
	connect( &start_stop_action, SIGNAL( triggered() ),
			 this, SLOT( buttonStartStop() ) );
	about_action.setText( tr("About Qt &Ctrl") );
	connect( &about_action, SIGNAL( triggered() ),
			 this, SLOT( actionAbout() ) );
	about_Qt_action.setText( tr("About &Qt") );
	connect( &about_Qt_action, SIGNAL( triggered() ),
			 qApp, SLOT( aboutQt() ) );
	// set the menu bar and the menus
	setMenuBar(&menu_bar);
	app_menu.setTitle( tr("&App") );
	app_menu.addAction(&start_stop_action);
	menu_bar.addMenu(&app_menu);
	help_menu.setTitle( tr("&Help") );
	help_menu.addAction(&about_action);
	help_menu.addAction(&about_Qt_action);
	menu_bar.addMenu(&help_menu);
	// set the tool bar and add start button
	addToolBar(&tool_bar);
	tool_bar.addAction(&start_stop_action);
	// set the main widget, with teleoperation and logging
	QWidget *central_widget = new QWidget(this);
	setCentralWidget(central_widget);
	// create a layout for the central widget
	QGridLayout *central_layout = new QGridLayout(central_widget);
	central_layout->addWidget(&( ctrl_wdgt->widget() ), 0, 0);
	central_layout->addWidget(&display, 0, 1);
	QGroupBox *log_box = new QGroupBox(tr("Logs"), central_widget);
	QBoxLayout *log_layout =   // create a layout for the log box
			new QBoxLayout(QBoxLayout::LeftToRight, log_box);
	logs.setMinimumSize(400, 100);
	log_layout->addWidget(&logs);
	central_layout->addWidget(log_box, 1, 0, 1, 2);
	// restore the settings
	readSettings();
	// initialises the state's display
	ctrl_wdgt->updateState( State() );
	// idem for the commands
	ctrl_wdgt->updateCommands(0, 0);
	// connects log signal of the ROS node to the GUI method
	logs.setModel(&( ctrl_node.loggingModel() ) );
	connect( &(ctrl_node), SIGNAL( loggingUpdated() ),
			 this, SLOT( updateLogging() ) );
	// needed after Q_DECLARE_METATYPE and before the connects
	qRegisterMetaType<State>();
	connectCtrl(); // connects the controller signals to the GUI methods
} // end of QtCtrlGUI::QtCtrlGUI(int, char**, QWidget*) --------------

// Load up Qt program settings at startup.
void QtCtrlGUI::readSettings() {
	QSettings settings("Qt-Ros Package", "qt_ctrl");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("windowState").toByteArray());
	QString master_url = settings.value
			("master_url",QString("http://192.168.1.2:11311/")).toString();
	QString host_url = settings.value
			("host_url", QString("192.168.1.3")).toString(); /*
    //QString topic_name = settings.value
    //("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
	} */
} // end of void QtCtrlGUI::readSettings() ---------------------------

// Save Qt program settings when closing.
void QtCtrlGUI::writeSettings() {
	QSettings settings("Qt-Ros Package", "qt_ctrl"); /*
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant
		      (ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant
		      (ui.checkbox_remember_settings->isChecked()));
						     */
} // end of void QtCtrlGUI::writeSettings() --------------------------

// Method activated by the "About" menu item (not yet used).
void QtCtrlGUI::actionAbout() {
	QMessageBox::about
			(this, tr("About ROS-Qt Control GUI"),
			 tr("<h2>ROS-Qt Control GUI</h2>\
           <p>Copyright <a href=\"http://members.loria.fr/AScheuer\"\
	      >Alexis Scheuer</a>, <a href=                         \
              \"http://www.univ-lorraine.fr\">UL</a>|<a href=       \
              \"http://fst.univ-lorraine.fr\">FST</a> &amp; <a href=\
              \"http://www.loria.fr\">Loria</a></p>		    \
           <p>This package proposes several controllers for ROS     \
              with a Qt GUI, in order to compare them.</p>"));
} // end of void QtCtrlGUI::actionAbout() ----------------------------

/// @brief Method activated by the "Start/Stop" button.
void QtCtrlGUI::buttonStartStop() {
	bool not_connected, trying, tracking;
	start_stop_action.setEnabled(false);// prevent the button activation
	if ( ctrl_node.connected() ) {      // stop the control
		ctrl_node.end();                // stop ROS node (= server + ctrl)
		if ( ROS_server.isRunning() ) {
			const QMessageBox::StandardButton ans = QMessageBox::question
					(this, tr("Stopping ROS master?"),  // title of the dialog
					 tr("Should this GUI stop the TurtleBot simulation "
						"(you may not be able to start it again)?"),
							// 3 buttons, Cancel is the default
					 QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
			if (ans == QMessageBox::Yes)
				ROS_server.stop();  // stop ROS server's thread
		} // en of if (ROS server is running)
		start_stop_action.setText( tr("Start Ctrl") );
	} else {      // start the control
		do { /* ask, then connect */
			const QMessageBox::StandardButton ans = QMessageBox::question
					(this, tr("How to connect to ROS?"),  // title of the dialog
					 tr("Should this GUI start a TurtleBot simulation, as well "
						"as the control node? Choosing \"No\" means you have "
						"started manually ROS, letting the GUI launching the "
						"control node, while \"Cancel\" aborts both launch (ROS "
						"and control)."), // 3 buttons, Cancel is the default
					 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
					 QMessageBox::Cancel);
			trying = (ans != QMessageBox::Cancel);  // trying = not abort
			if (ans == QMessageBox::Yes) {  // ask which workspace to use
				const QMessageBox::StandardButton ans = QMessageBox::question
						(this, tr("Which workspace to use?"),  // title of the dialog
						 tr("Should the simulation use the empty workspace (or "
							"the default one)?"), // 3 buttons, Yes is the default
						 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
						 QMessageBox::Yes);
				trying = (ans != QMessageBox::Cancel);
				if (trying) {  // start ROS
					ROS_server.setEmptyWorkspace(ans == QMessageBox::Yes);
					ROS_server.start();
					// previous call returns immediatly, but needs...
					ROS_server.sleep(5);  // ... about 5 s to be effective
				} // end of if (start ROS)
			} // end of if (ask which workspace to use)
			if (trying) { // ask for controller's category
				const QMessageBox::StandardButton ans = QMessageBox::question
						(this, tr("Which category of controller to use?"),  // title
						 tr("Should the robot track a trajectory "
							"(or generate one)?"), // 3 buttons, Yes is the default
						 QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel,
						 QMessageBox::Yes);
				trying = (ans != QMessageBox::Cancel);
				tracking = (ans == QMessageBox::Yes);
				if (trying) { // ask for trajectory or control file
					char inputType[20];
					sprintf(inputType, "%s File", tracking ? "Trajectory"
														   : "Accelerations");
					QString fileType = tr("CSV File (*.csv *.txt)"),
							inputFile = QFileDialog::getOpenFileName
							(this, tr(inputType), "", fileType),
							saveFile = QFileDialog::getSaveFileName
							(this, tr("Motion Output File"), "", fileType);
					// memorize old pointers before reallocation (free later)
					Controller*    old_ctrl = controller;
					ControlWidget* old_wdgt = ctrl_wdgt;
					if ( (! inputFile.isNull() ) && (! inputFile.isEmpty() ) ) {
						// using a variable DOES NOT WORK (gets *%§!$£ incorrect value)
						/* const char *fileName = inputFile.toStdString().c_str(); */
						// valid traj/ctrl file => imitate or file controller
						if (tracking) { // ask for the starting configuration
							static const int nb_val = 3;  // config. -> 3 values
							static const char* name_val[nb_val]
									= {"x", "y", "&theta;"};
							static const bool val_Greek[nb_val]
									= {false, false, true};
							double val[nb_val] = {0, 0, 0};
							getDoubleArray(1, nb_val, name_val, val_Greek, val);
							val[2] = Object::deg2rad(val[2]); // angle: deg.->rad.
							const Config start(Point(val[0], val[1]), val[2]);
							// ask which tracking controller
							QDialog select_dialog(this);
							select_dialog.setWindowTitle
									( tr("Select a tracking controller") );
							QDialogButtonBox select_wdgt(Qt::Horizontal);
							QVBoxLayout dialog_layout(&select_dialog);
							dialog_layout.addWidget(&select_wdgt);
							static const int nb_btn = 3;
							QPushButton select_btn[nb_btn];
							static const char* name_btn[nb_btn]
									= {"Analytical", "PID", "Imitate"};
							int idx;
							for(idx = 0; idx < nb_btn; idx++) {
								select_btn[idx].setText( tr(name_btn[idx]) );
								select_btn[idx].setAutoExclusive(true);
								select_btn[idx].setCheckable(true);
								select_btn[idx].setChecked(idx == 0);
								select_btn[idx].setDefault(idx == 0);
								// select a button closes the dialog (does not work)
								select_wdgt.addButton(&select_btn[idx],
													  QDialogButtonBox::AcceptRole);
								// add a connect
								connect( &select_btn[idx], SIGNAL( clicked() ),
										 &select_dialog, SLOT( close() ) );
							}
							select_dialog.exec();
							for(idx = 0; (idx < nb_btn) &&
										 ( !select_btn[idx].isChecked() ); idx++) ;
							// std::cout << "Selection: " << idx << std::endl;
							static const int nb_coef = 6;
							static const double h_coef = 1 / 0.35; // from [DP96]
							const double time_coef = 1 / time_step;
							double coef[nb_coef] =
									{ h_coef * h_coef, h_coef, 0,
									  time_coef * time_coef, time_coef, 0 };
							if (idx == 1) {
								static const char* name_coef[nb_coef]
										= { "Prop_trans", "Integ_trans", "Deriv_trans",
											"Prop_rot", "Integ_rot", "Deriv_rot" };
								static const bool coef_Greek[nb_coef]
										= {false, false, false, false, false, false};
								getDoubleArray(2, nb_coef / 2, name_coef,
											   coef_Greek, coef);
							}
							TrackingCtrl *new_ctrl =  // needed for trajectory()
									idx == 0 ? (TrackingCtrl*) new AnalyticCtrl
											( motion_model, time_step,
											  inputFile.toStdString().c_str(), start )
							: idx == 1 ? (TrackingCtrl*) new PIDCtrl
									( motion_model, time_step,
									  inputFile.toStdString().c_str(), start, coef )
							: (TrackingCtrl*) new ImitateCtrl
									( motion_model, time_step,
									  inputFile.toStdString().c_str(), start );
							controller = new_ctrl; // this is Controller*
							display.addTrajectory( new_ctrl->trajectory() );
						} else  controller = new FileCtrl
									( motion_model, time_step,
									  inputFile.toStdString().c_str() );
						ctrl_wdgt =
								new ViewCtrlWdgt(*this, *controller, motion_model);
					} else { // invalid control file => keyboard controller
						KbdCtrl* new_ctrl = new KbdCtrl(motion_model, time_step);
						controller = new_ctrl;  // this one is Controller*
						ctrl_wdgt  = new TeleopWidget(*this, *new_ctrl,
													  motion_model);
					} // end of else (keyboard controller)
					connectCtrl(); // connects controller signals to GUI methods
					hide();        // hide the window before modifying it
					// add the new control widget to the main window
					QGridLayout *central_layout =
							(QGridLayout*)( centralWidget()->layout() );
					old_wdgt->widget().hide(); // remove not enough, hide needed
					central_layout->removeWidget( &( old_wdgt->widget() ) );
					central_layout->addWidget(&( ctrl_wdgt->widget() ), 0, 0);
					show();  // show/redraw window once modifications are done
					// change has been done, let's free old pointers
					delete old_wdgt;
					delete old_ctrl;
					// valid save file => set it
					if ( (! saveFile.isNull() ) && (! saveFile.isEmpty() ) )
						display.setOutput( saveFile.toStdString().c_str() );
					ctrl_node.init(*controller);  // try to start control node
					not_connected = ! ctrl_node.connected();
				} // end of else (choose operating controller)
			} // end of if (trying)
		} while (trying && not_connected);
		if (trying)  start_stop_action.setText( tr("Stop Ctrl") );
	} // end of else (start control)
	start_stop_action.setEnabled(true);
} // end of void QtCtrlGUI::buttonStart() ----------------------------


/** @brief This is <tt>@ref index "qt_ctrl"</tt> main function, 
 **        starting a @ref QtCtrlGUI "Qt control GUI" 
 **        in a @ref refs_qt_application "Qt application".
 **
 ** @param argc  The parameters' count, and
 ** @param argv  the parameters' values, are both forwarded 
 **                  to Qt and ROS.
 **
 ** @return  The result of @ref refs_qt_application "Qt application"'s
 **          execution.
 **/
int main(int argc, char** argv) {
	QApplication app(argc, argv);  // start the Qt main loop
	QtCtrlGUI   ctrl(argc, argv);  // create the Qt control main window
	ctrl.show();                   // show this window
	// stop the Qt main loop when last window (= main window) is closed
	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
	return app.exec();             // return Qt main loop's return value
} // end of int main(int, char**) ------------------------------------
