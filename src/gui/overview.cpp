/**
 * @file /src/gui/overview.cpp
 *
 * @brief Qt based overview of the robot's motion.
 *
 * @date November 2018
 **/

#include <gui/overview.hpp>
#include <QGraphicsView>
#include <QVBoxLayout>

QtMotionGUI::QtMotionGUI() : motion_window(), not_first_state(false) {
    // Set the window's properties
    motion_window.setWindowTitle("Robot's Motion Overview");
    motion_window.setGeometry(200, 100, 800, 600);
    motion_window.setVisible(false);
    // Add the path scene in a first tab
    motion_window.addTab(new QGraphicsView(&path_scene),
                         "Motion's Path");
    // Add the velocities scenes in a second tab
    QWidget *vel_wdgt = new QWidget();
    QLayout *vel_layout = new QVBoxLayout(vel_wdgt);
    vel_layout->addWidget( new QGraphicsView(&trans_vel_scene) );
    vel_layout->addWidget( new QGraphicsView(&rot_vel_scene) );
    motion_window.addTab(vel_wdgt, "Motion's Velocities");
} // end of QtMotionGUI::QtMotionGUI() -------------------------------

void QtMotionGUI::addTrajectory(const std::list<State*>& trajectory) {
    static const Qt::GlobalColor traj_color = Qt::green;
    // adds each state of the trajectory, using traj_color
    for(std::list<State*>::const_iterator it = trajectory.begin();
        it != trajectory.end(); it++) {
        addState(**it, traj_color);
    } // end of for (each state of the trajectory)
    not_first_state = false;

    trajectory_states = trajectory;

    //trajectory_end = trajectory;
    trajectory_iterator = trajectory.begin();
} // end of void QtMotionGUI::addTrajectory(const std::list<State*>&)-

void QtMotionGUI::addState(const State& state,
                           const Qt::GlobalColor motion_color) {
    static const int abscIdx = 0, ordoIdx = abscIdx + 1,
            timeIdx = ordoIdx + 1, tranIdx = timeIdx + 1,  // was previously
            rotaIdx = tranIdx + 1, nmbrIdx = rotaIdx + 1;  // an enum
    if (not_first_state) {
        // Setting the color
        const QPen motion_pen = QColor(motion_color);
        // Scalling factors
        static const double factors[nmbrIdx]
                = {50, 50, 25, 100, 300 / M_PI};
        // extract the data from the state
        const Config old_q = last_state.configuration(),
                q = state.configuration();
        const Point  old_P = old_q.position(), P = q.position();
        // add a new line to the path (and redraw it)
        path_scene.addLine(factors[abscIdx] * old_P.xCoord(),
                           - factors[ordoIdx] * old_P.yCoord(),
                           factors[abscIdx] * P.xCoord(),
                           - factors[ordoIdx] * P.yCoord(), motion_pen);
        // add a new line to the translation velocity profile
        trans_vel_scene.addLine(factors[timeIdx] * last_state.date(),
                                - factors[tranIdx]
                                * last_state.translationVelocity(),
                                factors[timeIdx] * state.date(),
                                - factors[tranIdx]
                                * state.translationVelocity(),
                                motion_pen);
        // add a new line to the rotation velocity profile
        rot_vel_scene.addLine(factors[timeIdx] * last_state.date(),
                              - factors[rotaIdx]
                              * last_state.rotationVelocity(),
                              factors[timeIdx] * state.date(),
                              - factors[rotaIdx]
                              * state.rotationVelocity(), motion_pen);
    } // end of if (not_first_state)
    // memorize the last state for future drawing
    not_first_state = true;
    last_state = state;
} // end of void QtMotionGUI::addState(double[]) ---------------------


void QtMotionGUI::addStateAverage(const State& state, const Qt::GlobalColor motion_color){
   
 static const int abscIdx = 0, ordoIdx = abscIdx + 1,
            timeIdx = ordoIdx + 1, tranIdx = timeIdx + 1,  // was previously
            rotaIdx = tranIdx + 1, nmbrIdx = rotaIdx + 1;  // an enum




        const State ti = **trajectory_iterator;

        // extract the data from the state
        const Config q = state.configuration(), qTI = ti.configuration();
        const Point P = q.position(), PTI = qTI.position();

	const float averageX = (PTI.xCoord() +P.xCoord())/2;
	const float averageY = (PTI.yCoord() +P.yCoord())/2;
	//std::cout << "lol"<<averageX << " " << averageY << std::endl;

    if (average_not_first_state) {
        // Setting the color
        const QPen motion_pen = QColor(motion_color);
        // Scalling factors
        static const double factors[nmbrIdx]
                = {50, 50, 25, 100, 300 / M_PI};


       /* const State ti = **trajectory_iterator;

        // extract the data from the state
        const Config old_q = average_last_state.configuration(),
                q = state.configuration(), qTI = ti.configuration();
        const Point  old_P = old_q.position(), P = q.position(), PTI = qTI.position();*/

	
        // add a new line to the path (and redraw it)
       // path_scene.addLine(factors[abscIdx] * old_P.xCoord(),
         //                  - factors[ordoIdx] * old_P.yCoord(),
                          
        path_scene.addLine(factors[abscIdx] * average_lastX,
                           - factors[ordoIdx] * average_lastY,
                             factors[abscIdx] * averageX,
                           - factors[ordoIdx] * averageY, motion_pen);
        // add a new line to the translation velocity profile
        trans_vel_scene.addLine(factors[timeIdx] * last_state.date(),
                                - factors[tranIdx]
                                * last_state.translationVelocity(),
                                factors[timeIdx] * state.date(),
                                - factors[tranIdx]
                                * state.translationVelocity(),
                                motion_pen);
        // add a new line to the rotation velocity profile
        rot_vel_scene.addLine(factors[timeIdx] * last_state.date(),
                              - factors[rotaIdx]
                              * last_state.rotationVelocity(),
                              factors[timeIdx] * state.date(),
                              - factors[rotaIdx]
                              * state.rotationVelocity(), motion_pen);
    } // end of if (not_first_state)
    // memorize the last state for future drawing
    average_not_first_state = true;
    average_last_state = state;
    average_lastX = averageX;
    average_lastY = averageY;


    if(trajectory_iterator != trajectory_states.end()){
     trajectory_iterator++;
   }
}
