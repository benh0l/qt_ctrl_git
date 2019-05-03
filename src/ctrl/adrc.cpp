
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

  double xR = state.configuration().position().xCoord();
  double yR = state.configuration().position().yCoord();
  double thetaR = state.configuration().orientation();

  // filtrage des points
  double fenetreTemps = 3.;
  double fenetreDistance = 0.25;
  double distance;
  State *pointA, *pointAplus1, *pointAmoins1, *precedent, *pointB;
  double distA = 999999., distAplus1 = 999999., distAmoins1 = 999999.;
  double xIter, yIter;

  int nbPoints = 0;

  bool continuer = true;
  bool besoinAplus1 = false;

  log2ROS(Controller::LogLevel::Info,	"ADRC velocities choice!");
  
  std::list<State*>::const_iterator iter = traject.begin();
  // on cherche le premier point dans l'intervalle de temps accepte
  while ( ( iter != traject.end() )
	  && ( (*iter)->date() < state.date() - fenetreTemps ) )
    iter ++;  
      
  if ( iter == traject.end() ){
    iter--;
    continuer = false;
  }

  precedent = (*iter);
  xIter = (*iter)->configuration().position().xCoord();
  yIter = (*iter)->configuration().position().yCoord();
  nbPoints++;

  // on cherche le deuxieme point dans l'intervalle de temps accepte
  while ( continuer &&( iter != traject.end() )
	  && ( fenetreDistance > sqrt(pow(xIter - (*iter)->configuration().position().xCoord(), 2)
				      + pow(yIter - (*iter)->configuration().position().yCoord(), 2)) )){
    iter++;
    
    if ( iter == traject.end() ){
      iter--;
      continuer = false;
    }
    
  }

  // on parcourt tous les points de l'intervalle de temps accepte, en gardant les plus proches du robot
  while ( continuer && ( iter != traject.end() )
	  && ( (*iter)->date() < state.date() + fenetreTemps ) ){

    xIter = (*iter)->configuration().position().xCoord();
    yIter = (*iter)->configuration().position().yCoord();

    distance = sqrt(pow(xIter - xR, 2)+ pow(yIter - yR, 2));

    // std::cout << distance  << "\n";
    if(distance < distA){
      pointA = (*iter);
      distA = distance;
      pointAmoins1 = precedent;
      besoinAplus1 = true;
    }    
    // std::cout << pointA  << "\n";
    //on saute au prochain point suffisament loin
    nbPoints ++;
    while ( continuer &&( iter != traject.end() )
            && ( fenetreDistance > sqrt(pow(xIter - (*iter)->configuration().position().xCoord(), 2)
					+ pow(yIter - (*iter)->configuration().position().yCoord(), 2))) ){
      iter++;
        
      if ( iter == traject.end() ){
	iter--;
	continuer = false;
      }
        
    }  
    if(besoinAplus1){
      besoinAplus1 = false;
      pointAplus1 = (*iter);
    }
   

    if ( iter == traject.end() ){
      iter--;
      continuer = false;
    }
  }
  //std::cout << pointA->configuration().position().xCoord()  << " avec "<< nbPoints<< "\n";

  // on a le point le plus proche, on compare maintenant le point precedent avec le point suivant
  if(sqrt(pow((*pointAmoins1).configuration().position().xCoord() - xR, 2)
	  + pow((*pointAmoins1).configuration().position().yCoord() - yR, 2))
     <= sqrt(pow((*pointAplus1).configuration().position().xCoord() - xR, 2)
	     + pow((*pointAplus1).configuration().position().yCoord() - yR, 2))){
    pointB = pointA;
    pointA = pointAmoins1;
  }else{
    //pointA = pointA;
    pointB = pointAplus1;
  }        



  // calcul de P

  double xA = (*pointA).configuration().position().xCoord();
  double yA = (*pointA).configuration().position().yCoord();
  double thetaA = (*pointA).configuration().orientation();

  double xB = (*pointB).configuration().position().xCoord();
  double yB = (*pointB).configuration().position().yCoord();
  double thetaB = (*pointB).configuration().orientation();


  double xAPrim = (*pointA).translationVelocity() * cos(thetaA);
  double yAPrim = (*pointA).translationVelocity() * sin(thetaA);

  double xBPrim = (*pointB).translationVelocity() * cos(thetaB);
  double yBPrim = (*pointB).translationVelocity() * sin(thetaB);

  //interpolation de a et b
  double delta_t = (*pointB).date() - (*pointA).date();
  double tA = (*pointA).date();

  double six = (xB - xA)/delta_t;
  double siy = (yB - yA)/delta_t;

  double cx0 = xA;
  double cy0 = yA;

  double cx1 = xAPrim;
  double cy1 = yAPrim;


  double cx2 = (3 * six - xBPrim - 2*xAPrim)/delta_t;
  double cy2 = (3 * siy - yBPrim - 2*yAPrim)/delta_t;

  double cx3 = (xBPrim + xAPrim - 2 * six)/(delta_t * delta_t);
  double cy3 = (yBPrim + yAPrim - 2 * siy)/(delta_t * delta_t);


  // calcul de P

//double tStar = (*pointA).date();
double tStar = (*pointB).date();
double xdtStar, ydtStar;
double xPrim_dStar, yPrim_dStar;
double f_tStar, fPrim_tStar;
double tStarMoinstA;


bool trouvePlus = false, trouveMoins = false, trouveEgal = false;
double valeur;
double plus, moins, egal, proche, xProche, yProche;
double valProche = 999999.;
int i = 0;
double ecart = 0.1;
int essais = 0;

while(!trouveEgal && (!trouveMoins || !trouvePlus)){

tStar = (*pointA).date() + ecart * delta_t * i;
tStarMoinstA = tStar-tA;

xdtStar = cx3 * pow(tStarMoinstA, 3) + cx2 * pow(tStarMoinstA, 2) + cx1 * tStarMoinstA + cx0;
ydtStar = cy3 * pow(tStarMoinstA, 3) + cy2 * pow(tStarMoinstA, 2) + cy1 * tStarMoinstA + cy0;

//xPrim_dStar = 3*cx3*pow(tStarMoinstA, 2) + 2*cx2*tStarMoinstA + cx1*tStar;
//yPrim_dStar = 3*cy3*pow(tStarMoinstA, 2) + 2*cy2*tStarMoinstA + cy1*tStar;
xPrim_dStar = 3*cx3*pow(tStarMoinstA, 2) + 2*cx2*tStarMoinstA + cx1;
yPrim_dStar = 3*cy3*pow(tStarMoinstA, 2) + 2*cy2*tStarMoinstA + cy1;

f_tStar = (yPrim_dStar*(yR - ydtStar)) + (xPrim_dStar*(xR - xdtStar));

if(abs(f_tStar)<valProche){
	valProche = abs(f_tStar);
	proche = tStar;
	xProche = xdtStar;
	yProche = ydtStar;
}


if(f_tStar == 0){
	egal = tStar;
	trouveEgal == true;
}else{
	if(f_tStar < 0){
		moins = tStar;
		trouveMoins = true;
	}else{
		plus = tStar;
		trouvePlus = true;

	}
}
//std::cout << yAPrim << " " << yPrim_dStar<< " "<< yBPrim<<"\n";
//std::cout << yA << " " << ydtStar<< " "<< yB<<"\n";
//std::cout << " Un "<< tStar<< " "<< f_tStar<< " "<< trouvePlus<< " "<<trouveMoins<< "  "<<ecart * i<<"\n";

tStar = (*pointB).date() - ecart * delta_t * i;
tStarMoinstA = tStar-tA;

xdtStar = cx3 * pow(tStarMoinstA, 3) + cx2 * pow(tStarMoinstA, 2) + cx1 * tStarMoinstA + cx0;
ydtStar = cy3 * pow(tStarMoinstA, 3) + cy2 * pow(tStarMoinstA, 2) + cy1 * tStarMoinstA + cy0;


xPrim_dStar = 3*cx3*pow(tStarMoinstA, 2) + 2*cx2*tStarMoinstA + cx1;
yPrim_dStar = 3*cy3*pow(tStarMoinstA, 2) + 2*cy2*tStarMoinstA + cy1;

f_tStar = (yPrim_dStar*(yR - ydtStar)) + (xPrim_dStar*(xR - xdtStar));

if(abs(f_tStar)<valProche){
	valProche = abs(f_tStar);
	proche = tStar;
	xProche = xdtStar;
	yProche = ydtStar;
}

if(f_tStar == 0){
	egal = tStar;
	trouveEgal = true;
}else{
	if(f_tStar < 0){
		moins = tStar;
		trouveMoins = true;
	}else{
		plus = tStar;
		trouvePlus = true;

	}
}

if(ecart * i > 0.5){
	i = 1;
	ecart = ecart / 3;
	essais++;
	if(essais > 5){
		trouveEgal = true;
		egal = proche;
		xdtStar = xProche;
		ydtStar = yProche;

		std::cout << " Abandon "<< proche <<"\n";
	}
}else{
	i++;
} 
//std::cout << " Un "<< tStar<< " "<< f_tStar<< " "<< trouvePlus<< " "<<trouveMoins<< "  "<<ecart * i<<"\n";
}

while(!trouveEgal && (abs(moins - plus)>0.001)){

tStar = (plus + moins)/2;
tStarMoinstA = tStar-tA;

xdtStar = cx3 * pow(tStarMoinstA, 3) + cx2 * pow(tStarMoinstA, 2) + cx1 * tStarMoinstA + cx0;
ydtStar = cy3 * pow(tStarMoinstA, 3) + cy2 * pow(tStarMoinstA, 2) + cy1 * tStarMoinstA + cy0;

xPrim_dStar = 3*cx3*pow(tStarMoinstA, 2) + 2*cx2*tStarMoinstA + cx1;
yPrim_dStar = 3*cy3*pow(tStarMoinstA, 2) + 2*cy2*tStarMoinstA + cy1;

f_tStar = (yPrim_dStar*(yR - ydtStar)) + (xPrim_dStar*(xR - xdtStar));

if(abs(f_tStar)<valProche){
	valProche = abs(f_tStar);
	proche = tStar;
	xProche = xdtStar;
	yProche = ydtStar;
}

if(f_tStar == 0){
	egal = tStar;
	trouveEgal = true;
}else{
	if(f_tStar < 0){
		moins = tStar;
	}else{
		plus = tStar;

	}
}
std::cout << " Deux "<< f_tStar<< "\n";
}



//std::cout << tStar  << " -----------------------\n";
/*for (int i = 0; i < 100; i++){

    tStarMoinstA = tStar-tA;

    xdtStar = cx3 * pow(tStarMoinstA, 3) + cx2 * pow(tStarMoinstA, 2) + cx1 * tStarMoinstA + cx0;
    ydtStar = cy3 * pow(tStarMoinstA, 3) + cy2 * pow(tStarMoinstA, 2) + cy1 * tStarMoinstA + cy0;

    xPrim_dStar = 3*cx3*pow(tStarMoinstA, 2) + 2*cx2*tStarMoinstA + cx1*tStar;
    yPrim_dStar = 3*cy3*pow(tStarMoinstA, 2) + 2*cy2*tStarMoinstA + cy1*tStar;

    f_tStar = (yPrim_dStar*(yR - ydtStar)) + (xPrim_dStar*(xR - xdtStar));


    fPrim_tStar = ((6*cy3*tStarMoinstA + 2*cy2)*(yR - ydtStar))
      +((6*cx3*tStarMoinstA + 2*cx2)*(xR - xdtStar))
      -pow(xPrim_dStar, 2) 
      -pow(yPrim_dStar, 2);

tStar = tStar - (f_tStar / fPrim_tStar);
 //std::cout << tStar  << " f "<< f_tStar << " f'" << fPrim_tStar<<"\n";
}
*/

  //calcul de Q

  //xQ = xdtStar - (yR - ydtStar);
  //yQ = ydtStar + (xR - xdtStar);

  double xQ = xdtStar + cos (thetaR);
  double yQ = ydtStar + sin (thetaR);

//double vx = xQ - xR;
//double vy = yQ - yR;
double vx = xdtStar - xQ;
double vy = ydtStar - yQ;
//xQ = xdtStar - (yR - ydtStar);
//yQ = ydtStar + (xR - xdtStar);

//normalisation?
//std::cout <<  sqrt(pow(xQ - xR, 2)+ pow(yQ - yR, 2))<<"\n";
//std::cout << vx <<" " << vy<< "\n";

/*double a = 2;
double b = 2*vx + 2 *vy;
double c = vx * vx + vy * vy - 1;

  double solution1 = 0, solution2 = 0;

  double landa = b*b-4*a*c;
  // std::cout<<"landa = "<<landa;
    
  if(landa<0)
    {
      std::cout<<"il n'y a pas de solution\n";
    }
    
  if(landa==0)
    {
      // std::cout<<"landa est egal a zero, il existe une racine unique\n";
          
      solution1= -b /2*a;
      //      std::cout<<" la racine unique est\n "<<landa;
    }
  if(landa>0)
    {
      //std::cout<<"landa est superieure a zero, il existe donc deux racines : ";
      solution1= -b + sqrt(landa) /2*a;
      solution2 = -b - sqrt (landa) /2*a;
      // std::cout<<"les racines sont "<<solution3<<" et "<<solution2;
    }
       
*/
double lambda = sqrt(1/(vx *vx + vy*vy));

if(lambda * vx *vx + lambda*vy * vy > 0){

xQ = xdtStar + lambda * vx;
yQ = ydtStar + lambda * vy;
}else{
xQ = xdtStar - lambda * vx;
yQ = ydtStar - lambda * vy;
}


double tQ = sqrt((xQ - xR)*(xQ - xR)+ (yQ - yR)*(yQ - yR))/ (*pointB).translationVelocity();

  //std::cout << xR  << " , "<< yR << " // " << xdtStar  << " , "<< ydtStar << " // "<< xQ  << " , "<< yQ << " "<< ((xR-xdtStar)*(xR-xQ )+(yR-ydtStar)*(yR-yQ ))<<"\n";



  // calculs d'omega et theta
  double thetaFinal = atan2(yQ-yR, xQ-xR);
  double omegaFinal = (thetaFinal - thetaR)/(tQ-state.date());

  if(omegaFinal != omegaFinal) {
    omegaFinal =  (*pointA).rotationVelocity(); 
    std::cout <<omegaFinal<< " NANi?\n";
  }

//std::cout << xR  << " , "<< yR << " // " << xQ  << " , "<< yQ << " // "<<  tQ << "  ; "<< thetaFinal << " ; "<< omegaFinal<<"\n";

  searchGoal();  // updates the goal to be after the robot's date
  //searchGoal(0.05);  // updates the goal to be after the robot's date
  //searchGoal(0.2);  // updates the goal to be after the robot's date
  // gets the goal's velocities
  moving_velocity = (*goal)->translationVelocity();
  //moving_velocity = (*pointA).translationVelocity();  
  //turning_velocity = (*goal)->rotationVelocity();   
  // turning_velocity = (*pointA).rotationVelocity(); 
  turning_velocity = omegaFinal;


  //moving_velocity +=0.1; 
  // turning_velocity += 0.1; 
  /*
    double xi = state.configuration().position().xCoord();
    double yi = state.configuration().position().yCoord();
    double thetai = state.configuration().orientation();
    double omegai = state.rotationVelocity();

    double xiplus1 = (*goal)->configuration().position().xCoord();
    double yiplus1 =  (*goal)->configuration().position().yCoord();
    double thetaiplus1 = (*goal)->configuration().orientation();

    double xiPrim = state.translationVelocity() * cos(thetai);
    double yiPrim = state.translationVelocity() * sin(thetai);

    double xiplus1Prim = (*goal)->translationVelocity() * cos(thetaiplus1);
    double yiplus1Prim = (*goal)->translationVelocity() * sin(thetaiplus1);

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
    double cy3 = (yiplus1Prim + yiPrim - 2 * siy)/(delta_t * delta_t);*/


  //double xd0 = cx3 * pow(0.05 * delta_t, 3) + cx2 * pow(0.05 * delta_t, 2) + cx1 * 0.05 + cx0;
  //double yd0 = cy3 * pow(0.05 * delta_t, 3) + cy2 * pow(0.05 * delta_t, 2) + cy1 * 0.05 + cy0;

  //double xd00 = cx3 * pow(0.1 * delta_t, 3) + cx2 * pow(0.1 * delta_t, 2) + cx1 * 0.1 + cx0;
  //double yd00 = cy3 * pow(0.1 * delta_t, 3) + cy2 * pow(0.1 * delta_t, 2) + cy1 * 0.1 + cy0;

  /*
    double xd1 = cx3 * pow(0.49 * delta_t, 3) + cx2 * pow(0.49 * delta_t, 2) + cx1 * 0.49 + cx0;
    double yd1 = cy3 * pow(0.49 * delta_t, 3) + cy2 * pow(0.49 * delta_t, 2) + cy1 * 0.49 + cy0;

    double xd2 = cx3 * pow(0.5 * delta_t, 3) + cx2 * pow(0.5 * delta_t, 2) + cx1 * 0.5 + cx0;
    double yd2 = cy3 * pow(0.5 * delta_t, 3) + cy2 * pow(0.5 * delta_t, 2) + cy1 * 0.5 + cy0;

    double xd3 = cx3 * pow(0.51 * delta_t, 3) + cx2 * pow(0.51 * delta_t, 2) + cx1 * 0.51 + cx0;
    double yd3 = cy3 * pow(0.51 * delta_t, 3) + cy2 * pow(0.51 * delta_t, 2) + cy1 * 0.51 + cy0;

  */

  //double xd1 = cx3 * pow(0.09 * delta_t, 3) + cx2 * pow(0.09 * delta_t, 2) + cx1 * 0.09 + cx0;
  //double yd1 = cy3 * pow(0.09 * delta_t, 3) + cy2 * pow(0.09 * delta_t, 2) + cy1 * 0.09 + cy0;

  //double xd2 = cx3 * pow(0.1 * delta_t, 3) + cx2 * pow(0.1 * delta_t, 2) + cx1 * 0.1 + cx0;
  //double yd2 = cy3 * pow(0.1 * delta_t, 3) + cy2 * pow(0.1 * delta_t, 2) + cy1 * 0.1 + cy0;

  //double xd3 = cx3 * pow(0.11 * delta_t, 3) + cx2 * pow(0.11 * delta_t, 2) + cx1 * 0.11 + cx0;
  //double yd3 = cy3 * pow(0.11 * delta_t, 3) + cy2 * pow(0.11 * delta_t, 2) + cy1 * 0.11 + cy0;

  //double t049 = 0.49 + ti;


  /*double xdPrim = cx3 * 3 * t049 * t049 
    - 3 * cx3 * ti * 2 * t049
    + cx2 * 2 * t049
    + 3*cx3 * ti * ti  
    - cx2 *2 * ti 
    + cx1;*/

  /*
    double a = cx3;
    double b = cx2;
    double d = cy3;
    double e = cy2;
    double x = 1;

    double omegaDeriv = (-0.4*e+b*x*e*b-0.2*d*x*b-0.01*d*b-0.2*x*x*x*e+a*a*a*e*a+0.2*e*b+0.4*e*x*x*e+a*a+0.02*e*e+a*x*a+4*b*e*x+2*b*d-2*b*d*x*x)
    /pow((d*(x*x+0.1*x+1)+e*(0.1+2*x)),2)*(1+pow(((a*(x*x+0.1*x+1)+b*(0.1+2*x))/(d*(x*x+0.1*x+1)+e*(0.1+2*x))),2));
  */

  /*
    double thetaAtan1 = atan2(xd2-xd1,yd2-yd1);
    double thetaAtan2 = atan2(xd3-xd2,yd3-yd2);

    double theta1_2 =  atan((xd2-xd1) / (yd2-yd1));
    double theta2_3 =   atan((xd3-xd2) / (yd3-yd2));*/

  //double theta00 =  atan((xd0-xi) / (yd0-yi));

  //double theta0 =  atan((xd00-xd0) / (yd00-yd0));

  //Que faire en cas de div par zero?

  //double omega = (theta0 - theta00) / 0.05;
  //double omega = (theta2_3 - theta1_2) / 0.01;
  //double omega = (theta2_3 - theta1_2) / 0.01;
  //double omega = (thetaAtan2 - thetaAtan1) / 0.01;
  //double omega = (theta1_2 - state.rotationVelocity()) / 0.5;
  //double omega = (theta0 - state.rotationVelocity()) / 0.1;

  //std::cout << xdPrim  << " au lieu de "<< theta1_2  << "\n";
  //std::cout << omega  << " au lieu de "<< state.rotationVelocity()  << "\n";

  //std::cout <<  theta1_2 << " et "<<  theta2_3  << " contre "<< thetai  << "\n";
  //std::cout <<  theta1_2 << " et "<<  theta2_3  << " contre "<< thetaAtan1  << "\n";
  //std::cout <<  theta1_2 << " et "<<  theta2_3  << " contre "<< state.rotationVelocity()  << "\n";
  //std::cout << state.rotationVelocity() << " puis "<< omega << " puis "<< (*goal)->rotationVelocity()<< "\n";

  //std::cout << (*goal)->rotationVelocity()*100/(*goal)->translationVelocity() << "\n";


  //std::cout <<  acos(((xd2 - xd1)/0.01)/(*goal)->translationVelocity()) << "  " << asin(((yd2 - yd1)/0.01)/(*goal)->translationVelocity())<< "  " <<(*goal)->translationVelocity() << "\n";

  //std::cout << xi << " puis "<< xd1 << " puis "<< xiplus1<< "\n";

  //double vitesse = sqrt(pow(xiplus1 - xi, 2)+ pow(yiplus1 - yi, 2));

  //std::cout << delta_t << ": "<< vitesse << " au lieu de "<<(*goal)->translationVelocity()<< "\n";

  //std::cout << omegaDeriv << " a la place de " << omega << "\n";

  //moving_velocity = vitesse;
  //turning_velocity = omega;
  //turning_velocity = omegaDeriv;

  //moving_velocity = 2.; 
  //turning_velocity = 3.; 
  // updates the parameters and send the update signal
  updateVelocities(trans_vel, rot_vel);
} // end of void ImitateCtrl::chooseVelocities(double&, double&, ...)-



