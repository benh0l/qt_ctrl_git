/**
 ** @file  src/model/Dubins.cpp
 **
 ** @brief Shortest path between two configurations.
 **
 ** @date  November 2018
 **/

#include <model/Dubins.hpp>


/* Finds the configuration at a given length along the path.
 * Parameter curv_lgth  the curvilinear length.
 * Returns the configuration at the given curvilinear length.
 */
Config DubinsPath::ConstCurvArc::operator[](const double l) const {
  Config res;
  if (l <= 0)           res = start;
  else if (l > length)  res = end;
  else {  // l is OK, let's compute
    const double theta = start.orientation(); 
    const double delta_orient = l * curv;
    if ( Object::isZero(curv) )  // line segment
      res = Config(start.position() + l * Vector(theta), theta);
    else  // circular arc
      res = Config(start.position()
		   + 1 / curv * Vector(sin(delta_orient), 
				       1 - cos(delta_orient) )
		   .rotate(theta), theta + delta_orient);
  } // end of if (l OK)
  return res; 
} // end of Config DubinsPath::ConstCurvArc::operator[](...) const ---

// Fills the different arcs of a path.
//
// The starting configurations (except the first one, 
//  which is the path's starting configuration) are given.
// Final configuration of each arc is either the next arc's
// starting configuration or the path's final configuration.
// Curvature are found using the path type and the maximum 
// curvature. 
//
// Parameter start    the initial configuration, 
//           end      the final configuration,
//           configs  the intermediate starting configurations, 
//           type     the type of the path, 
//           lengths  the lengths of the arcs, 
//           center   the turning centers of the arcs.
void DubinsPath::fillArcs(const Config& start, const Config& end,
			  const Config* configs[nbConfig],
			  const Type type, 
			  const double lengths[nbArcs], 
			  const Point* center[nbArcs]){
  path_type = type; 
  // sets up the extremities configuration
  paths[0].start = start; 
  paths[nbConfig].end = end;
  total_length = 0;  // and the total length
  // for each arc
  for(int idx = 0; idx < nbArcs; idx++) {
    if (idx < nbConfig)  // end and next config., except for the last
      paths[idx].end  = paths[idx + 1].start = *configs[idx];
    paths[idx].curv   = turnSign(type, idx) * maxi_curv;   // curvature
    paths[idx].length = lengths[idx];                      // length
    total_length     += lengths[idx]; 
    paths[idx].center = *center[idx]; 
  } // end of for (each arc)
} // end of void DubinsPath::fillArcs(const Config[], const Type, ...) 
  
// Computes the different arcs of a path.
//
// The configurations are computed from the first one, the type
// and the deflections / length.
// Curvature are found using the path type and the maximum 
// curvature. 
//
// Parameter start    the initial configuration, 
//           sgn      the signs of the curvatures,
//           lengths  the deflections or length of the arcs, 
//           center   the turning centers of the arcs.
void DubinsPath::computeArcs(const Config& start,
			     const int sgn[nbArcs], 
			     const double defl_lngth[nbArcs], 
			     const Point* center[nbArcs]) {
  static const double PIx2 = 2 * M_PI;
  Config q = start; 
  total_length = 0; 
  for(int idx = 0; idx < nbArcs; idx++) { // sets the arcs
    // sign of the deflection is constrained
    const double correct_defl = (sgn[idx] * defl_lngth[idx] < 0 )
      ? defl_lngth[idx] + sgn[idx] * PIx2 : defl_lngth[idx],
      curv = sgn[idx] * maxi_curv,
      // special length for the straight segment
      l = sgn[idx] == 0 ? correct_defl : correct_defl / curv; 
    paths[idx].start  = q;
    paths[idx].curv   = curv;
    paths[idx].length = l;
    total_length     += l; 
    paths[idx].center = *center[idx]; 
    q = paths[idx].end = paths[idx][l]; 
  } // end of for (sets each arc)
} // end of void DubinsPath::computeArcs(const Type, ...) ------------

/* Connects two configurations. 
 * Parameter  start     the initial configuration,
 *            end       the final configuration. 
 */
void DubinsPath::connect(const Config& start, const Config& end) {
  static const Point infty_pt(INFINITY, INFINITY); 
  const Point& start_pos = start.position(), end_pos = end.position();
  if (! isPositive(maxi_curv) ) // If maximum curvature is very small
    if ( start.isAlignedWith(end) && start.hasInFront(end_pos) ) { 
      const Config* cfgs[nbConfig] = {&start, &end}; 
      const double lgts[nbArcs]    =
	{0, start.position().distance(end_pos), 0};
      const Point* centers_ptr[nbArcs] =
	{&infty_pt, &infty_pt, &infty_pt}; 
      // any *s* connects, lsl chosen
      fillArcs(start, end, cfgs, lsl, lgts, centers_ptr); 
    } else  // small max. curvature but not aligned or not in front
      setNoMove(start);  // no solution -> no path
  else {  // General case (max. curvature big enough)
    const double radius = 1. / maxi_curv;  // not infinite
    Vector start_vect(start.orientation() + M_PI_2), // normal vectors
      end_vect(end.orientation() + M_PI_2);        // at start and end
    start_vect.multiply(radius);
    end_vect.multiply(radius);
    const Point centers[nbTurningCircles]  // turns' centers
      = { start_pos + start_vect, start_pos - start_vect,
	  end_pos - end_vect, end_pos + end_vect };
    // computes all possible paths
    DubinsPath all_paths[nbPossiblePaths];
    int typeIdx;  // for each of these paths
    for(typeIdx = 0; typeIdx < nbPossiblePaths; typeIdx++) {
      DubinsPath &path = all_paths[typeIdx]; 
      path.path_type = (Type) typeIdx; 
      // finds 1st and last curvature signs of the path
      const int sgn[nbArcs] =
	{ turnSign(path.path_type, 0), turnSign(path.path_type, 1),
	  turnSign(path.path_type, 2) };
      // gets the associated turning centers
      const Point& center1 = centers[ (1 - sgn[0]) / 2 ], 
	center3 = centers[ 2 + (1 + sgn[2]) / 2 ]; 
      // gets the vector connecting these turning centers
      const Vector centers_vect = center3 - center1;
      const double dist = centers_vect.length();  // its length
      if ( isPositive(dist) ) {  // if circle's centers are distant 
	const double dir = centers_vect.orientation();
	// different categories -> different computations
	const bool typeCat[2] =
	  { (path.path_type == lsl) || (path.path_type == rsr), 
	    (path.path_type == lsr) || (path.path_type == rsl) },
	  has_line = typeCat[0] || typeCat[1];
	// proportion value depends of the category
	const double prop_val = typeCat[0] ? 0 : typeCat[1] 
	  ? 2 * radius / dist : 16 * radius * radius - dist * dist;
	// it determines whether path exists
	const bool no_path = (! typeCat[0]) &&     // OK for lsl & rsr
	  ( (! typeCat[1]) || (prop_val > 1) ) &&  // ! far enough 
	  ( (typeCat[1]) || (prop_val < 0) );   // too far (lrl & rlr)
	if (no_path)  path.setNoMove(start);  // no solution -> no path
	else {  // path exists, let's play geometry
	  // length of the middle segment (also depends of the categ.)
	  const double middle_length = typeCat[0] ? dist 
	    : typeCat[1] ? dist * sqrt(1 - prop_val * prop_val) : 0;
	  // the angle between the segment connecting the circles
	  // and the orientation at the end of the first arc, idem
	  const double middle_angle = typeCat[0] ? 0 : typeCat[1] 
	    ? sgn[0] * asin(prop_val) : sgn[0] 
	    * (atan(sqrt(prop_val) / dist) + M_PI_2);
	  // orientation at the end of the first arc
	  const double middle_dir
	    = mod2pi(dir + middle_angle),
	    // orientation at the beginning of the last arc
	    last_dir = has_line ? middle_dir
	    : mod2pi(dir - middle_angle), defl[nbArcs] = 
	    // deflections of each parts 
	    { middle_dir - start.orientation(), 
	      has_line ? middle_length : mod2pi(- 2 * middle_angle),
	      end.orientation() - last_dir};
	  // center of middle turn, and list of center's pointers
	  const Point center2 = has_line ? infty_pt : 
	    center1 + Vector(middle_dir + sgn[1] * M_PI_2)
	    * (2 * radius), 
	    *centers_ptr[nbArcs] = {&center1, &center2, &center3};
	  path.computeArcs(start, sgn, defl, centers_ptr);
	} // end of else (path exists)
      } else  // the turning circles are equal, OK for lsl and rsr
	if ( (path.path_type == lsl) || (path.path_type == rsr) ) { 
	  const double defl[nbArcs] = {end.orientation()
				       - start.orientation(), 0, 0};
	  const Point* centers_ptr[nbArcs] = 
	    {&center1, &center1, &center3}; 
	  path.computeArcs(start, sgn, defl, centers_ptr); 
	} else  setNoMove(start);  // no path possible
    } // end of for (each possible path)
    // the solution is the shortest path: find the best index
    int idx, best = 0;
    double shortest = all_paths[best].length();
    for(idx = 1; idx < nbPossiblePaths; idx++) {
      double current = all_paths[idx].length();
      if ( isPositive(current) &&  // valid & better 
	   ( (current < shortest) || (! isPositive(shortest) ) ) ) 
	{ shortest = current;  best = idx; }
    } // end of for (each possible path)
    // copy the best path
    *this = all_paths[best]; 
  } // end of else (general case)
} // end of void DubinsPath::connect(const Config&, const Config&) ---

/* Finds the configuration at a given length along the path.
 * Parameter curv_lgth  the curvilinear length.
 * Returns the configuration at the given curvilinear length.
 */
Config DubinsPath::operator[](const double curv_lgth) const {
  double l = curv_lgth;
  int idx; 
  // search in which arc we are
  for(idx = 0; (idx < nbArcs) && (l > paths[idx].length); idx++)
    l -= paths[idx].length;
  return (idx == nbArcs ? paths[idx - 1].end : paths[idx][l]); 
} // end of DubinsPath::operator[](const double) const ---------------
