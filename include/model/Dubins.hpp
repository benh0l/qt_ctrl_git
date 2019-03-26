/**
 ** @file  include/model/Dubins.hpp
 **
 ** @brief Shortest path between two configurations.
 **
 ** @date  November 2018
 **/

#ifndef QTCTRL_DUBINS
#define QTCTRL_DUBINS


#include <string.h>   // for memcpy
#include <model/configuration.hpp>


/** 
 ** @brief This class computes the shortest path 
 **        between two configurations.
 ** 
 ** @since 0.3.1
 */
class DubinsPath : public Object {
public:
  /// @brief The number of different arcs in a path.
  static const int nbArcs = 3;
  
  /// @brief Dubins path can have several type, according to 
  ///        the nature of their three parts.
  enum Type {
    lsl,   ///< Parts are left (turn), straight (line), left (turn).
    lsr,   ///< Parts are left (turn), straight (line), right (turn).
    rsl,   ///< Parts are right (turn), straight (line), left (turn).
    rsr,   ///< Parts are right (turn), straight (line), right (turn).
    lrl,   ///< Parts are left (turn), right (turn), left (turn).
    rlr,   ///< Parts are right (turn), left (turn), right (turn).
    nbPossiblePaths  /// The number of possible paths.
  };
  
  /// @brief Gives the sign of the curvature for a given type of path
  ///        and arc number.
  /// @param type  the type of the considered path,
  /// @param part  the arc number (corrected if not valid).
  /// @return  The corresponding sign of the curvature.
  /// @see validIndex
  static int turnSign(const Type type, const int part) {
    static const int turn_sign[nbPossiblePaths][nbArcs] =
      { {1, 0, 1}, {1, 0, -1}, {-1, 0, 1},       // lsl, lsr, rsl,
	{-1, 0, -1}, {1, -1, 1}, {-1, 1, -1} };  // rsr, lrl, rlr
    return turn_sign[type][validIndex(part)];  // l = 1, r = -1, s = 0
  }
  
private:
  /// @brief The number of usefull turning circles.
  static const int nbTurningCircles = 4;
  
  /// @brief Circular arcs or line segments
  ///        (constant curvature paths).
  class ConstCurvArc {
  public:
    Config start;   ///< The initial configuration.
    double curv;    ///< The constant curvature. 
    double length;  ///< The length of the arc.
    Config end;     ///< The final configuration.
    Point  center;  ///< The turning center (at infinity for segment).
    
    /// @brief A default constructor is needed to build arrays. 
    ConstCurvArc() : ConstCurvArc(Config(), 0, 0) {}
    
    /// @brief The constructor needs the starting configurations,
    ///        the curvature and the length. 
    /// @param q  the initial configuration,
    /// @param k  the constant curvature,
    /// @param l  the length of the arc. 
    ConstCurvArc(const Config& q, const double& k, const double& l)
      : start(q), curv(k), length(l), end( operator[](l) ) {}
    
    /** @brief Finds the configuration at a given length along 
     **        the path.
     ** @param l  the curvilinear length.
     ** @return  the configuration at the given curvilinear length.
     **/
    Config operator[](const double l) const; 
  }; // end of class ConstCurvArc

  Type path_type;              ///< The type of the path. 
  double maxi_curv;            ///< The maximum curvature. 
  double total_length;         ///< The total length of the path. 
  ConstCurvArc paths[nbArcs];  ///< The different arcs of the path.
  
  /// @brief The number of different configurations in a path.
  /// @see  fillArcs
  static const int nbConfig = nbArcs - 1;

  /// @brief Transform any arc number into a valid one
  ///        (too small -> 0, too big -> nbArcs - 1).
  /// @param part  the arc number.
  /// @return  The corrected arc number.
  /// @see arcLength, turnCenter
  static int validIndex(const int part) 
  { return part < 1 ? 0 : part >= nbConfig ? nbConfig : part; }
    
  /// @brief Fills the different arcs of the path.
  ///
  /// The starting configurations (except the first one, 
  ///  which is the path's starting configuration) are given.
  /// Final configuration of each arc is either the next arc's
  /// starting configuration or the path's final configuration.
  /// Curvature are found using the path type and the maximum 
  /// curvature. 
  ///
  /// @param start    the initial configuration, 
  /// @param end      the final configuration,
  /// @param configs  the intermediate starting configurations, 
  /// @param type     the type of the path, 
  /// @param lengths  the lengths of the arcs, 
  /// @param center   the turning centers of the arcs.
  void fillArcs(const Config& start, const Config& end,
		const Config* configs[nbConfig], const Type type, 
		const double lengths[nbArcs], 
		const Point* center[nbArcs]); 

  /// @brief Computes the different arcs of the path.
  ///
  /// The configurations are computed from the first one, the type
  /// and the deflections / length.
  /// Curvature are found using the path type and the maximum 
  /// curvature. 
  ///
  /// @param start    the initial configuration, 
  /// @param sgn      the signs of the curvatures,
  /// @param lengths  the deflections or length of the arcs, 
  /// @param center   the turning centers of the arcs.
  void computeArcs(const Config& start, const int sgn[nbArcs], 
		   const double defl_lngth[nbArcs], 
		   const Point* center[nbArcs]); 

  /// @brief Fills the arcs of path, when it does not move 
  ///        from the initial configuration.
  /// @param start    the initial configuration.
  void setNoMove(const Config& start) {
    const Config* cfgs[nbConfig] = {&start, &start}; 
    const double lgts[nbArcs]    = {0, 0, 0};
    const Point &start_pt = start.position(),
      *center_ptr[nbArcs] = {&start_pt, &start_pt, &start_pt}; 
    fillArcs(start, start, cfgs, lsl, lgts, center_ptr);
  } // end of void setNoMove(const Config&)
  
  /// @brief The default constructor is needed to build an array. 
  DubinsPath() : maxi_curv(1) {}
  
public:
  /** @brief The constructor needs the configurations to join
   **        and the maximum curvature. 
   ** @param start     the initial configuration,
   ** @param end       the final configuration,
   ** @param max_curv  the maximum curvature (in absolute value). 
   **/
  DubinsPath(const Config& start, const Config& end,
	     const double max_curv) : maxi_curv( fabs(max_curv) )
  { connect(start, end); } 
  
  // The default copy constructor should OK.

  /** @brief Connects two configurations. 
   ** @param start     the initial configuration,
   ** @param end       the final configuration. 
   **/
  void connect(const Config& start, const Config& end); 

  /// @brief Gives the path's type.  @return  the path's type. 
  const Type& type()  const { return path_type; }

  /// @brief Returns the initial configuration of the path.
  /// @return  the starting configuration.
  const Config& start() const { return paths[0].start; }

  /// @brief Returns the final configuration of the path.
  /// @return  the end configuration.
  const Config& end() const { return paths[nbConfig].end; }

  /// @brief Gives the path's maximum curvature. 
  /// @return    the path's maximum curvature. 
  const double& maxCurv()  const { return maxi_curv; }

  /// @brief Gives the path's total length. 
  /// @return    the path's total length. 
  const double& length()  const { return total_length; }

  /// @brief Returns the length of given arc. 
  /// @param part  the arc number (corrected if not valid).
  /// @return    the length of the asked arc. 
  /// @see validIndex
  const double& arcLength(const int idx) const 
  { return paths[ validIndex(idx) ].length; }

  /// @brief Returns the turning center of given arc. 
  /// @param part  the arc number (corrected if not valid).
  /// @return    the turning center of the asked arc. 
  /// @see validIndex
  const Point& turnCenter(const int idx) const 
  { return paths[ validIndex(idx) ].center; }
  
  /// @brief Writes the configuration's values in an output stream.
  /// @param O  the output stream in which the description is written.
  /// @see ostream::operator<<(const Object&)
  void writeTo(std::ostream& O) const {
    static const char *type_name[nbPossiblePaths]
      = {"lsl", "lsr", "rsl", "rsr", "lrl", "rlr"}; 
    O << start() << " -[" << maxCurv() << "]-> " << end()
      << ": " << type_name[type()] << ", ";
    for(int idx = 0; idx < nbArcs; idx++)
      O << arcLength(idx) << (idx < nbConfig ? " + " : ""); 
  } // end of void writeTo(std::ostream&) const
  
  /** @brief Finds the configuration at a given length along 
   **        the path.
   ** @param curv_lgth  the curvilinear length.
   ** @return  the configuration at the given curvilinear length.
   **/
  Config operator[](const double curv_lgth) const; 

}; // end of class DubinsPath

#endif // QTCTRL_DUBINS
