/**
 ** @file  src/model/object.cpp
 **
 ** @brief Static mathematical methods.
 **
 ** @date  November 2018
 **/

#include <model/object.hpp>

/* A small value under which doubles are considered as zero.
 * 
 * Double values should never be compared to zero, but considered 
 * as zero if and only if their absolute value is smaller than 
 * this value. 
 */ 
const double Object::smallDouble = 1E-6; 
