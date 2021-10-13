#ifndef _POSITION_VECTOR_H
#define _POSITION_VECTOR_H

#include "sys.h"
#include "mymath.h"
#include "mission.h"

float pv_get_bearing_cd(const vec3f origin, const vec3f destination);
float pv_get_horizontal_distance_cm(const vec3f origin, const vec3f destination);
void pv_location_to_vector(const Location loc, vec3f vec_neu);
void pv_vector_to_location(const vec3f vec, Location *loc);

#endif

