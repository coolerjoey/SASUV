#include "position_vector.h"
#include "wpnav.h"

//cd - centi degree: 角度*100
float pv_get_bearing_cd(const vec3f origin, const vec3f destination){
	float bearing = 9000 + atan2f(-(destination[1]-origin[1]), destination[0]-origin[0]) * 18000/M_PI;	
	if(bearing<0) bearing+=36000;
	return bearing;
}

float pv_get_horizontal_distance_cm(const vec3f origin, const vec3f destination){
	float dx = origin[0] - destination[0];
	float dy = origin[1] - destination[1];
	return sqrt(dx*dx+dy*dy);
}

float longtitude_scale(const Location loc){
	float scale = cosf(loc.lat * 1e-7f * DEG_TO_RAD);
	return constrain_float(scale, 0.01f, 1.0f);
}

//将经纬度坐标转化为位置矢量
void pv_location_to_vector(const Location loc, vec3f vec_neu){
	vec_neu[1] = (loc.lat - wp_nav.home.lat) * LATLON_TO_CM;	//纬度相差1°，距离相差111.316666...km(经线长度/180)
	vec_neu[0] = (loc.lng - wp_nav.home.lng) * LATLON_TO_CM * longtitude_scale(loc);	//经线相差1°，还需要考虑纬度的尺度 -> 球上不同截面的直径
	vec_neu[2] = loc.alt;
}

//将位置矢量转化为经纬度坐标 在loc的经纬度上叠加vec向量
void pv_vector_to_location(const vec3f vec, Location *loc){
	(*loc).lat += vec[0]*M_TO_LATLON;
	(*loc).lng += vec[1]*M_TO_LATLON/longtitude_scale(*loc);
}






