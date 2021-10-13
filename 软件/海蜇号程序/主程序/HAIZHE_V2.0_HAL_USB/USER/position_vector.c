#include "position_vector.h"
#include "wpnav.h"

//cd - centi degree: �Ƕ�*100
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

//����γ������ת��Ϊλ��ʸ��
void pv_location_to_vector(const Location loc, vec3f vec_neu){
	vec_neu[1] = (loc.lat - wp_nav.home.lat) * LATLON_TO_CM;	//γ�����1�㣬�������111.316666...km(���߳���/180)
	vec_neu[0] = (loc.lng - wp_nav.home.lng) * LATLON_TO_CM * longtitude_scale(loc);	//�������1�㣬����Ҫ����γ�ȵĳ߶� -> ���ϲ�ͬ�����ֱ��
	vec_neu[2] = loc.alt;
}

//��λ��ʸ��ת��Ϊ��γ������ ��loc�ľ�γ���ϵ���vec����
void pv_vector_to_location(const vec3f vec, Location *loc){
	(*loc).lat += vec[0]*M_TO_LATLON;
	(*loc).lng += vec[1]*M_TO_LATLON/longtitude_scale(*loc);
}






