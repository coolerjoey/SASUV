#ifndef __WPNAV_H
#define __WPNAV_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include <sys.h>
#include "defines.h"
#include "mymath.h"
#include "mission.h"

typedef struct{
	bool reached_destination	: 1;
	bool fast_waypoint			: 1;
	bool slowing_down			: 1;
	bool recalc_wp_leash		: 1;
}FLAG;


typedef struct{
	FLAG flag;
	Location home;	//返航点
	Location destination; //下个航点
	Location origin;		//正在进行的航迹的起始航点
}WP_NAV;
extern WP_NAV wp_nav;


bool wpnav_set_wp_destination(const Location *destination);
float wp_nav_get_wp_distance_to_destination(void);




#ifdef __cplusplus
}
#endif
   
#endif /*__LOGGER_H*/
