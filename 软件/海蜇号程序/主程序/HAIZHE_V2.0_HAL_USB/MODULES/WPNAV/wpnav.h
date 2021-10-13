#ifndef __WPNAV_H
#define __WPNAV_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include <sys.h>
#include "defines.h"
#include "mymath.h"
#include "mission.h"






bool wpnav_set_wp_destination(const Location *destination);
float wp_nav_get_wp_distance_to_destination(void);
void DR(Location *pos, vec3f acc_b, vec3f acc_n, vec3f v, vec3f att_rad,float dt);




#ifdef __cplusplus
}
#endif
   
#endif /*__LOGGER_H*/
