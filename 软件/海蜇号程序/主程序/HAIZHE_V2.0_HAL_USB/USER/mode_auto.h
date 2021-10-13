#ifndef _MODE_AUTO_H
#define _MODE_AUTO_H
#include "sys.h"
#include "mission.h"

void auto_submerge_start(const Location dest_loc);
void auto_wp_start(const Location dest_lo);
void auto_loiter_start(const Mission_Command *cmd);
void auto_surface_start(void);
void auto_circle_start(const Mission_Command *cmd);


u8 auto_init(void);
void auto_run(void);

#endif

