#ifndef _COMMANDS_LOGIC_H
#define _COMMANDS_LOGIC_H
#include "sys.h"
#include "mission.h"

bool verify_command(const Mission_Command *cmd);
bool start_command(const Mission_Command *cmd);
void exit_mission(void);


#endif

