#ifndef __CONSOLE_H
#define __CONSOLE_H

#include "sys.h"
#include "scheduler.h"

typedef  void (*pFunction)(void);

typedef struct{
	char command[10];
	char *desciption;
	void (*Function)(char *argv) ;
}COMMAND_LIST;

void console_init(const Task *tasks,u8 num_tasks);
void console_debug(void);
static void handle_lscmd(char *argv);
static void handle_lstt(char *argv);
static void handle_lsll(char *argv);
static void handle_lsstatus(char *argv);
static void handle_lsadc(char *argv);
static void handle_lsbaro(char *argv);
static void handle_lspwr(char *argv);
static void handle_lsang(char *argv);
static void handle_setECatt(char *argv);
static void handle_lsgyro(char *argv);
static void handle_lsacc(char *argv);
static void handle_setfreq(char *argv);
static void handle_setbaud(char *argv);
static void handle_calacc(char *argv);
static void handle_lsgps(char *argv);
static void handle_lstemp(char *argv);
static void handle_lsrc(char *argv);
static void handle_lsppm(char *argv);
static void handle_lspwm(char *argv);
static void handle_lstime(char *argv);
static void handle_settime(char *argv);
static void handle_lsjoy(char *argv);
static void handle_reset(char *argv);
static void handle_powerup(char *argv);
static void handle_powerdown(char *argv);
static void handle_arm(char *argv);
static void handle_disarm(char *argv);
static void handle_MANUAL(char *argv);
static void handle_STABILIZE(char *argv);
static void handle_AUTO(char *argv);
static void handle_ALT_HOLD(char *argv);
static void handle_sdread(char *argv);
static void handle_paraset(char *argv);
static void handle_lswpcnt(char *argv);
static void handle_lswp(char *argv);
static void handle_lsdatesize(char *argv);
static void handle_lsparams(char *argv);
static void handle_setparam(char *argv);
static void handle_showmavlink(char *argv);
static void handle_lscomm(char *argv);
static void handle_ttpwm(char *argv);
static void handle_stpwm(char *argv);
static void handle_send_allparams(char *argv);
static void handle_ls_sensorparse(char *argv);
static void handle_setheading(char *argv);
static void handle_lssramin(char *argv);
static void handle_parasave(char *argv);
static void handle_lssocsta(char *argv);
static void handle_showgps(char *argv);
static void handle_kfverify(char *argv);
static void handle_kftest(char *argv);
static void handle_DCM_update(char *argv);
static void handle_showpass(char *argv);
static void handle_showwp(char *argv);
static void handle_modelver(char *argv);
static void handle_modeltest(char *argv);
static void handle_pwm2force(char *argv);
static void handle_force2pwm(char *argv);
static void handle_runtest(char *argv);
static void handle_showUART(char *argv);
static void handle_UARTTC(char *argv);


#endif


