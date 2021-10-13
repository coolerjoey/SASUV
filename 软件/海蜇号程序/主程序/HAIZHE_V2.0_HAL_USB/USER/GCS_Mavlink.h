#ifndef _GCS_MAVLINK_H
#define _GCS_MAVLINK_H

#include "mavlink.h"
#include "scheduler.h"
#include "GCS.h"

void gcs_send_heartbeat(void);
void gcs_data_stream_send(void);
void gcs_send_deferred(void);
void gcs_check_input(void);

void gcs_send_message(enum mav_message id);
void gcs_send_text(MAV_SEVERITY severity, const char *str);
void gcs_send_msg(void);
void gcs_send_allparams(void);


#endif

