/*
 * Copyright (c) 2026 UC Berkeley
 * SPDX-License-Identifier: Apache-2.0
 *
 * Latest DroNet output, handed from the inference thread (rose_fc_dronet/src/dronet_thread.c) to the
 * control loop (ROSE_DRONET_NAV). Raw int8 outputs plus the time of the camera frame they were
 * computed from; the control loop dequantizes and applies its own staleness timeout
 * (fc_dronet_nav.h), so the timeout covers capture + inference + hand-off end to end.
 */
#ifndef ROSE_DRONET_CMD_H_
#define ROSE_DRONET_CMD_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct dronet_cmd {
	uint32_t seq;      /* publishes since boot (1 = first) */
	int64_t  t_ms;     /* k_uptime_get() when the source frame was captured */
	int8_t   steer_q;  /* output[0], linear1 */
	int8_t   coll_q;   /* output[1], sigmoid1 */
	bool     valid;    /* false after dronet_cmd_invalidate() */
};

void dronet_cmd_publish(int8_t steer_q, int8_t coll_q, int64_t t_capture_ms);

/* Withdraw the latest command at once (inference found corrupt), instead of letting it time out. */
void dronet_cmd_invalidate(void);

/* Copy of the latest command; false until the first publish or after an invalidate. */
bool dronet_cmd_latest(struct dronet_cmd *out);

#ifdef __cplusplus
}
#endif

#endif /* ROSE_DRONET_CMD_H_ */
