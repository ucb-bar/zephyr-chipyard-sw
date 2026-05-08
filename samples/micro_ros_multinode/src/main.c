/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two-node smoke test for micro-ROS on spike_riscv64 / chipyard_riscv64.
 *
 * Spawns two nodes, each with one Int32 publisher fed by a periodic timer.
 * Talks to a ros-jazzy-micro-ros-agent on the host via the HDLC-on-HTIF
 * custom transport (transport_htif.{c,h}). The agent connection is brokered
 * by tools/microros/htif_proxy.py (see tools/microros/run_with_agent.sh).
 *
 *   /node_a/counter   std_msgs/Int32   1 Hz
 *   /node_b/counter   std_msgs/Int32   2 Hz
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "transport_htif.h"

#define RCCHECK(rc) do { \
	rcl_ret_t _rc = (rc); \
	if (_rc != RCL_RET_OK) { \
		printk("rcl error %d at %s:%d\n", (int)_rc, __FILE__, __LINE__); \
		return; \
	} \
} while (0)

static rcl_publisher_t pub_a;
static rcl_publisher_t pub_b;
static std_msgs__msg__Int32 msg_a;
static std_msgs__msg__Int32 msg_b;

static void timer_a_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	ARG_UNUSED(timer); ARG_UNUSED(last_call_time);
	rcl_ret_t rc = rcl_publish(&pub_a, &msg_a, NULL);
	if (rc != RCL_RET_OK) {
		printk("publish a failed: %d\n", (int)rc);
	}
	msg_a.data++;
}

static void timer_b_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	ARG_UNUSED(timer); ARG_UNUSED(last_call_time);
	rcl_ret_t rc = rcl_publish(&pub_b, &msg_b, NULL);
	if (rc != RCL_RET_OK) {
		printk("publish b failed: %d\n", (int)rc);
	}
	msg_b.data += 10;   /* easy visual distinction from a's stream */
}

static void uros_main(void)
{
	/* 1. Wire the custom transport before anything else touches rmw. */
	rmw_uros_set_custom_transport(
		true,    /* MICRO_ROS_FRAMING_REQUIRED — keeps message reassembly
			  * intact even if our HDLC frames split a multi-byte msg
			  * (we don't, today, but it's robust to add). */
		NULL,
		htif_transport_open,
		htif_transport_close,
		htif_transport_write,
		htif_transport_read);

	/* 2. Wait for the agent to come up. We block forever — that matches
	 *    the post-boot sequencing the host script uses (agent first,
	 *    then it'll feed pings through). */
	printk("micro_ros_multinode: pinging agent...\n");
	while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
		k_msleep(200);
	}
	printk("micro_ros_multinode: agent reachable\n");

	/* 3. Standard rclc bring-up. */
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t  support;
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	rcl_node_t node_a, node_b;
	RCCHECK(rclc_node_init_default(&node_a, "node_a", "", &support));
	RCCHECK(rclc_node_init_default(&node_b, "node_b", "", &support));

	const rosidl_message_type_support_t *int32_ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

	RCCHECK(rclc_publisher_init_default(&pub_a, &node_a, int32_ts, "/node_a/counter"));
	RCCHECK(rclc_publisher_init_default(&pub_b, &node_b, int32_ts, "/node_b/counter"));

	std_msgs__msg__Int32__init(&msg_a);
	std_msgs__msg__Int32__init(&msg_b);
	msg_a.data = 0;
	msg_b.data = 0;

	rcl_timer_t timer_a, timer_b;
	RCCHECK(rclc_timer_init_default2(&timer_a, &support,
					RCL_MS_TO_NS(1000), timer_a_cb, true));
	RCCHECK(rclc_timer_init_default2(&timer_b, &support,
					RCL_MS_TO_NS(500),  timer_b_cb, true));

	/* 4. Executor for two timers. */
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_a));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_b));

	/* Run for a bounded duration so the sim can self-terminate via the
	 * HTIF exit command. Long enough to see ros2 topic echo cycle through
	 * many publishes, short enough that CI-style runs don't hang. */
	const int64_t deadline_ms = k_uptime_get() + 30000;
	printk("micro_ros_multinode: entering executor loop (30s)\n");
	while (k_uptime_get() < deadline_ms) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		k_msleep(10);
	}

	printk("micro_ros_multinode: shutting down\n");
	(void)rcl_publisher_fini(&pub_a, &node_a);
	(void)rcl_publisher_fini(&pub_b, &node_b);
	(void)rcl_node_fini(&node_a);
	(void)rcl_node_fini(&node_b);
	(void)rclc_support_fini(&support);
}

int main(void)
{
	printk("*** micro_ros_multinode boot ***\n");
	uros_main();
	printk("*** micro_ros_multinode exit — issuing HTIF shutdown ***\n");
	/* Sends the HTIF exit command, which fesvr handles by terminating the
	 * sim (instead of relying on a host-side timeout to kill spike). */
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
