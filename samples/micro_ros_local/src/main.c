/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Self-contained on-target ping-pong:
 *
 *   node_a:  timer @ 1 Hz  -> publishes /ping (Int32 incrementing counter)
 *            sub on /pong  -> prints "pong N" via printk on receive
 *
 *   node_b:  sub on /ping  -> prints "ping N" via printk on receive,
 *                             then publishes /pong (= ping+1) immediately
 *
 * No host agent. The target-resident broker (broker.c) routes WRITE_DATA
 * from each publisher to the matching subscribed datareader.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "transport_loopback.h"
#include "broker.h"

#define RCCHECK(rc) do { \
	rcl_ret_t _rc = (rc); \
	if (_rc != RCL_RET_OK) { \
		printk("rcl error %d at %s:%d\n", (int)_rc, __FILE__, __LINE__); \
		return; \
	} \
} while (0)

static rcl_publisher_t   pub_ping;     /* on node_a, topic /ping */
static rcl_publisher_t   pub_pong;     /* on node_b, topic /pong */
static std_msgs__msg__Int32 ping_out;
static std_msgs__msg__Int32 pong_out;
static std_msgs__msg__Int32 ping_in;   /* node_b's incoming ping */
static std_msgs__msg__Int32 pong_in;   /* node_a's incoming pong */

static void ping_timer_cb(rcl_timer_t *t, int64_t last_call_time)
{
	ARG_UNUSED(t); ARG_UNUSED(last_call_time);
	printk("[node_a] tick: send ping=%d\n", (int)ping_out.data);
	(void)rcl_publish(&pub_ping, &ping_out, NULL);
	ping_out.data++;
}

/* Runs on node_b when /ping arrives. Echoes back to /pong. */
static void on_ping_received(const void *msg)
{
	const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msg;
	printk("[node_b] recv ping=%d -> send pong=%d\n",
	       (int)m->data, (int)(m->data + 1));
	pong_out.data = m->data + 1;
	(void)rcl_publish(&pub_pong, &pong_out, NULL);
}

/* Runs on node_a when /pong arrives. */
static void on_pong_received(const void *msg)
{
	const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msg;
	printk("[node_a] recv pong=%d\n", (int)m->data);
}

static void uros_main(void)
{
	rmw_uros_set_custom_transport(false, NULL,
				      loopback_open, loopback_close,
				      loopback_write, loopback_read);
	broker_start();

	printk("micro_ros_local: pinging local broker...\n");
	while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
		k_msleep(50);
	}
	printk("micro_ros_local: broker reachable\n");

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t  support;
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	rcl_node_t node_a, node_b;
	RCCHECK(rclc_node_init_default(&node_a, "node_a", "", &support));
	RCCHECK(rclc_node_init_default(&node_b, "node_b", "", &support));

	const rosidl_message_type_support_t *int32_ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

	/* Publishers */
	RCCHECK(rclc_publisher_init_default(&pub_ping, &node_a, int32_ts, "/ping"));
	RCCHECK(rclc_publisher_init_default(&pub_pong, &node_b, int32_ts, "/pong"));

	/* Subscribers — node_b listens to /ping, node_a listens to /pong */
	rcl_subscription_t sub_ping_on_b, sub_pong_on_a;
	RCCHECK(rclc_subscription_init_default(&sub_ping_on_b, &node_b, int32_ts, "/ping"));
	RCCHECK(rclc_subscription_init_default(&sub_pong_on_a, &node_a, int32_ts, "/pong"));

	std_msgs__msg__Int32__init(&ping_out);
	std_msgs__msg__Int32__init(&pong_out);
	std_msgs__msg__Int32__init(&ping_in);
	std_msgs__msg__Int32__init(&pong_in);
	ping_out.data = 0;

	rcl_timer_t ping_timer;
	RCCHECK(rclc_timer_init_default2(&ping_timer, &support,
					 RCL_MS_TO_NS(1000), ping_timer_cb, true));

	/* Executor: 1 timer + 2 subscriptions = 3 handles. */
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &ping_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_ping_on_b, &ping_in,
					       on_ping_received, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &sub_pong_on_a, &pong_in,
					       on_pong_received, ON_NEW_DATA));

	const int64_t deadline_ms = k_uptime_get() + 8000;
	printk("micro_ros_local: ping-pong loop (8s)\n");
	while (k_uptime_get() < deadline_ms) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		k_msleep(10);
	}

	printk("micro_ros_local: shutting down (ping_out=%d)\n", (int)ping_out.data);

	(void)rcl_subscription_fini(&sub_ping_on_b, &node_b);
	(void)rcl_subscription_fini(&sub_pong_on_a, &node_a);
	(void)rcl_publisher_fini(&pub_ping, &node_a);
	(void)rcl_publisher_fini(&pub_pong, &node_b);
	(void)rcl_node_fini(&node_a);
	(void)rcl_node_fini(&node_b);
	(void)rclc_support_fini(&support);
}

int main(void)
{
	printk("*** micro_ros_local boot ***\n");
	uros_main();
	printk("*** micro_ros_local exit ***\n");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
