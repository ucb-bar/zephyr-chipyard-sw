/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Self-contained two-node micro-ROS sample: target-resident broker, no
 * host-side agent required. Spike runs the full sample and exits cleanly.
 *
 * For step 3 (stub broker): the executor / nodes / publishers are created
 * successfully and the executor loop runs. rcl_publish() calls succeed but
 * the broker drops WRITE_DATA — no inter-node delivery yet. Step 4 adds the
 * routing so node_a's pub reaches node_b's sub callback.
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

static rcl_publisher_t pub_a;
static rcl_publisher_t pub_b;
static std_msgs__msg__Int32 msg_a;
static std_msgs__msg__Int32 msg_b;

static void timer_a_cb(rcl_timer_t *t, int64_t last_call_time)
{
	ARG_UNUSED(t); ARG_UNUSED(last_call_time);
	(void)rcl_publish(&pub_a, &msg_a, NULL);
	msg_a.data++;
}

static void timer_b_cb(rcl_timer_t *t, int64_t last_call_time)
{
	ARG_UNUSED(t); ARG_UNUSED(last_call_time);
	(void)rcl_publish(&pub_b, &msg_b, NULL);
	msg_b.data += 10;
}

static void uros_main(void)
{
	/* Wire the loopback custom transport. framing=false because k_msgq
	 * preserves message boundaries — no need for the stream-framing
	 * wrapper that targets serial-like transports. */
	rmw_uros_set_custom_transport(
		false,
		NULL,
		loopback_open,
		loopback_close,
		loopback_write,
		loopback_read);

	/* Launch the in-target broker thread. */
	broker_start();

	/* Same micro-ROS bring-up as before — but the agent it talks to is
	 * the broker thread we just launched, on a queue. */
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

	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_a));
	RCCHECK(rclc_executor_add_timer(&executor, &timer_b));

	const int64_t deadline_ms = k_uptime_get() + 10000;
	printk("micro_ros_local: executor running (10s)\n");
	while (k_uptime_get() < deadline_ms) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		k_msleep(10);
	}
	printk("micro_ros_local: shutting down (a=%d b=%d)\n",
	       (int)msg_a.data, (int)msg_b.data);

	(void)rcl_publisher_fini(&pub_a, &node_a);
	(void)rcl_publisher_fini(&pub_b, &node_b);
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
