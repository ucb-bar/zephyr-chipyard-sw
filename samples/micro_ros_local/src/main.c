/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Two-node ping-pong on independent harts via the in-target broker.
 *
 *   node_a:  pinned to hart 0
 *            timer @ 1 Hz publishes /ping (Int32 incrementing counter)
 *            sub on /pong  -> prints "pong N"
 *
 *   node_b:  pinned to hart 1
 *            sub on /ping  -> prints "ping N", publishes pong=ping+1
 *
 *   broker:  pinned to hart 2 — single thread serving both sessions via
 *            k_poll over their loopback queues
 *
 * Each node owns its own rclc_support_t / uxrSession / loopback queue pair
 * (transport->args carries the session index 0 or 1). That keeps each
 * session single-threaded internally — required because the unsynchronized
 * rmw_microxrcedds_c session would race if two cores called rcl_publish
 * into the same session. Cross-session pub/sub is routed by the broker
 * via topic-name matching.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rcl/init_options.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>
#include <std_msgs/msg/int32.h>

#include <microros_local/transport_loopback.h>
#include <microros_local/broker.h>

#define RCCHECK(rc) do { \
	rcl_ret_t _rc = (rc); \
	if (_rc != RCL_RET_OK) { \
		printk("rcl error %d at %s:%d\n", (int)_rc, __FILE__, __LINE__); \
		return; \
	} \
} while (0)

/* Per-node state lives in static storage so the threads can share it for
 * the cross-callback pieces (publishers + buffers used in subscription
 * callbacks). Each thread only ever touches its own struct. */
struct node_ctx {
	int                  session_idx;     /* loopback queue index */
	int                  cpu;             /* hart we pin this executor to */
	const char          *node_name;
	rcl_publisher_t      pub;             /* outgoing publisher */
	const char          *pub_topic;
	rcl_subscription_t   sub;
	const char          *sub_topic;
	std_msgs__msg__Int32 sub_msg;         /* destination buffer for sub */
	std_msgs__msg__Int32 pub_msg;         /* outgoing message scratch */
	void               (*on_msg)(struct node_ctx *, int32_t value);
	uint64_t             timer_period_ms; /* 0 = no timer */
	void               (*on_tick)(struct node_ctx *);
};

static struct node_ctx ctx_a;
static struct node_ctx ctx_b;

#define EXECUTOR_STACK_SIZE 16384
K_THREAD_STACK_DEFINE(stack_a, EXECUTOR_STACK_SIZE);
K_THREAD_STACK_DEFINE(stack_b, EXECUTOR_STACK_SIZE);
static struct k_thread thread_a, thread_b;

/* ----- node_a behavior ---- */

static void node_a_tick(struct node_ctx *self)
{
	printk("[node_a hart=%d] tick: send ping=%d\n",
	       arch_proc_id(), (int)self->pub_msg.data);
	(void)rcl_publish(&self->pub, &self->pub_msg, NULL);
	self->pub_msg.data++;
}

static void node_a_on_pong(struct node_ctx *self, int32_t value)
{
	(void)self;
	printk("[node_a hart=%d] recv pong=%d\n", arch_proc_id(), (int)value);
}

/* ----- node_b behavior ---- */

static void node_b_on_ping(struct node_ctx *self, int32_t value)
{
	int32_t reply = value + 1;
	printk("[node_b hart=%d] recv ping=%d -> send pong=%d\n",
	       arch_proc_id(), (int)value, (int)reply);
	self->pub_msg.data = reply;
	(void)rcl_publish(&self->pub, &self->pub_msg, NULL);
}

/* ----- shared callback shims (rclc passes void* msg + we recover ctx) ---- */

static void sub_trampoline_a(const void *msg)
{
	const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msg;
	if (ctx_a.on_msg) {
		ctx_a.on_msg(&ctx_a, m->data);
	}
}

static void sub_trampoline_b(const void *msg)
{
	const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msg;
	if (ctx_b.on_msg) {
		ctx_b.on_msg(&ctx_b, m->data);
	}
}

static void timer_trampoline_a(rcl_timer_t *t, int64_t last_call_time)
{
	ARG_UNUSED(t); ARG_UNUSED(last_call_time);
	if (ctx_a.on_tick) {
		ctx_a.on_tick(&ctx_a);
	}
}

/* ----- the per-node executor entry point (runs pinned on each hart) ---- */

static void node_thread_fn(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg2); ARG_UNUSED(arg3);
	struct node_ctx *ctx = (struct node_ctx *)arg1;

	printk("[%s] starting on hart %d, session_idx=%d\n",
	       ctx->node_name, arch_proc_id(), ctx->session_idx);

	rcl_allocator_t allocator = rcl_get_default_allocator();

	/* Per-init-options custom transport. The non-options variant
	 * (rmw_uros_set_custom_transport) writes to a single global
	 * `rmw_uxrce_transport_default_params` — incompatible with two
	 * concurrently-initializing sessions; the second call clobbers the
	 * first's args.  The options-based variant stores per-context state. */
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
		printk("[%s] rcl_init_options_init failed\n", ctx->node_name);
		return;
	}
	rmw_init_options_t *rmw_options =
		rcl_init_options_get_rmw_init_options(&init_options);
	if (!rmw_options) {
		printk("[%s] rcl_init_options_get_rmw_init_options failed\n",
		       ctx->node_name);
		return;
	}
	if (rmw_uros_options_set_custom_transport(false,
				      (void *)(intptr_t)ctx->session_idx,
				      loopback_open, loopback_close,
				      loopback_write, loopback_read,
				      rmw_options) != RMW_RET_OK) {
		printk("[%s] options_set_custom_transport failed\n",
		       ctx->node_name);
		return;
	}

	rclc_support_t  support;
	rcl_ret_t rc = rclc_support_init_with_options(
		&support, 0, NULL, &init_options, &allocator);
	printk("[%s] rclc_support_init_with_options rc=%d\n",
	       ctx->node_name, (int)rc);
	if (rc != RCL_RET_OK) {
		return;
	}

	rcl_node_t node;
	rc = rclc_node_init_default(&node, ctx->node_name, "", &support);
	printk("[%s] rclc_node_init_default rc=%d\n", ctx->node_name, (int)rc);
	if (rc != RCL_RET_OK) {
		return;
	}

	const rosidl_message_type_support_t *int32_ts =
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

	if (rclc_publisher_init_default(&ctx->pub, &node, int32_ts,
					ctx->pub_topic) != RCL_RET_OK) {
		printk("[%s] pub init failed\n", ctx->node_name);
		return;
	}
	if (rclc_subscription_init_default(&ctx->sub, &node, int32_ts,
					   ctx->sub_topic) != RCL_RET_OK) {
		printk("[%s] sub init failed\n", ctx->node_name);
		return;
	}

	std_msgs__msg__Int32__init(&ctx->sub_msg);
	std_msgs__msg__Int32__init(&ctx->pub_msg);
	ctx->pub_msg.data = 0;

	rcl_timer_t timer;
	bool have_timer = (ctx->timer_period_ms > 0);
	if (have_timer) {
		if (rclc_timer_init_default2(
			    &timer, &support,
			    RCL_MS_TO_NS(ctx->timer_period_ms),
			    timer_trampoline_a, true) != RCL_RET_OK) {
			printk("[%s] timer init failed\n", ctx->node_name);
			return;
		}
	}

	rclc_executor_t executor;
	const size_t handles = (have_timer ? 1u : 0u) + 1u; /* + sub */
	if (rclc_executor_init(&executor, &support.context, handles, &allocator)
	    != RCL_RET_OK) {
		printk("[%s] executor init failed\n", ctx->node_name);
		return;
	}
	if (have_timer) {
		(void)rclc_executor_add_timer(&executor, &timer);
	}
	(void)rclc_executor_add_subscription(
		&executor, &ctx->sub, &ctx->sub_msg,
		(ctx == &ctx_a) ? sub_trampoline_a : sub_trampoline_b,
		ON_NEW_DATA);

	const int64_t deadline_ms = k_uptime_get() + 8000;
	printk("[%s] spinning (8s)\n", ctx->node_name);
	while (k_uptime_get() < deadline_ms) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		k_msleep(10);
	}
	printk("[%s] done (pub_data=%d)\n", ctx->node_name, (int)ctx->pub_msg.data);

	(void)rcl_subscription_fini(&ctx->sub, &node);
	(void)rcl_publisher_fini(&ctx->pub, &node);
	(void)rcl_node_fini(&node);
	(void)rclc_support_fini(&support);
}

int main(void)
{
	printk("*** micro_ros_local boot (multicore) ***\n");

	/* Configure the two node contexts. */
	ctx_a.session_idx = 0;
	ctx_a.cpu = 0;
	ctx_a.node_name = "node_a";
	ctx_a.pub_topic = "/ping";
	ctx_a.sub_topic = "/pong";
	ctx_a.on_msg = node_a_on_pong;
	ctx_a.timer_period_ms = 1000;
	ctx_a.on_tick = node_a_tick;

	ctx_b.session_idx = 1;
	ctx_b.cpu = 1;
	ctx_b.node_name = "node_b";
	ctx_b.pub_topic = "/pong";
	ctx_b.sub_topic = "/ping";
	ctx_b.on_msg = node_b_on_ping;
	ctx_b.timer_period_ms = 0;       /* node_b is sub-driven only */

	/* Broker on hart 2, off the harts running the executors. */
	broker_start_pinned(2);

	/* Spawn the two executor threads suspended, pin, then start.
	 * Pin order matters under CONFIG_SCHED_CPU_MASK_PIN_ONLY=y — pinning
	 * a runnable thread asserts. Same pattern as samples/test_mt_rvv. */
	k_tid_t tid_a = k_thread_create(&thread_a, stack_a, K_THREAD_STACK_SIZEOF(stack_a),
					node_thread_fn, &ctx_a, NULL, NULL,
					K_PRIO_PREEMPT(1), 0, K_FOREVER);
	k_thread_name_set(tid_a, "node_a");
	k_thread_cpu_pin(tid_a, ctx_a.cpu);

	k_tid_t tid_b = k_thread_create(&thread_b, stack_b, K_THREAD_STACK_SIZEOF(stack_b),
					node_thread_fn, &ctx_b, NULL, NULL,
					K_PRIO_PREEMPT(1), 0, K_FOREVER);
	k_thread_name_set(tid_b, "node_b");
	k_thread_cpu_pin(tid_b, ctx_b.cpu);

	k_thread_start(tid_a);
	k_thread_start(tid_b);

	/* Wait for both to finish, then exit cleanly via HTIF. */
	k_thread_join(tid_a, K_FOREVER);
	k_thread_join(tid_b, K_FOREVER);

	printk("*** micro_ros_local exit ***\n");
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
