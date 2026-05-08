/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Target-resident XRCE-DDS broker (stub).
 *
 * Drains messages from the loopback transport's rmw->broker queue, parses
 * each XRCE-DDS message header + submessage(s), and emits the minimum
 * responses needed to keep rmw_microxrcedds happy:
 *
 *   CREATE_CLIENT (0)  -> STATUS_AGENT (4)  with OK
 *   CREATE        (1)  -> STATUS        (5)  with OK
 *   GET_INFO      (2)  -> INFO          (6)  with OK
 *   DELETE        (3)  -> STATUS        (5)  with OK
 *   WRITE_DATA    (7)  -> dropped (routing comes in step 4)
 *   READ_DATA     (8)  -> ignored
 *   ACKNACK       (10) -> ignored (we always send best-effort)
 *   HEARTBEAT     (11) -> ignored
 *
 * This is enough to get rclc_publisher_init_default() / executor entry to
 * succeed without an actual host-side agent. Real WRITE_DATA -> DATA fanout
 * is the next step.
 */

#include "broker.h"
#include "transport_loopback.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include <ucdr/microcdr.h>

#define BROKER_THREAD_STACK 4096
#define BROKER_THREAD_PRIO  6

/* Constants from the upstream protocol headers — duplicated locally so the
 * broker stays decoupled from libmicroros internals (those headers are
 * `_internal` and not part of the public API surface). */
enum {
	SUB_CREATE_CLIENT   = 0,
	SUB_CREATE          = 1,
	SUB_GET_INFO        = 2,
	SUB_DELETE          = 3,
	SUB_STATUS_AGENT    = 4,
	SUB_STATUS          = 5,
	SUB_INFO            = 6,
	SUB_WRITE_DATA      = 7,
	SUB_READ_DATA       = 8,
	SUB_DATA            = 9,
	SUB_ACKNACK         = 10,
	SUB_HEARTBEAT       = 11,
};

#define FLAG_ENDIANNESS  0x01
#define FLAG_FORMAT_DATA 0x00

#define SESSION_NO_KEY   0x80   /* SESSION_ID_WITHOUT_CLIENT_KEY */
#define HEADER_NO_KEY    4
#define HEADER_WITH_KEY  8

#define STATUS_OK         0x00
#define STATUS_OK_MATCHED 0x01

/* Stream classes (matches client/core/session/stream/stream_id.c). */
#define STREAM_THRESH_BE  1
#define STREAM_THRESH_REL 128

/* XRCE protocol cookie + version, copied from xrce_types.h. The agent
 * representation in STATUS_AGENT carries these. */
static const uint8_t XRCE_COOKIE[4] = {0x58, 0x52, 0x43, 0x45};  /* "XRCE" */
static const uint8_t XRCE_VERSION[2] = {0x01, 0x00};
static const uint8_t XRCE_VENDOR_EPROSIMA[2] = {0x01, 0x0F};

K_THREAD_STACK_DEFINE(broker_stack, BROKER_THREAD_STACK);
static struct k_thread broker_thread_data;

/* Per-session state — we have at most one client. */
struct session_state {
	bool     active;
	uint8_t  session_id;
	uint8_t  key[4];
	/* One seq counter per output-stream class. We don't model multiple
	 * indices because rmw_microxrcedds_c only uses one of each. */
	uint16_t seq_none;
	uint16_t seq_be;
	uint16_t seq_rel;
};
static struct session_state sess;

/* Header values from the message currently being processed. Replies echo
 * these so pre-session pings (session_id=0x80, no key) get pre-session
 * replies and post-session traffic gets the established session header. */
static uint8_t  cur_session_id;
static uint8_t  cur_key[4];
static uint8_t  cur_in_stream_id;

/* ----- Helpers ----- */

static inline uint8_t pick_endian_flag(void)
{
	uint16_t test = 1;
	return (*(uint8_t *)&test == 1) ? FLAG_ENDIANNESS : 0;
}

static void put_msg_header(ucdrBuffer *ub, uint8_t session_id, uint8_t stream_id,
			   uint16_t seq, const uint8_t *key)
{
	ucdr_serialize_uint8_t(ub, session_id);
	ucdr_serialize_uint8_t(ub, stream_id);
	ucdr_serialize_endian_uint16_t(ub, UCDR_LITTLE_ENDIANNESS, seq);
	if (session_id < SESSION_NO_KEY) {
		ucdr_serialize_array_uint8_t(ub, key, 4);
	}
}

/* Bump and return the appropriate per-stream seq counter. */
static uint16_t next_seq_for(uint8_t stream_id)
{
	if (stream_id == 0) {
		return sess.seq_none++;
	}
	if (stream_id < STREAM_THRESH_REL) {
		return sess.seq_be++;
	}
	return sess.seq_rel++;
}

static void put_subheader(ucdrBuffer *ub, uint8_t id, uint8_t flags, uint16_t length)
{
	/* Subheader is 4-byte aligned within the message buffer. */
	size_t off = ucdr_buffer_length(ub);
	while (off % 4 != 0) {
		ucdr_serialize_uint8_t(ub, 0);
		off++;
	}
	ucdr_serialize_uint8_t(ub, id);
	ucdr_serialize_uint8_t(ub, flags);
	ucdr_serialize_endian_uint16_t(ub, UCDR_LITTLE_ENDIANNESS, length);
}

/* ----- Reply builders ----- */

static void send_status_agent(uint16_t request_id, uint8_t status_value)
{
	struct loopback_slot s;
	ucdrBuffer ub;
	ucdr_init_buffer(&ub, s.data, sizeof(s.data));

	/* STATUS_AGENT must arrive on the NONE stream (the client's read path
	 * for STATUS_AGENT requires stream_id.type == UXR_NONE_STREAM). */
	const uint8_t reply_stream = 0;
	put_msg_header(&ub, cur_session_id, reply_stream, next_seq_for(reply_stream), cur_key);

	/* STATUS_AGENT payload:
	 *   ResultStatus (2): status, implementation_status
	 *   AGENT_Representation: cookie(4), version(2), vendor(2), optional_props(1)
	 */
	const uint16_t payload_len = 2 + 4 + 2 + 2 + 1;
	uint8_t flags = pick_endian_flag();
	put_subheader(&ub, SUB_STATUS_AGENT, flags, payload_len);

	/* ResultStatus */
	ucdr_serialize_uint8_t(&ub, status_value);
	ucdr_serialize_uint8_t(&ub, 0);   /* implementation_status */

	/* AGENT_Representation */
	ucdr_serialize_array_uint8_t(&ub, XRCE_COOKIE, 4);
	ucdr_serialize_array_uint8_t(&ub, XRCE_VERSION, 2);
	ucdr_serialize_array_uint8_t(&ub, XRCE_VENDOR_EPROSIMA, 2);
	ucdr_serialize_uint8_t(&ub, 0);   /* optional_properties = false */

	(void)request_id;  /* not echoed in STATUS_AGENT */

	s.len = (uint16_t)ucdr_buffer_length(&ub);
	(void)loopback_broker_send(&s);
}

static void send_status(uint16_t related_request_id, const uint8_t object_id[2],
			uint8_t status_value)
{
	struct loopback_slot s;
	ucdrBuffer ub;
	ucdr_init_buffer(&ub, s.data, sizeof(s.data));

	/* For non-DELETE_SESSION STATUS, anything except NONE stream is fine.
	 * Mirror the inbound stream so reliable requests get reliable replies. */
	const uint8_t reply_stream = cur_in_stream_id ? cur_in_stream_id : 0x80;
	put_msg_header(&ub, cur_session_id, reply_stream, next_seq_for(reply_stream), cur_key);

	/* STATUS payload: BaseObjectReply
	 *   related_request: RequestId(2) + ObjectId(2) = 4
	 *   result:          ResultStatus(2)
	 * = 6
	 */
	const uint16_t payload_len = 6;
	uint8_t flags = pick_endian_flag();
	put_subheader(&ub, SUB_STATUS, flags, payload_len);

	/* RequestId is 2 raw bytes; in BaseObjectRequest the on-wire layout is
	 * { request_id[2], object_id[2] } big/little-endian-agnostic byte arrays. */
	ucdr_serialize_endian_uint16_t(&ub, UCDR_BIG_ENDIANNESS, related_request_id);
	ucdr_serialize_array_uint8_t(&ub, object_id, 2);
	ucdr_serialize_uint8_t(&ub, status_value);
	ucdr_serialize_uint8_t(&ub, 0);   /* implementation_status */

	s.len = (uint16_t)ucdr_buffer_length(&ub);
	(void)loopback_broker_send(&s);
}

static void send_info(uint16_t related_request_id, const uint8_t object_id[2])
{
	struct loopback_slot s;
	ucdrBuffer ub;
	ucdr_init_buffer(&ub, s.data, sizeof(s.data));

	const uint8_t reply_stream = 0;   /* INFO matches GET_INFO stream (NONE) */
	put_msg_header(&ub, cur_session_id, reply_stream, next_seq_for(reply_stream), cur_key);

	/* INFO payload — must satisfy uxr_acknack_pong / read_submessage_info:
	 *   BaseObjectReply: request_id(2) + object_id(2) + result(2)        = 6
	 *   optional_config:   0 (false)                                     = 1
	 *   optional_activity: 1 (true)                                      = 1
	 *   activity.kind:     0x0D (DDS_XRCE_OBJK_AGENT)                    = 1
	 *   availability:      int16_t > 0                                   = 2
	 *                                                                   ----
	 *                                                                    11
	 *
	 * The endianness flag in the subheader is mandatory — the client uses
	 * it to pick the right deserializer for the int16_t. */
	const uint16_t payload_len = 11;
	uint8_t flags = pick_endian_flag();
	put_subheader(&ub, SUB_INFO, flags, payload_len);

	/* BaseObjectReply */
	ucdr_serialize_endian_uint16_t(&ub, UCDR_BIG_ENDIANNESS, related_request_id);
	ucdr_serialize_array_uint8_t(&ub, object_id, 2);
	ucdr_serialize_uint8_t(&ub, STATUS_OK);
	ucdr_serialize_uint8_t(&ub, 0);                         /* impl status */

	/* ObjectInfo */
	ucdr_serialize_uint8_t(&ub, 0);                         /* optional_config = false */
	ucdr_serialize_uint8_t(&ub, 1);                         /* optional_activity = true */
	ucdr_serialize_uint8_t(&ub, 0x0D);                      /* kind = AGENT */
	ucdr_serialize_endian_int16_t(&ub,
		(flags & FLAG_ENDIANNESS) ? UCDR_LITTLE_ENDIANNESS : UCDR_BIG_ENDIANNESS,
		1);                                             /* availability > 0 */

	s.len = (uint16_t)ucdr_buffer_length(&ub);
	(void)loopback_broker_send(&s);
}

/* ----- Inbound submessage handling ----- */

/* Parse the BaseObjectRequest at the head of CREATE/DELETE/WRITE_DATA payloads.
 * Returns the (request_id, object_id) by value. */
struct base_request {
	uint16_t request_id;
	uint8_t  object_id[2];
};

static struct base_request read_base_request(ucdrBuffer *ub)
{
	struct base_request b = {0};
	ucdr_deserialize_endian_uint16_t(ub, UCDR_BIG_ENDIANNESS, &b.request_id);
	ucdr_deserialize_array_uint8_t(ub, b.object_id, 2);
	return b;
}

static void handle_create_client(ucdrBuffer *ub, uint8_t session_id_masked,
				 const uint8_t key[4], uint16_t length)
{
	(void)length;
	/* The CREATE_CLIENT submessage payload is a CLIENT_Representation:
	 *   XrceCookie(4) XrceVersion(2) XrceVendorId(2) ClientKey(4)
	 *   session_id(1) optional_properties(1) [PropertySeq] mtu(2)
	 *
	 * The session_id INSIDE the payload is the real info->id (e.g. 0x81 for
	 * a session with reliable streams). The session_id in the message header
	 * is `info->id & 0x80` (masked for the create-session bootstrap). The
	 * client's read_session_header() insists subsequent replies use the
	 * un-masked value, so we extract it here. */
	uint8_t cookie[4], version[2], vendor[2], client_key[4];
	uint8_t real_session_id = session_id_masked;
	ucdr_deserialize_array_uint8_t(ub, cookie, 4);
	ucdr_deserialize_array_uint8_t(ub, version, 2);
	ucdr_deserialize_array_uint8_t(ub, vendor, 2);
	ucdr_deserialize_array_uint8_t(ub, client_key, 4);
	ucdr_deserialize_uint8_t(ub, &real_session_id);

	sess.active = true;
	sess.session_id = real_session_id;
	memcpy(sess.key, client_key, 4);
	sess.seq_none = 0;
	sess.seq_be = 0;
	sess.seq_rel = 0;

	/* uxr_read_session_header rejects any reply whose header session_id !=
	 * info->id, including the STATUS_AGENT we send back here. So we already
	 * need to use the un-masked id for THIS reply. */
	cur_session_id = real_session_id;
	memcpy(cur_key, client_key, 4);

	printk("broker: session up id=0x%02x\n", real_session_id);
	send_status_agent(0, STATUS_OK);
}

static void handle_create(ucdrBuffer *ub, uint16_t length)
{
	(void)length;
	struct base_request b = read_base_request(ub);
	send_status(b.request_id, b.object_id, STATUS_OK);
}

static void handle_get_info(ucdrBuffer *ub, uint16_t length)
{
	(void)length;
	struct base_request b = read_base_request(ub);
	send_info(b.request_id, b.object_id);
}

static void handle_delete(ucdrBuffer *ub, uint16_t length)
{
	(void)length;
	struct base_request b = read_base_request(ub);
	send_status(b.request_id, b.object_id, STATUS_OK);
}

static void handle_write_data(ucdrBuffer *ub, uint16_t length)
{
	(void)ub; (void)length;
	/* Step 4 will route this. For now, drop. */
}

static void send_acknack(uint16_t first_unacked, uint8_t reliable_stream_id)
{
	struct loopback_slot s;
	ucdrBuffer ub;
	ucdr_init_buffer(&ub, s.data, sizeof(s.data));

	/* Per upstream client (write_submessage_acknack): the ACKNACK message
	 * itself goes on stream 0 with seq 0. The reliable stream it concerns
	 * is carried in the ACKNACK_Payload.stream_id field below. */
	put_msg_header(&ub, cur_session_id, 0, 0, cur_key);

	/* ACKNACK_Payload: first_unacked_seq_num(2) nack_bitmap(2) stream_id(1) = 5 */
	const uint16_t payload_len = 5;
	uint8_t flags = pick_endian_flag();
	put_subheader(&ub, SUB_ACKNACK, flags, payload_len);

	ucdr_serialize_endian_uint16_t(&ub, UCDR_LITTLE_ENDIANNESS, first_unacked);
	ucdr_serialize_uint8_t(&ub, 0);   /* nack_bitmap[0] = 0 (nothing missing) */
	ucdr_serialize_uint8_t(&ub, 0);   /* nack_bitmap[1] = 0 */
	ucdr_serialize_uint8_t(&ub, reliable_stream_id);

	s.len = (uint16_t)ucdr_buffer_length(&ub);
	(void)loopback_broker_send(&s);
}

static void handle_heartbeat(ucdrBuffer *ub, uint16_t length)
{
	(void)length;
	uint16_t first, last;
	uint8_t  hb_stream;
	ucdr_deserialize_endian_uint16_t(ub, UCDR_LITTLE_ENDIANNESS, &first);
	ucdr_deserialize_endian_uint16_t(ub, UCDR_LITTLE_ENDIANNESS, &last);
	ucdr_deserialize_uint8_t(ub, &hb_stream);
	send_acknack((uint16_t)(last + 1), hb_stream);
}

/* ----- Main broker loop ----- */

static void broker_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	struct loopback_slot in;
	for (;;) {
		if (loopback_broker_recv(&in, K_FOREVER) != 0) {
			continue;
		}
		if (in.len < HEADER_NO_KEY) {
			continue;
		}

		ucdrBuffer ub;
		ucdr_init_buffer(&ub, in.data, in.len);

		/* Message header. */
		uint8_t session_id, stream_id;
		uint16_t seq_num;
		uint8_t key[4] = {0};
		ucdr_deserialize_uint8_t(&ub, &session_id);
		ucdr_deserialize_uint8_t(&ub, &stream_id);
		ucdr_deserialize_endian_uint16_t(&ub, UCDR_LITTLE_ENDIANNESS, &seq_num);
		if (session_id < SESSION_NO_KEY) {
			ucdr_deserialize_array_uint8_t(&ub, key, 4);
		}
		/* Stash so reply builders can echo the right header values. */
		cur_session_id = session_id;
		cur_in_stream_id = stream_id;
		memcpy(cur_key, key, 4);

		/* Proactive ACKNACK for messages on the agent's input reliable
		 * stream. The client's reliable output buffer is small (default
		 * RMW_UXRCE_STREAM_HISTORY=4); without timely ACKs the client
		 * stalls and run_xrce_session times out. */
		if (stream_id >= STREAM_THRESH_REL) {
			send_acknack((uint16_t)(seq_num + 1), stream_id);
		}

		/* Iterate submessages until buffer exhausted. */
		while (ucdr_buffer_remaining(&ub) >= 4) {
			/* Subheader is 4-byte aligned within the message body. */
			size_t off = ucdr_buffer_length(&ub);
			while (off % 4 != 0) {
				uint8_t pad;
				if (!ucdr_deserialize_uint8_t(&ub, &pad)) {
					goto next_msg;
				}
				off++;
			}

			uint8_t sub_id, sub_flags;
			uint16_t sub_len;
			if (!ucdr_deserialize_uint8_t(&ub, &sub_id) ||
			    !ucdr_deserialize_uint8_t(&ub, &sub_flags) ||
			    !ucdr_deserialize_endian_uint16_t(&ub,
				   UCDR_LITTLE_ENDIANNESS, &sub_len)) {
				break;
			}
			if (ucdr_buffer_remaining(&ub) < sub_len) {
				break;
			}

			/* Save pre-payload position so we can advance even if
			 * a handler doesn't fully drain the buffer. */
			size_t before = ucdr_buffer_length(&ub);

			switch (sub_id) {
			case SUB_CREATE_CLIENT:
				handle_create_client(&ub, session_id, key, sub_len);
				break;
			case SUB_CREATE:
				handle_create(&ub, sub_len);
				break;
			case SUB_GET_INFO:
				handle_get_info(&ub, sub_len);
				break;
			case SUB_DELETE:
				handle_delete(&ub, sub_len);
				break;
			case SUB_WRITE_DATA:
				handle_write_data(&ub, sub_len);
				break;
			case SUB_HEARTBEAT:
				handle_heartbeat(&ub, sub_len);
				break;
			case SUB_READ_DATA:
			case SUB_ACKNACK:
			default:
				break;
			}

			/* Skip past any unconsumed bytes of this submessage. */
			size_t consumed = ucdr_buffer_length(&ub) - before;
			if (consumed < sub_len) {
				size_t skip = sub_len - consumed;
				for (size_t i = 0; i < skip; i++) {
					uint8_t junk;
					if (!ucdr_deserialize_uint8_t(&ub, &junk)) {
						goto next_msg;
					}
				}
			}
		}
	next_msg:
		;
	}
}

void broker_start(void)
{
	memset(&sess, 0, sizeof(sess));
	k_thread_create(&broker_thread_data, broker_stack,
			K_THREAD_STACK_SIZEOF(broker_stack),
			broker_thread_fn, NULL, NULL, NULL,
			BROKER_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&broker_thread_data, "uros_broker");
}
