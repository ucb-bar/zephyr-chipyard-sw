/*
 * riskybird FPGA telemetry bridge (ESP32-C6)
 *
 * Transparent UART<->WiFi bridge: relays whatever the FPGA (RV64GC Rocket)
 * prints on the ESP link UART out over the SoftAP as UDP, and relays UDP
 * commands back to the FPGA. Lets a laptop on the drone's WiFi see the FPGA's
 * telemetry with no wired connection.
 *
 * LINK: the ESP's UART0 is wired to the FPGA's 2nd peripheral UART (uart1):
 *   ESP U0TXD (GPIO16) -> FPGA E13 (rxd)      [ESP -> FPGA, commands]
 *   ESP U0RXD (GPIO17) <- FPGA F14 (txd)      [FPGA -> ESP, telemetry]
 * (Derived from riskybirdv3_base netlist x Trenz TE0712 B2B map; see
 *  FPGA_BRINGUP_LOG.md. The ESP debug console stays on USB-serial-JTAG, so
 *  UART0 is free for the FPGA link.)
 *
 * WiFi: SoftAP, static 192.168.4.1 + DHCP server (pool from .10), mirroring
 * rose_flight_controller/telem_wifi so the same ground station works:
 *   downlink  UDP :14550  (FPGA bytes, broadcast on 192.168.4.255)
 *   uplink    UDP :14551  (bytes -> FPGA)
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/net/http/service.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/method.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#define AP_SSID_PREFIX "riskybird-"
#define AP_IP          "192.168.4.1"
#define AP_NETMASK     "255.255.255.0"
#define AP_BROADCAST   "192.168.4.255"
#define AP_CHANNEL     6
#define TELEM_PORT     14550
#define CMD_PORT       14551
#ifndef FPGA_LINK_BAUD
#define FPGA_LINK_BAUD 921600          /* matches the FPGA uart1 baud (telem_uart.c) -- fast enough
                                        * for downsampled camera snapshots + never back-pressures telem */
#endif

/* The FPGA-link UART: the board overlay points chosen `riskybird,fpga-uart` at uart0. */
static const struct device *const fpga_uart = DEVICE_DT_GET(DT_CHOSEN(riskybird_fpga_uart));

static struct net_if *ap_iface;
static struct net_mgmt_event_callback wifi_cb;
static K_SEM_DEFINE(ap_up, 0, 1);
static volatile bool g_ap_ready;   /* set when the SoftAP is operational (AP_ENABLE_RESULT) */

#define WIFI_AP_EVENTS (NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_STA_CONNECTED | \
			NET_EVENT_WIFI_AP_STA_DISCONNECTED)

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t ev, struct net_if *iface)
{
	ARG_UNUSED(cb); ARG_UNUSED(iface);
	switch (ev) {
	case NET_EVENT_WIFI_AP_ENABLE_RESULT:
		printk("fpga_bridge: SoftAP up -- relaying FPGA UART on udp :%d, cmds on :%d\n",
		       TELEM_PORT, CMD_PORT);
		g_ap_ready = true;
		k_sem_give(&ap_up);
		break;
	case NET_EVENT_WIFI_AP_STA_CONNECTED:   printk("fpga_bridge: client joined\n"); break;
	case NET_EVENT_WIFI_AP_STA_DISCONNECTED: printk("fpga_bridge: client left\n"); break;
	default: break;
	}
}

static void configure_ap_ipv4(struct net_if *iface)
{
	struct in_addr addr, netmask, pool;
	if (zsock_inet_pton(AF_INET, AP_IP, &addr) != 1 ||
	    zsock_inet_pton(AF_INET, AP_NETMASK, &netmask) != 1) {
		printk("fpga_bridge: bad AP IP literal\n"); return;
	}
	net_if_ipv4_set_gw(iface, &addr);
	if (net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0) == NULL)
		printk("fpga_bridge: set AP IP failed\n");
	if (!net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask))
		printk("fpga_bridge: set AP netmask failed\n");
	pool = addr; pool.s4_addr[3] += 10;
	if (net_dhcpv4_server_start(iface, &pool) != 0)
		printk("fpga_bridge: DHCP server did not start\n");
}

static void start_softap(void)
{
	uint8_t mac[6] = {0};
	char ssid[32];
	net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler, WIFI_AP_EVENTS);
	net_mgmt_add_event_callback(&wifi_cb);

	ap_iface = net_if_get_first_wifi();
	if (!ap_iface) { printk("fpga_bridge: no WiFi iface\n"); return; }

	struct net_linkaddr *ll = net_if_get_link_addr(ap_iface);
	if (ll && ll->len >= 6) memcpy(mac, ll->addr, 6);
	int n = snprintk(ssid, sizeof(ssid), "%s%02x%02x", AP_SSID_PREFIX, mac[4], mac[5]);

	configure_ap_ipv4(ap_iface);

	struct wifi_connect_req_params ap = {0};
	ap.ssid = (const uint8_t *)ssid;
	ap.ssid_length = n;
	ap.channel = AP_CHANNEL;
	ap.band = WIFI_FREQ_BAND_2_4_GHZ;
	ap.security = WIFI_SECURITY_TYPE_NONE;   /* open network */
	if (net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, ap_iface, &ap, sizeof(ap)) != 0)
		printk("fpga_bridge: AP_ENABLE request failed\n");
	printk("fpga_bridge: SoftAP SSID '%s' (open), IP %s\n", ssid, AP_IP);
}

/* ---- FPGA UART -> WiFi (downlink) ------------------------------------------------------------ *
 * Interrupt-driven RX into a ring buffer: the ISR drains the small hardware RX FIFO into an 8 KB
 * ring the instant bytes arrive, so a sustained burst (e.g. a camera snapshot's chunk stream) never
 * overflows the FIFO. The relay thread then reframes the ring on newlines and ships UDP at its own
 * pace -- decoupling ingest from the (slower, blocking) UDP send is what the old poll+sleep loop
 * lacked, which corrupted any line that landed while it was mid-send or mid-sleep. */
RING_BUF_DECLARE(rx_ring, 16384);
static K_SEM_DEFINE(rx_data, 0, 1);

/* Shared with the on-board HTTP server: the latest "RBT ..." telemetry line (served at GET /t) and a
 * mutex serializing FPGA-link UART TX so the UDP :14551 uplink and the HTTP POST /cmd don't interleave. */
static K_MUTEX_DEFINE(telem_mtx);
static char   g_telem[256];
static size_t g_telem_len;
static K_MUTEX_DEFINE(uart_tx_mtx);

static void fpga_send(const uint8_t *d, size_t n)
{
	k_mutex_lock(&uart_tx_mtx, K_FOREVER);
	for (size_t i = 0; i < n; i++) { uart_poll_out(fpga_uart, d[i]); }
	k_mutex_unlock(&uart_tx_mtx);
}

/* the shared mobile flight page, gzipped, embedded into ESP flash (served at GET /) */
static const uint8_t mobile_html_gz[] = {
#include "mobile_html.gz.inc"
};

/* ---- camera snapshot reassembly (served at GET /snap) --------------------------------------- *
 * The FPGA streams a downsampled frame as newline-framed base64 chunks on the telemetry link:
 *     IMG s=<seq> k=<k>/<total> w=<W> h=<H> <base64 of up to 150 raw grayscale bytes>
 * (k is 0-based and the chunks arrive in order over the lossless ISR ring.) We concatenate the
 * chunks' base64 into the base64 of the whole W*H frame and hand the last COMPLETE frame to /snap.
 * No re-encode is needed: every chunk but the last carries a multiple of 3 raw bytes (150), so the
 * chunk base64 strings concatenate into valid base64 of the full frame -- the browser atob()s it
 * straight into pixels. A partial/corrupted frame is dropped, so /snap always holds a whole one. */
#define SNAP_B64_MAX 24576          /* base64 of ~18 KB raw px; a 128x126 frame needs 21504 chars */
static K_MUTEX_DEFINE(snap_mtx);
static char   g_snap[SNAP_B64_MAX]; /* last complete frame: concatenated base64 (0 len = none yet) */
static size_t g_snap_len;
static int    g_snap_w, g_snap_h;
/* in-progress accumulator -- touched only by the single uart_to_udp thread, so it needs no lock */
static char     asm_buf[SNAP_B64_MAX];
static size_t   asm_len;
static uint32_t asm_seq;
static int      asm_next_k, asm_total, asm_w, asm_h, asm_chunk_len;
static bool     asm_on;

static void snap_feed(const char *line, size_t len)
{
	/* Parse "IMG s=<u> k=<u>/<u> w=<u> h=<u> <b64>" strictly; reject anything malformed so a line
	 * mangled by an RX-ring byte-drop can't poison the accumulator (same guard rationale as /t). */
	if (len < 20 || memcmp(line, "IMG s=", 6) != 0) { return; }
	char *p = (char *)line + 6;
	unsigned long seq   = strtoul(p, &p, 10); if (*p != ' ') { return; } while (*p == ' ') { p++; }
	if (memcmp(p, "k=", 2) != 0) { return; } p += 2;
	unsigned long k     = strtoul(p, &p, 10); if (*p != '/') { return; } p++;
	unsigned long total = strtoul(p, &p, 10); if (*p != ' ') { return; } while (*p == ' ') { p++; }
	if (memcmp(p, "w=", 2) != 0) { return; } p += 2;
	unsigned long w     = strtoul(p, &p, 10); if (*p != ' ') { return; } while (*p == ' ') { p++; }
	if (memcmp(p, "h=", 2) != 0) { return; } p += 2;
	unsigned long h     = strtoul(p, &p, 10); if (*p != ' ') { return; } while (*p == ' ') { p++; }
	const char *b64 = p;
	size_t blen = (size_t)(line + len - b64);
	while (blen && (b64[blen - 1] == '\n' || b64[blen - 1] == '\r' || b64[blen - 1] == ' ')) { blen--; }
	if (total == 0 || total > 128 || blen == 0 || blen > 300) { return; }

	if (k == 0) {                       /* first chunk -> start a fresh frame */
		asm_seq = (uint32_t)seq; asm_total = (int)total; asm_w = (int)w; asm_h = (int)h;
		asm_len = 0; asm_next_k = 0; asm_on = true;
		asm_chunk_len = (int)blen;  /* every non-final chunk must be exactly this long */
	}
	/* accept only the next in-order chunk of the current frame; else drop this partial frame */
	if (!asm_on || (uint32_t)seq != asm_seq || (int)k != asm_next_k) { asm_on = false; return; }
	/* Length gate: every chunk but the last carries the same 150 raw bytes -> identical base64 length.
	 * A short one is a mid-payload byte-drop that would shift all following chunks in the concatenated
	 * base64 (garbling the lower frame); reject the frame instead of publishing a corrupted image. */
	if ((int)k < asm_total - 1 ? (int)blen != asm_chunk_len : (int)blen > asm_chunk_len) {
		asm_on = false; return;
	}
	if (asm_len + blen > sizeof(asm_buf)) { asm_on = false; return; }
	memcpy(asm_buf + asm_len, b64, blen);
	asm_len += blen;
	asm_next_k++;
	if (asm_next_k >= asm_total) {       /* all chunks in -> publish the completed frame */
		k_mutex_lock(&snap_mtx, K_FOREVER);
		memcpy(g_snap, asm_buf, asm_len);
		g_snap_len = asm_len; g_snap_w = asm_w; g_snap_h = asm_h;
		k_mutex_unlock(&snap_mtx);
		asm_on = false;
	}
}

static void fpga_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t tmp[64];
		int n = uart_fifo_read(dev, tmp, sizeof(tmp));
		if (n <= 0) {
			break;
		}
		/* Best-effort: if the ring is full the ground station will just see a dropped line and
		 * resync on the next newline -- far better than losing bytes mid-line (which merges lines). */
		(void)ring_buf_put(&rx_ring, tmp, n);
		k_sem_give(&rx_data);
	}
}

static void uart_to_udp(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	(void)k_sem_take(&ap_up, K_SECONDS(15));

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) { printk("fpga_bridge: tx socket failed (%d)\n", errno); return; }
	int on = 1;
	(void)zsock_setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
	struct sockaddr_in dst = {0};
	dst.sin_family = AF_INET;
	dst.sin_port = htons(TELEM_PORT);
	(void)zsock_inet_pton(AF_INET, AP_BROADCAST, &dst.sin_addr);

	uart_irq_callback_user_data_set(fpga_uart, fpga_uart_isr, NULL);
	uart_irq_rx_enable(fpga_uart);

	uint8_t buf[600];
	size_t len = 0;
	for (;;) {
		uint8_t ch;
		if (ring_buf_get(&rx_ring, &ch, 1) == 1) {
			buf[len++] = ch;
			if (ch == '\n' || len == sizeof(buf)) {
				/* Stash the latest COMPLETE telemetry line for the HTTP /t poll. Require the
				 * "RBT it=" prefix so a line corrupted by an RX-ring byte-drop (which can look
				 * like "RBT .004 ...") doesn't overwrite the last good line. */
				if (len >= 7 && memcmp(buf, "RBT it=", 7) == 0 && len <= sizeof(g_telem)) {
					k_mutex_lock(&telem_mtx, K_FOREVER);
					g_telem_len = len;
					memcpy(g_telem, buf, len);
					k_mutex_unlock(&telem_mtx);
				} else if (len >= 4 && memcmp(buf, "IMG ", 4) == 0) {
					/* camera snapshot chunk -> reassemble the frame for GET /snap */
					snap_feed((const char *)buf, len);
				}
				(void)zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&dst, sizeof(dst));
				len = 0;
			}
		} else {
			/* Ring empty: wait for the ISR to signal more bytes. Do NOT flush a partial line here
			 * -- the ring routinely drains faster than the ISR delivers, so flushing on idle would
			 * split most lines mid-way, which breaks the ground-station line framing and means the
			 * /t telemetry capture (which keys on a complete "RBT it=" line) never sees a full one. */
			(void)k_sem_take(&rx_data, K_MSEC(20));
		}
	}
}

/* ---- WiFi -> FPGA UART (uplink) -------------------------------------------------------------- */
static void udp_to_uart(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	(void)k_sem_take(&ap_up, K_SECONDS(15));

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) { printk("fpga_bridge: rx socket failed (%d)\n", errno); return; }
	struct sockaddr_in me = {0};
	me.sin_family = AF_INET;
	me.sin_port = htons(CMD_PORT);
	me.sin_addr.s_addr = htonl(INADDR_ANY);
	if (zsock_bind(sock, (struct sockaddr *)&me, sizeof(me)) < 0) {
		printk("fpga_bridge: cmd bind failed (%d)\n", errno);
		(void)zsock_close(sock); return;
	}
	uint8_t buf[256];
	for (;;) {
		ssize_t n = zsock_recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
		if (n > 0) { fpga_send(buf, (size_t)n); }
	}
}

/* ---- On-board HTTP server (port 80) ---------------------------------------------------------- *
 *   GET  /     -> the gzipped mobile flight page (from flash)
 *   GET  /t    -> the latest RBT telemetry line as text (the page polls this ~15 Hz)
 *   GET  /snap -> the last complete camera frame ("<w> <h>\n<base64>"); POST /cmd SNAP forces one
 *   POST /cmd  -> forward the body to the FPGA over uart1 (ESTOP/RESET/HOVER_Z/SNAP/...)
 * A phone on the SoftAP just browses to http://192.168.4.1/ -- no laptop or bridge script. */
static struct http_resource_detail_static page_res = {
	.common = { .type = HTTP_RESOURCE_TYPE_STATIC,
		    .bitmask_of_supported_http_methods = BIT(HTTP_GET),
		    .content_encoding = "gzip", .content_type = "text/html" },
	.static_data = mobile_html_gz,
	.static_data_len = sizeof(mobile_html_gz),
};

static int t_handler(struct http_client_ctx *client, enum http_data_status status,
		     const struct http_request_ctx *req, struct http_response_ctx *resp, void *ud)
{
	static uint8_t tbuf[sizeof(g_telem)];
	ARG_UNUSED(client); ARG_UNUSED(req); ARG_UNUSED(ud);
	if (status == HTTP_SERVER_DATA_FINAL) {
		k_mutex_lock(&telem_mtx, K_FOREVER);
		size_t n = g_telem_len;
		memcpy(tbuf, g_telem, n);
		k_mutex_unlock(&telem_mtx);
		resp->body = tbuf;
		resp->body_len = n;
		resp->final_chunk = true;
	}
	return 0;
}
static struct http_resource_detail_dynamic t_res = {
	.common = { .type = HTTP_RESOURCE_TYPE_DYNAMIC,
		    .bitmask_of_supported_http_methods = BIT(HTTP_GET) },
	.cb = t_handler,
};

static int cmd_handler(struct http_client_ctx *client, enum http_data_status status,
		       const struct http_request_ctx *req, struct http_response_ctx *resp, void *ud)
{
	static uint8_t cbuf[128];
	static size_t clen;
	static char ack[160];
	ARG_UNUSED(client); ARG_UNUSED(ud);
	if (status == HTTP_SERVER_DATA_ABORTED) { clen = 0; return 0; }
	if (req->data_len && clen + req->data_len <= sizeof(cbuf)) {
		memcpy(cbuf + clen, req->data, req->data_len);
		clen += req->data_len;
	}
	if (status == HTTP_SERVER_DATA_FINAL) {
		while (clen && (cbuf[clen - 1] == '\n' || cbuf[clen - 1] == '\r' ||
				cbuf[clen - 1] == ' ')) { clen--; }
		if (clen) { fpga_send(cbuf, clen); fpga_send((const uint8_t *)"\n", 1); }
		int n = snprintk(ack, sizeof(ack), "sent: %.*s", (int)clen, cbuf);
		resp->body = (const uint8_t *)ack;
		resp->body_len = n;
		resp->final_chunk = true;
		clen = 0;
	}
	return 0;
}
static struct http_resource_detail_dynamic cmd_res = {
	.common = { .type = HTTP_RESOURCE_TYPE_DYNAMIC,
		    .bitmask_of_supported_http_methods = BIT(HTTP_POST) },
	.cb = cmd_handler,
};

/* GET /snap -> the last complete camera frame as "<w> <h>\n<base64 of w*h grayscale bytes>"
 * (empty until the first frame reassembles). The page atob()s the payload straight into a canvas. */
static int snap_handler(struct http_client_ctx *client, enum http_data_status status,
			const struct http_request_ctx *req, struct http_response_ctx *resp, void *ud)
{
	static uint8_t sbuf[SNAP_B64_MAX + 32];
	ARG_UNUSED(client); ARG_UNUSED(req); ARG_UNUSED(ud);
	if (status == HTTP_SERVER_DATA_FINAL) {
		size_t n = 0;
		k_mutex_lock(&snap_mtx, K_FOREVER);
		if (g_snap_len) {
			n = snprintk((char *)sbuf, 32, "%d %d\n", g_snap_w, g_snap_h);
			memcpy(sbuf + n, g_snap, g_snap_len);
			n += g_snap_len;
		}
		k_mutex_unlock(&snap_mtx);
		resp->body = sbuf;
		resp->body_len = n;
		resp->final_chunk = true;
	}
	return 0;
}
static struct http_resource_detail_dynamic snap_res = {
	.common = { .type = HTTP_RESOURCE_TYPE_DYNAMIC,
		    .bitmask_of_supported_http_methods = BIT(HTTP_GET) },
	.cb = snap_handler,
};

static uint16_t http_port = 80;
HTTP_SERVICE_DEFINE(rb_http, NULL, &http_port, 4, 10, NULL, NULL, NULL);
HTTP_RESOURCE_DEFINE(rb_root, rb_http, "/", &page_res);
HTTP_RESOURCE_DEFINE(rb_t, rb_http, "/t", &t_res);
HTTP_RESOURCE_DEFINE(rb_cmd, rb_http, "/cmd", &cmd_res);
HTTP_RESOURCE_DEFINE(rb_snap, rb_http, "/snap", &snap_res);

K_THREAD_DEFINE(tx_tid, 4096, uart_to_udp, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(rx_tid, 4096, udp_to_uart, NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
	printk("\n=== riskybird FPGA telemetry bridge ===\n");
	if (!device_is_ready(fpga_uart)) {
		printk("fpga_bridge: FPGA-link UART not ready\n");
		return 1;
	}
	struct uart_config uc;
	if (uart_config_get(fpga_uart, &uc) == 0) {
		uc.baudrate = FPGA_LINK_BAUD;
		(void)uart_configure(fpga_uart, &uc);
	}
	printk("fpga_bridge: FPGA link on %s @ %d baud\n", fpga_uart->name, FPGA_LINK_BAUD);
	start_softap();
	/* Bind the HTTP listener only once the SoftAP interface is operational, else it attaches to no
	 * usable interface and every connection is refused. */
	for (int i = 0; i < 400 && !g_ap_ready; i++) { k_msleep(25); }
	k_msleep(300);
	if (http_server_start() == 0) {
		printk("fpga_bridge: HTTP server on http://%s/ (phone: join the AP, open it)\n", AP_IP);
	} else {
		printk("fpga_bridge: HTTP server failed to start\n");
	}
	return 0;
}
