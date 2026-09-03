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
#include <string.h>
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

#define WIFI_AP_EVENTS (NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_STA_CONNECTED | \
			NET_EVENT_WIFI_AP_STA_DISCONNECTED)

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t ev, struct net_if *iface)
{
	ARG_UNUSED(cb); ARG_UNUSED(iface);
	switch (ev) {
	case NET_EVENT_WIFI_AP_ENABLE_RESULT:
		printk("fpga_bridge: SoftAP up -- relaying FPGA UART on udp :%d, cmds on :%d\n",
		       TELEM_PORT, CMD_PORT);
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
RING_BUF_DECLARE(rx_ring, 8192);
static K_SEM_DEFINE(rx_data, 0, 1);

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
				(void)zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&dst, sizeof(dst));
				len = 0;
			}
		} else {
			/* ring empty: flush a pending partial line, then wait for the ISR to signal more */
			if (len > 0) {
				(void)zsock_sendto(sock, buf, len, 0, (struct sockaddr *)&dst, sizeof(dst));
				len = 0;
			}
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
		for (ssize_t i = 0; i < n; i++) uart_poll_out(fpga_uart, buf[i]);
	}
}

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
	return 0;
}
