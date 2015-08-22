/*
 * libwebsockets-test-server - libwebsockets test implementation
 *
 * Copyright (C) 2010-2011 Andy Green <andy@warmcat.com>
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation:
 *  version 2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA  02110-1301  USA
 */

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <libwebsockets.h>
#include <linux/limits.h>
#include <lws_config.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

#include <sstream>

#include "ros/ros.h"

#include "dfcompass_msgs/status.h"
#include "dfkeggy_webui/WebUI.h"
#include "roboteq_msgs/Command.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"


ros::Publisher *pub = NULL, *lwp = NULL, *rwp = NULL;

static int close_testing;
int max_poll_elements;

static volatile int force_exit = 0;
static struct libwebsocket_context *context;

void request_transmit();

struct keggy_status_t {
	int controller_id;
	char mode;
	double lat;
	double lng;
	double heading;
	double vl;
	double vr;
	double goalX;
	double goalY;
	double goalT;
};

keggy_status_t keggy_status;

void drive(double vl, double vr) {
	if (lwp && rwp) {
		roboteq_msgs::Command cmd;
		cmd.commanded_velocity = vl * 500;
		if (cmd.commanded_velocity > 1000) {
		    cmd.commanded_velocity = 1000;
		} else if (cmd.commanded_velocity < -1000) {
		    cmd.commanded_velocity = -1000;
		}
		lwp->publish(cmd);
		cmd.commanded_velocity = vr * 500;
		if (cmd.commanded_velocity > 1000) {
		    cmd.commanded_velocity = 1000;
		} else if (cmd.commanded_velocity < -1000) {
		    cmd.commanded_velocity = -1000;
		}
		rwp->publish(cmd);
	}
}

void handle(int cid, const char *msg) {
	dfkeggy_webui::WebUI rosmsg;
	double d1, d2, d3;
	if (*msg == 'S') {
		keggy_status.controller_id = cid;
		keggy_status.mode = 'S';
		keggy_status.vl = keggy_status.vr = 0;
		drive(0,0);
		printf("RX c%d STOP\n", cid);
	}
	else if (*msg == 'F' && sscanf(msg, "F:%lf:%lf:%lf", &d1, &d2, &d3) == 3) {
		keggy_status.controller_id = cid;
		keggy_status.mode = 'F';
		keggy_status.goalX = d1;
		keggy_status.goalY = d2;
		keggy_status.goalT = d3;
		printf("RX c%d GOAL dx=%2.5lf, dy=%2.5lf, dtheta=%2.5lf\n", cid, d1, d2, d3);


		// double keggy_width = 0.4;
		// keggy_status.vl = -keggy_status.goalY + keggy_status.goalT*keggy_width/2;
		// keggy_status.vr = -keggy_status.goalY - keggy_status.goalT*keggy_width/2;
		// const double angular_threshold = M_PI/90;
		// const double linear_threshold = 0.05;
		// if (keggy_status.goalT > angular_threshold) {
		// 	keggy_status.vl = +1;
		// 	keggy_status.vr = -1;
		// }
		// else if (keggy_status.goalT < -angular_threshold) {
		// 	keggy_status.vl = -1;
		// 	keggy_status.vr = +1;
		// }
		// else if (fabs(keggy_status.goalY) > linear_threshold) {
		// 	keggy_status.vl = keggy_status.vr = keggy_status.goalY > 0 ? -1 : 1;
		// }
		// else {
		// 	keggy_status.vl = keggy_status.vr = 0;
		// }

	}
	else if (*msg == 'C' && sscanf(msg, "C:%lf:%lf", &d1, &d2) == 2) {
		keggy_status.controller_id = cid;
		keggy_status.mode = 'C';
		keggy_status.vl = d1;
		keggy_status.vr = d2;
		drive(keggy_status.vl, keggy_status.vr);
		printf("RX c%d CONTROL vl=%1.3lf, vr=%1.3lf\n", cid, d1, d2);
	}
	else {
		printf( "RX c%d BADMSG [%s]\n", cid, msg);
	}
}

size_t format_status(int cid, char *buf) {
	return sprintf(buf, "{\"active\":%d, \"mode\":\"%c\", \"vl\": %2.5lf, \"vr\": %2.5lf, \"location\": [%lf, %lf], \"heading\": %lf, \"goalX\":%lf, \"goalY\":%lf, \"goalTheta\":%f}", 
		cid == keggy_status.controller_id, keggy_status.mode,
		keggy_status.vl, keggy_status.vr,
		keggy_status.lat, keggy_status.lng, keggy_status.heading,
		keggy_status.goalX, keggy_status.goalY, keggy_status.goalT);
}

/*
 * This demo server shows how to use libwebsockets for one or more
 * websocket protocols in the same server
 *
 * It defines the following websocket protocols:
 *
 *  dumb-increment-protocol:  once the socket is opened, an incrementing
 *				ascii string is sent down it every 50ms.
 *				If you send "reset\n" on the websocket, then
 *				the incrementing number is reset to 0.
 *
 *  lws-mirror-protocol: copies any received packet to every connection also
 *				using this protocol, including the sender
 */

enum demo_protocols {
	PROTOCOL_HTTP = 0,
	PROTOCOL_DUMB_INCREMENT,
	DEMO_PROTOCOL_COUNT
};


const char *resource_path = 0;

/*
 * We take a strict whitelist approach to stop ../ attacks
 */

struct serveable {
	const char *urlpath;
	const char *mimetype;
}; 

struct per_session_data__http {
	int fd;
};

const char * get_mimetype(const char *file)
{
	int n = strlen(file);

	if (n < 5)
		return NULL;

	if (!strcmp(&file[n - 4], ".ico"))
		return "image/x-icon";

	if (!strcmp(&file[n - 4], ".png"))
		return "image/png";

	if (!strcmp(&file[n - 5], ".html"))
		return "text/html";

	if (!strcmp(&file[n - 3], ".js"))
		return "application/javascript";

	return NULL;
}

/* this protocol server (always the first one) just knows how to do HTTP */

static int callback_http(struct libwebsocket_context *context,
		struct libwebsocket *wsi,
		enum libwebsocket_callback_reasons reason, void *user,
							   void *vin, size_t len)
{
	const char *in = (const char*)vin;
	char buf[256];
	char leaf_path[1024];
	char b64[64];
	struct timeval tv;
	int n, m;
	unsigned char *p;
	char *other_headers;
	static unsigned char buffer[4096];
	struct stat stat_buf;
	struct per_session_data__http *pss =
			(struct per_session_data__http *)user;
	const char *mimetype;
#ifdef EXTERNAL_POLL
	struct libwebsocket_pollargs *pa = (struct libwebsocket_pollargs *)in;
#endif
	unsigned char *end;
	switch (reason) {
	case LWS_CALLBACK_HTTP:

		//dump_handshake_info(wsi);

		if (len < 1) {
			libwebsockets_return_http_status(context, wsi,
						HTTP_STATUS_BAD_REQUEST, NULL);
			goto try_to_reuse;
		}

		/* this example server has no concept of directories */
		if (strchr((const char *)in + 1, '/')) {
			libwebsockets_return_http_status(context, wsi,
						HTTP_STATUS_FORBIDDEN, NULL);
			goto try_to_reuse;
		}

		/* if a legal POST URL, let it continue and accept data */
		if (lws_hdr_total_length(wsi, WSI_TOKEN_POST_URI))
			return 0;

		/* check for the "send a big file by hand" example case */

		if (!strcmp((const char *)in, "/leaf.jpg")) {
			if (strlen(resource_path) > sizeof(leaf_path) - 10)
				return -1;
			sprintf(leaf_path, "%s/leaf.jpg", resource_path);

			/* well, let's demonstrate how to send the hard way */

			p = buffer + LWS_SEND_BUFFER_PRE_PADDING;
			end = p + sizeof(buffer) - LWS_SEND_BUFFER_PRE_PADDING;
#ifdef WIN32
			pss->fd = open(leaf_path, O_RDONLY | _O_BINARY);
#else
			pss->fd = open(leaf_path, O_RDONLY);
#endif

			if (pss->fd < 0)
				return -1;

			if (fstat(pss->fd, &stat_buf) < 0)
				return -1;

			/*
			 * we will send a big jpeg file, but it could be
			 * anything.  Set the Content-Type: appropriately
			 * so the browser knows what to do with it.
			 * 
			 * Notice we use the APIs to build the header, which
			 * will do the right thing for HTTP 1/1.1 and HTTP2
			 * depending on what connection it happens to be working
			 * on
			 */
			if (lws_add_http_header_status(context, wsi, 200, &p, end))
				return 1;
			if (lws_add_http_header_by_token(context, wsi,
					WSI_TOKEN_HTTP_SERVER,
				    	(unsigned char *)"libwebsockets",
					13, &p, end))
				return 1;
			if (lws_add_http_header_by_token(context, wsi,
					WSI_TOKEN_HTTP_CONTENT_TYPE,
				    	(unsigned char *)"image/jpeg",
					10, &p, end))
				return 1;
			if (lws_add_http_header_content_length(context, wsi,
						stat_buf.st_size, &p, end))
				return 1;
			if (lws_finalize_http_header(context, wsi, &p, end))
				return 1;

			/*
			 * send the http headers...
			 * this won't block since it's the first payload sent
			 * on the connection since it was established
			 * (too small for partial)
			 * 
			 * Notice they are sent using LWS_WRITE_HTTP_HEADERS
			 * which also means you can't send body too in one step,
			 * this is mandated by changes in HTTP2
			 */

			n = libwebsocket_write(wsi,
					buffer + LWS_SEND_BUFFER_PRE_PADDING,
					p - (buffer + LWS_SEND_BUFFER_PRE_PADDING),
					LWS_WRITE_HTTP_HEADERS);

			if (n < 0) {
				close(pss->fd);
				return -1;
			}
			/*
			 * book us a LWS_CALLBACK_HTTP_WRITEABLE callback
			 */
			libwebsocket_callback_on_writable(context, wsi);
			break;
		}

		/* if not, send a file the easy way */
		strcpy(buf, resource_path);
		if (strcmp(in, "/")) {
			if (*((const char *)in) != '/')
				strcat(buf, "/");
			strncat(buf, in, sizeof(buf) - strlen(resource_path));
		} else /* default file to serve */
			strcat(buf, "/index.html");
		buf[sizeof(buf) - 1] = '\0';

		/* refuse to serve files we don't understand */
		mimetype = get_mimetype(buf);
		if (!mimetype) {
			lwsl_err("Unknown mimetype for %s\n", buf);
			libwebsockets_return_http_status(context, wsi,
				      HTTP_STATUS_UNSUPPORTED_MEDIA_TYPE, NULL);
			return -1;
		}

		/* demostrates how to set a cookie on / */

		other_headers = NULL;
		n = 0;
		if (!strcmp((const char *)in, "/") &&
			   !lws_hdr_total_length(wsi, WSI_TOKEN_HTTP_COOKIE)) {
			/* this isn't very unguessable but it'll do for us */
			gettimeofday(&tv, NULL);
			n = sprintf(b64, "test=LWS_%u_%u_COOKIE;Max-Age=360000",
				(unsigned int)tv.tv_sec,
				(unsigned int)tv.tv_usec);

			p = (unsigned char *)leaf_path;

			if (lws_add_http_header_by_name(context, wsi, 
				(unsigned char *)"set-cookie:", 
				(unsigned char *)b64, n, &p,
				(unsigned char *)leaf_path + sizeof(leaf_path)))
				return 1;
			n = (char *)p - leaf_path;
			other_headers = leaf_path;
		}

		n = libwebsockets_serve_http_file(context, wsi, buf,
						mimetype, other_headers, n);
		if (n < 0 || ((n > 0) && lws_http_transaction_completed(wsi)))
			return -1; /* error or can't reuse connection: close the socket */

		/*
		 * notice that the sending of the file completes asynchronously,
		 * we'll get a LWS_CALLBACK_HTTP_FILE_COMPLETION callback when
		 * it's done
		 */

		break;

	case LWS_CALLBACK_HTTP_BODY:
		strncpy(buf, in, 20);
		buf[20] = '\0';
		if (len < 20)
			buf[len] = '\0';

		lwsl_notice("LWS_CALLBACK_HTTP_BODY: %s... len %d\n",
				(const char *)buf, (int)len);

		break;

	case LWS_CALLBACK_HTTP_BODY_COMPLETION:
		lwsl_notice("LWS_CALLBACK_HTTP_BODY_COMPLETION\n");
		/* the whole of the sent body arrived, close or reuse the connection */
		libwebsockets_return_http_status(context, wsi,
						HTTP_STATUS_OK, NULL);
		goto try_to_reuse;

	case LWS_CALLBACK_HTTP_FILE_COMPLETION:
//		lwsl_info("LWS_CALLBACK_HTTP_FILE_COMPLETION seen\n");
		/* kill the connection after we sent one file */
		goto try_to_reuse;

	case LWS_CALLBACK_HTTP_WRITEABLE:
		/*
		 * we can send more of whatever it is we were sending
		 */
		do {
			/* we'd like the send this much */
			n = sizeof(buffer) - LWS_SEND_BUFFER_PRE_PADDING;
			
			/* but if the peer told us he wants less, we can adapt */
			m = lws_get_peer_write_allowance(wsi);

			/* -1 means not using a protocol that has this info */
			if (m == 0)
				/* right now, peer can't handle anything */
				goto later;

			if (m != -1 && m < n)
				/* he couldn't handle that much */
				n = m;
			
			n = read(pss->fd, buffer + LWS_SEND_BUFFER_PRE_PADDING,
									n);
			/* problem reading, close conn */
			if (n < 0)
				goto bail;
			/* sent it all, close conn */
			if (n == 0)
				goto flush_bail;
			/*
			 * To support HTTP2, must take care about preamble space
			 * 
			 * identification of when we send the last payload frame
			 * is handled by the library itself if you sent a
			 * content-length header
			 */
			m = libwebsocket_write(wsi,
					       buffer + LWS_SEND_BUFFER_PRE_PADDING,
					       n, LWS_WRITE_HTTP);
			if (m < 0)
				/* write failed, close conn */
				goto bail;

			/*
			 * http2 won't do this
			 */
			if (m != n)
				/* partial write, adjust */
				if (lseek(pss->fd, m - n, SEEK_CUR) < 0)
					goto bail;

			if (m) /* while still active, extend timeout */
				libwebsocket_set_timeout(wsi,
					PENDING_TIMEOUT_HTTP_CONTENT, 5);
			
			/* if we have indigestion, let him clear it before eating more */
			if (lws_partial_buffered(wsi))
				break;

		} while (!lws_send_pipe_choked(wsi));

later:
		libwebsocket_callback_on_writable(context, wsi);
		break;
flush_bail:
		/* true if still partial pending */
		if (lws_partial_buffered(wsi)) {
			libwebsocket_callback_on_writable(context, wsi);
			break;
		}
		close(pss->fd);
		goto try_to_reuse;

bail:
		close(pss->fd);
		return -1;

	/*
	 * callback for confirming to continue with client IP appear in
	 * protocol 0 callback since no websocket protocol has been agreed
	 * yet.  You can just ignore this if you won't filter on client IP
	 * since the default uhandled callback return is 0 meaning let the
	 * connection continue.
	 */

	case LWS_CALLBACK_FILTER_NETWORK_CONNECTION:

		/* if we returned non-zero from here, we kill the connection */
		break;

	case LWS_CALLBACK_GET_THREAD_ID:
		/*
		 * if you will call "libwebsocket_callback_on_writable"
		 * from a different thread, return the caller thread ID
		 * here so lws can use this information to work out if it
		 * should signal the poll() loop to exit and restart early
		 */

		/* return pthread_getthreadid_np(); */

		break;

	default:
		break;
	}

	return 0;
	
try_to_reuse:
	if (lws_http_transaction_completed(wsi))
		return -1;

	return 0;
}


static int next_client_id = 1;

struct per_session_data__dumb_increment {
	int client_id;
};

static int
callback_dumb_increment(struct libwebsocket_context *context,
			struct libwebsocket *wsi,
			enum libwebsocket_callback_reasons reason,
					       void *userdata, void *in, size_t len)
{
	int n, m;
	unsigned char buf[LWS_SEND_BUFFER_PRE_PADDING + 1024 +
						  LWS_SEND_BUFFER_POST_PADDING];
	unsigned char *p = &buf[LWS_SEND_BUFFER_PRE_PADDING];
	struct per_session_data__dumb_increment *pss = (struct per_session_data__dumb_increment *)userdata;

	switch (reason) {

		case LWS_CALLBACK_ESTABLISHED:
			pss->client_id = next_client_id++;
			printf("RX c%d CONNECT\n", pss->client_id);
			request_transmit();	
			break;

		case LWS_CALLBACK_SERVER_WRITEABLE:
			n = format_status(pss->client_id, (char *)p);
			m = libwebsocket_write(wsi, p, n, LWS_WRITE_TEXT);
			if (m < n) {
				lwsl_err("ERROR %d writing to di socket\n", n);
				return -1;
			}
			else {
				printf("TX c%d %s\n", pss->client_id, (char*)p);
			}
			break;

		case LWS_CALLBACK_RECEIVE:
			handle(pss->client_id, (const char*)in);
			request_transmit();	
			break;

		default:
			break;
	}

	return 0;
}

/* list of supported protocols and callbacks */

static struct libwebsocket_protocols protocols[] = {
	/* first protocol must always be HTTP handler */

	{
		"http",		/* name */
		callback_http,		/* callback */
		sizeof (struct per_session_data__http),	/* per_session_data_size */
		0,			/* max frame size / rx buffer */
	},
	{
		"keggyctl",
		callback_dumb_increment,
		sizeof(struct per_session_data__dumb_increment),
		0,
	},
	{ NULL, NULL, 0, 0 } /* terminator */
};

void request_transmit() {
	libwebsocket_callback_on_writable_all_protocol(&protocols[PROTOCOL_DUMB_INCREMENT]);
}

void sighandler(int sig)
{
	force_exit = 1;
	libwebsocket_cancel_service(context);
}

static struct option options[] = {
	{ "help",	no_argument,		NULL, 'h' },
	{ "debug",	required_argument,	NULL, 'd' },
	{ "port",	required_argument,	NULL, 'p' },
	{ "ssl",	no_argument,		NULL, 's' },
	{ "allow-non-ssl",	no_argument,		NULL, 'a' },
	{ "interface",  required_argument,	NULL, 'i' },
	{ "closetest",  no_argument,		NULL, 'c' },
	{ "libev",  no_argument,		NULL, 'e' },
	{ "resource_path", required_argument,		NULL, 'r' },
	{ NULL, 0, 0, 0 }
};

extern void broadcast_geometry();

std::string get_cwd() {
	char buf[PATH_MAX];
	return getcwd(buf,PATH_MAX);
}

static void callback_compass(const dfcompass_msgs::status::ConstPtr& msg) {
	keggy_status.heading = msg->heading_degrees;
	request_transmit();
}

static void callback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  printf("GPS data: longitude=%f, latitude=%f\n", msg->longitude, msg->latitude);
}

int main(int argc, char **argv)
{
	char cert_path[1024];
	char key_path[1024];
	int n = 0;
	int use_ssl = 0;
	int opts = 0;
	char interface_name[128] = "";
	const char *iface = NULL;
	int syslog_options = LOG_PID | LOG_PERROR;
	unsigned int ms, oldms = 0;
	struct lws_context_creation_info info;

	int debug_level = 7;

	memset(&info, 0, sizeof info);
	info.port = 8082;

	memset(&keggy_status, 0, sizeof keggy_status);
	keggy_status.controller_id = -1;
	keggy_status.mode = 'S';

	std::string keggy_home = getenv("KEGGY_HOME") ? std::string(getenv("KEGGY_HOME")) : get_cwd(); 
	std::string webroot = keggy_home + "/dfkeggy_webui/www";
	resource_path = webroot.c_str();
	printf("KEGGY_HOME=%s\n", keggy_home.c_str());

	while (n >= 0) {
		n = getopt_long(argc, argv, "eci:hsap:d:Dr:", options, NULL);
		if (n < 0)
			continue;
		switch (n) {
		case 'e':
			opts |= LWS_SERVER_OPTION_LIBEV;
			break;
		case 'd':
			debug_level = atoi(optarg);
			break;
		case 's':
			use_ssl = 1;
			break;
		case 'a':
			opts |= LWS_SERVER_OPTION_ALLOW_NON_SSL_ON_SSL_PORT;
			break;
		case 'p':
			info.port = atoi(optarg);
			break;
		case 'i':
			strncpy(interface_name, optarg, sizeof interface_name);
			interface_name[(sizeof interface_name) - 1] = '\0';
			iface = interface_name;
			break;
		case 'c':
			close_testing = 1;
			fprintf(stderr, " Close testing mode -- closes on "
					   "client after 50 dumb increments"
					   "and suppresses lws_mirror spam\n");
			break;
		case 'r':
			resource_path = optarg;
			printf("Setting resource path to \"%s\"\n", resource_path);
			break;
		case 'h':
			fprintf(stderr, "Usage: test-server "
					"[--port=<p>] [--ssl] "
					"[-d <log bitfield>] "
					"[--resource_path <path>]\n");
			exit(1);
		}
	}

	signal(SIGINT, sighandler);

	/* we will only try to log things according to our debug_level */
	setlogmask(LOG_UPTO (LOG_DEBUG));
	openlog("lwsts", syslog_options, LOG_DAEMON);

	/* tell the library what debug level to emit and to send it to syslog */
	lws_set_log_level(debug_level, lwsl_emit_syslog);

	lwsl_notice("libwebsockets test server - "
			"(C) Copyright 2010-2015 Andy Green <andy@warmcat.com> - "
						    "licensed under LGPL2.1\n");

	printf("Using resource path \"%s\"\n", resource_path);

	info.iface = iface;
	info.protocols = protocols;
	if (!use_ssl) {
		info.ssl_cert_filepath = NULL;
		info.ssl_private_key_filepath = NULL;
	} else {
		if (strlen(resource_path) > sizeof(cert_path) - 32) {
			lwsl_err("resource path too long\n");
			return -1;
		}
		sprintf(cert_path, "%s/libwebsockets-test-server.pem",
								resource_path);
		if (strlen(resource_path) > sizeof(key_path) - 32) {
			lwsl_err("resource path too long\n");
			return -1;
		}
		sprintf(key_path, "%s/libwebsockets-test-server.key.pem",
								resource_path);

		info.ssl_cert_filepath = cert_path;
		info.ssl_private_key_filepath = key_path;
	}
	info.gid = -1;
	info.uid = -1;
	info.options = opts;

	context = libwebsocket_create_context(&info);
	if (context == NULL) {
		lwsl_err("libwebsocket init failed\n");
		return -1;
	}

    ros::init(argc, argv, "dfkeggy_webui");
    ros::NodeHandle ros_node;
    ros::Publisher  webui_pub = ros_node.advertise<dfkeggy_webui::WebUI>("webui", 1000);
    ros::Publisher  left_wheel_pub = ros_node.advertise<roboteq_msgs::Command>("/roboteq_left/cmd", 1000);
    ros::Publisher  right_wheel_pub = ros_node.advertise<roboteq_msgs::Command>("/roboteq_right/cmd", 1000);
    ros::Subscriber compass_sub = ros_node.subscribe("/dfcompass_driver/status", 1000, callback_compass);
    ros::Subscriber gps_sub = ros_node.subscribe("/gps_driver/fix", 1000, callback_gps);
    pub = &webui_pub;
    lwp = &left_wheel_pub;
    rwp = &right_wheel_pub;
    
    ros::spinOnce();
    broadcast_geometry();

	n = 0;
	while (n >= 0 && ros::ok() && !force_exit) {
		n = libwebsocket_service(context, 50);
	    ros::spinOnce();
	}

	libwebsocket_context_destroy(context);

	lwsl_notice("libwebsockets-test-server exited cleanly\n");

	closelog();
	return 0;
}
