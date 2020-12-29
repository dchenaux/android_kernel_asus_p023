/*
 * android_main.cpp: Thermal Daemon entry point tuned for Android
 *
 * Copyright (C) 2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 or later as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 *
 * Author Name <Srinivas.Pandruvada@linux.intel.com>
 *
 * This is the main entry point for thermal daemon. This has main function
 * which parses command line arguments, setup logs and starts thermal
 * engine.
 */

#include "thermald.h"
/*#include "thd_preference.h"
#include "thd_engine.h"
#include "thd_engine_default.h"
#include "thd_parse.h"*/

// for AID_* constatns
#include <private/android_filesystem_config.h>


#define SENSOR_TYPE_PATH "/sys/class/thermal/thermal_zone%d/type"//"/data/fake_sensor"
#define SENSOR_PATH "/sys/class/thermal/thermal_zone%d/temp"

#define CDEV_PATH_MAX "/sys/devices/system/cpu/cpu0/cpufreq/thermal_scaling_max_freq"
#define TRIP_P1 70000
#define CPUFREQ_HIGH "900000"
#define CPUFREQ_LOW "416000"

int trip_pts[3] = {90000, 105000, 105000}; // milli C
char cpu_freq[][10] = {"900000", "728000", "416000"};// kHz

int throttle(char* value, int len);

char sensor_path[50];

// getdtablesize() is removed from bionic/libc in LPDK*/
// use POSIX alternative available. Otherwise fail
#ifdef _POSIX_OPEN_MAX
#define   getdtablesize()	(_POSIX_OPEN_MAX)
#endif

// poll mode
int thd_poll_interval = 4; //in seconds
static int pid_file_handle;

// Thermal engine
//cthd_engine *thd_engine;

// Stop daemon
static void daemonShutdown() {
	if (pid_file_handle)
		close(pid_file_handle);
	//thd_engine->thd_engine_terminate();
	throttle(CPUFREQ_HIGH, 6);

	sleep(1);
	//delete thd_engine;
}

// signal handler
static void signal_handler(int sig) {
	switch (sig) {
	case SIGHUP:
		thd_log_warn("Received SIGHUP signal. \n");
		break;
	case SIGINT:
	case SIGTERM:
		thd_log_info("Daemon exiting \n");
		daemonShutdown();
		exit(EXIT_SUCCESS);
		break;
	default:
		thd_log_warn("Unhandled signal %s\n", strsignal(sig));
		break;
	}
}

static void daemonize(char *rundir, char *pidfile) {
	int pid, sid, i;
	char str[10];
	struct sigaction sig_actions;
	sigset_t sig_set;

	if (getppid() == 1) {
		return;
	}
	sigemptyset(&sig_set);
	sigaddset(&sig_set, SIGCHLD);
	sigaddset(&sig_set, SIGTSTP);
	sigaddset(&sig_set, SIGTTOU);
	sigaddset(&sig_set, SIGTTIN);
	sigprocmask(SIG_BLOCK, &sig_set, NULL);

	sig_actions.sa_handler = signal_handler;
	sigemptyset(&sig_actions.sa_mask);
	sig_actions.sa_flags = 0;

	sigaction(SIGHUP, &sig_actions, NULL);
	sigaction(SIGTERM, &sig_actions, NULL);
	sigaction(SIGINT, &sig_actions, NULL);

	pid = fork();
	if (pid < 0) {
		/* Could not fork */
		exit(EXIT_FAILURE);
	}
	if (pid > 0) {
		thd_log_info("Child process created: %d\n", pid);
		exit(EXIT_SUCCESS);
	}
	umask(027);

	sid = setsid();
	if (sid < 0) {
		exit(EXIT_FAILURE);
	}
	/* close all descriptors */
	for (i = getdtablesize(); i >= 0; --i) {
		close(i);
	}

	i = open("/dev/null", O_RDWR);
	dup(i);
	dup(i);
	chdir(rundir);

	pid_file_handle = open(pidfile, O_RDWR | O_CREAT, 0600);
	if (pid_file_handle == -1) {
		/* Couldn't open lock file */
		thd_log_info("Could not open PID lock file %s, exiting\n", pidfile);
		exit(EXIT_FAILURE);
	}
	/* Try to lock file */
#ifdef LOCKF_SUPPORT
	if (lockf(pid_file_handle, F_TLOCK, 0) == -1) {
#else
		if (flock(pid_file_handle,LOCK_EX|LOCK_NB) < 0) {
#endif
		/* Couldn't get lock on lock file */
		thd_log_info("Couldn't get lock file %d\n", getpid());
		exit(EXIT_FAILURE);
	}
	thd_log_info("Thermal PID %d\n", getpid());
	snprintf(str, sizeof(str), "%d\n", getpid());
	write(pid_file_handle, str, strlen(str));
}

static void print_usage(FILE* stream, int exit_code) {
	fprintf(stream, "Usage:  thermal-daemon options [ ... ]\n");
	fprintf(stream, "  --help Display this usage information.\n"
			"  --version Show version.\n"
			"  --no-daemon No daemon.\n"
			"  --poll-interval poll interval 0 to disable.\n"
			"  --exclusive_control to act as exclusive thermal controller. \n");

	exit(exit_code);
}


int get_core_sensor()
{
	int fd_sensor, i, ret, sid = -1;
	char path[50], buffer[20];

	for(i=0; i<10;i++) {
		memset(path, 0, 50);
		snprintf(path, 50, SENSOR_TYPE_PATH, i);
		//thd_log_info("Try to open sensor %s\n", path);
		fd_sensor = open(path, O_RDONLY, 0);
		if (fd_sensor < 0) {
			thd_log_warn("open sensor failed %s\n", path);
			continue;
		}
		ret = read(fd_sensor, (void *)buffer, 20);
		if(ret <= 0) {
			thd_log_warn("read sensor failed %s\n", path);
		} else {
			if(!strncmp(buffer, "coretemp1", strlen("coretemp1"))) {
				sid = i;
				memset(sensor_path, 0, 50);
				snprintf(sensor_path, 50, SENSOR_PATH, i);
				thd_log_warn("Found sensor %s\n", sensor_path);
				close(fd_sensor);
				break;
			}
		}
		close(fd_sensor);
	}
	return sid;
}

int read_sensor()
{
	int fd_sensor, temp = -1;
	char buffer[20];

	fd_sensor = open(sensor_path, O_RDONLY, 0);
	if (fd_sensor < 0) {
	    thd_log_warn("open sensor failed %s\n", sensor_path);
	    return -1;
	}

	if(read(fd_sensor, (void *)buffer, 20) <= 0) {
		thd_log_warn("read sensor failed %s\n", sensor_path);
	} else {
		temp = atoi(buffer);
		thd_log_info("read sensor %s temp = %d\n", sensor_path, temp);
	}
	close(fd_sensor);

	return temp;
}

int throttle(char* value, int len)
{
	int fd_cdev_max;
	char buffer[20];

	thd_log_info("throttling cooling dev %s = %s\n", CDEV_PATH_MAX, value);

	fd_cdev_max = open(CDEV_PATH_MAX, O_WRONLY, 0);
	if (fd_cdev_max < 0) {
		thd_log_warn("open cooling dev failed %s\n", CDEV_PATH_MAX);
		close(fd_cdev_max);
		return -1;
	}

	if(write(fd_cdev_max, value, len) <= 0){
		thd_log_warn("write cooling dev failed %s\n", CDEV_PATH_MAX);
	}

	close(fd_cdev_max);

	return 1;

}

int calculate_state(int sensor_temp)
{
	if(sensor_temp < trip_pts[0]) {
		return 0;
	} else if(sensor_temp >= trip_pts[0] && sensor_temp < trip_pts[1]) {
		return 1;
	} else {//>=trip_pts[2]
		return 2;
	}
}

int main(int argc, char *argv[]) {
#if 0
	int c;
	int option_index = 0;
	bool no_daemon = false;
	bool exclusive_control = false;
	bool test_mode = false;
	bool is_privileged_user = false;

	const char* const short_options = "hvnp:de";
	static struct option long_options[] = {
			{ "help", no_argument, 0, 'h' },
			{ "version", no_argument, 0, 'v' },
			{ "no-daemon", no_argument, 0, 'n' },
			{ "poll-interval", required_argument, 0, 'p' },
			{ "exclusive_control", no_argument, 0, 'e' },
			{ "test-mode", no_argument, 0, 't' },
			{ NULL, 0, NULL, 0 } };

	if (argc > 1) {
		while ((c = getopt_long(argc, argv, short_options, long_options,
				&option_index)) != -1) {
			int this_option_optind = optind ? optind : 1;
			switch (c) {
			case 'h':
				print_usage(stdout, 0);
				break;
			case 'v':
				fprintf(stdout, "1.1\n");
				exit(EXIT_SUCCESS);
				break;
			case 'n':
				no_daemon = true;
				break;
			case 'p':
				thd_poll_interval = atoi(optarg);
				break;
			case 'e':
				exclusive_control = true;
				break;
			case 't':
				test_mode = true;
				break;
			case -1:
			case 0:
				break;
			default:
				break;
			}
		}
	}

	is_privileged_user = (getuid() == 0) || (getuid() == AID_SYSTEM);
	if (!is_privileged_user && !test_mode) {
		thd_log_error("You do not have correct permissions to run thermal dameon!\n");
		exit(1);
	}

	if ((c = mkdir(TDRUNDIR, 0755)) != 0) {
		if (errno != EEXIST) {
			fprintf(stderr, "Cannot create '%s': %s\n", TDRUNDIR,
					strerror(errno));
			exit(EXIT_FAILURE);
		}
	}
	mkdir(TDCONFDIR, 0755); // Don't care return value as directory
	if (!no_daemon) {
		daemonize((char *) "/tmp/", (char *) "/tmp/thermald.pid");
	} else
		signal(SIGINT, signal_handler);

	thd_log_info(
			"Linux Thermal Daemon is starting mode %d : poll_interval %d :ex_control %d\n",
			no_daemon, thd_poll_interval, exclusive_control);

	thd_engine = new cthd_engine_default();
	if (exclusive_control)
		thd_engine->set_control_mode(EXCLUSIVE);

	thd_engine->set_poll_interval(thd_poll_interval);

	// Initialize thermald objects
	if (thd_engine->thd_engine_start(false) != THD_SUCCESS) {
		thd_log_error("thermald engine start failed:\n");
		exit(EXIT_FAILURE);
	}
#endif
#ifdef VALGRIND_TEST
	// lots of STL lib function don't free memory
	// when called with exit().
	// Here just run for some time and gracefully return.
/*	sleep(10);
	if (pid_file_handle)
		close(pid_file_handle);
	thd_engine->thd_engine_terminate();
	sleep(1);
	delete thd_engine;*/
#else
    int sensor_temp, cur_state, new_state;

	daemonize((char *) "/tmp/", (char *) "/tmp/thermald.pid");

	thd_log_info("Linux Thermal Daemon is starting\n");
	cur_state = -1;// 0 is normal, 1 is warning, 2 is alert, 3 is critical
	if(get_core_sensor() < 0){
		thd_log_info("Get coretemp1 sensor failed. Thermal Daemon Exit!\n");
		return 0;
	}

	for (;;) {
		//if (thd_engine)
		//	thd_engine->reinspect_max();
		thd_log_error("thermal dameon running, cur_state=%d!\n", cur_state);
		sensor_temp = read_sensor();
		new_state = calculate_state(sensor_temp);

		if(new_state != cur_state){
			throttle(cpu_freq[new_state], strlen(cpu_freq[new_state]));
			cur_state = new_state;
		}
		sleep(1);
	}

	thd_log_info("Linux Thermal Daemon is exiting \n");
#endif
	return 0;
}
