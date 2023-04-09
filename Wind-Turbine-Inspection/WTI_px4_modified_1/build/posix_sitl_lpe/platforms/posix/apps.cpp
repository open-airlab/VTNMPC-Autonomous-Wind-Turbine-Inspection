/* definitions of builtin command list - automatically generated, do not edit */
#include "px4_posix.h"
#include "px4_log.h"

#include "apps.h"

#include <cstdio>
#include <map>
#include <string>

#include <cstdlib>

extern "C" {

int uorb_main(int argc, char *argv[]);
int ets_airspeed_main(int argc, char *argv[]);
int ms4525_airspeed_main(int argc, char *argv[]);
int ms5525_airspeed_main(int argc, char *argv[]);
int sdp3x_airspeed_main(int argc, char *argv[]);
int ll40ls_main(int argc, char *argv[]);
int mb12xx_main(int argc, char *argv[]);
int sf0x_main(int argc, char *argv[]);
int sf1xx_main(int argc, char *argv[]);
int srf02_main(int argc, char *argv[]);
int teraranger_main(int argc, char *argv[]);
int tfmini_main(int argc, char *argv[]);
int ulanding_radar_main(int argc, char *argv[]);
int leddar_one_main(int argc, char *argv[]);
int vl53lxx_main(int argc, char *argv[]);
int batt_smbus_main(int argc, char *argv[]);
int camera_trigger_main(int argc, char *argv[]);
int gps_main(int argc, char *argv[]);
int pwm_out_sim_main(int argc, char *argv[]);
int vmount_main(int argc, char *argv[]);
int sensors_main(int argc, char *argv[]);
int tone_alarm_main(int argc, char *argv[]);
int esc_calib_main(int argc, char *argv[]);
int led_control_main(int argc, char *argv[]);
int mixer_main(int argc, char *argv[]);
int motor_ramp_main(int argc, char *argv[]);
int param_main(int argc, char *argv[]);
int perf_main(int argc, char *argv[]);
int pwm_main(int argc, char *argv[]);
int reboot_main(int argc, char *argv[]);
int sd_bench_main(int argc, char *argv[]);
int top_main(int argc, char *argv[]);
int listener_main(int argc, char *argv[]);
int tune_control_main(int argc, char *argv[]);
int ver_main(int argc, char *argv[]);
int sf0x_tests_main(int argc, char *argv[]);
int rc_tests_main(int argc, char *argv[]);
int commander_tests_main(int argc, char *argv[]);
int controllib_test_main(int argc, char *argv[]);
int mavlink_tests_main(int argc, char *argv[]);
int mc_pos_control_tests_main(int argc, char *argv[]);
int uorb_tests_main(int argc, char *argv[]);
int tests_main(int argc, char *argv[]);
int hello_main(int argc, char *argv[]);
int hrt_test_main(int argc, char *argv[]);
int muorb_test_main(int argc, char *argv[]);
int vcdev_test_main(int argc, char *argv[]);
int commander_main(int argc, char *argv[]);
int send_event_main(int argc, char *argv[]);
int land_detector_main(int argc, char *argv[]);
int load_mon_main(int argc, char *argv[]);
int mavlink_main(int argc, char *argv[]);
int navigator_main(int argc, char *argv[]);
int replay_main(int argc, char *argv[]);
int simulator_main(int argc, char *argv[]);
int accelsim_main(int argc, char *argv[]);
int measairspeedsim_main(int argc, char *argv[]);
int barosim_main(int argc, char *argv[]);
int gpssim_main(int argc, char *argv[]);
int gyrosim_main(int argc, char *argv[]);
int attitude_estimator_q_main(int argc, char *argv[]);
int ekf2_main(int argc, char *argv[]);
int local_position_estimator_main(int argc, char *argv[]);
int position_estimator_inav_main(int argc, char *argv[]);
int wind_estimator_main(int argc, char *argv[]);
int fw_att_control_main(int argc, char *argv[]);
int fw_pos_control_l1_main(int argc, char *argv[]);
int gnd_att_control_main(int argc, char *argv[]);
int gnd_pos_control_main(int argc, char *argv[]);
int mc_att_control_main(int argc, char *argv[]);
int mc_pos_control_main(int argc, char *argv[]);
int vtol_att_control_main(int argc, char *argv[]);
int logger_main(int argc, char *argv[]);
int sdlog2_main(int argc, char *argv[]);
int dataman_main(int argc, char *argv[]);
int landing_target_estimator_main(int argc, char *argv[]);
int bottle_drop_main(int argc, char *argv[]);
int rover_steering_control_main(int argc, char *argv[]);
int uuv_example_app_main(int argc, char *argv[]);
int segway_main(int argc, char *argv[]);
int px4_simple_app_main(int argc, char *argv[]);
int px4_mavlink_debug_main(int argc, char *argv[]);
int ex_fixedwing_control_main(int argc, char *argv[]);
int module_main(int argc, char *argv[]);

int shutdown_main(int argc, char *argv[]);
int list_tasks_main(int argc, char *argv[]);
int list_files_main(int argc, char *argv[]);
int list_devices_main(int argc, char *argv[]);
int list_topics_main(int argc, char *argv[]);
int sleep_main(int argc, char *argv[]);
int wait_for_topic(int argc, char *argv[]);

}

extern void px4_show_devices(void);

void init_app_map(apps_map_type &apps)
{
	apps["uorb"] = uorb_main;
	apps["ets_airspeed"] = ets_airspeed_main;
	apps["ms4525_airspeed"] = ms4525_airspeed_main;
	apps["ms5525_airspeed"] = ms5525_airspeed_main;
	apps["sdp3x_airspeed"] = sdp3x_airspeed_main;
	apps["ll40ls"] = ll40ls_main;
	apps["mb12xx"] = mb12xx_main;
	apps["sf0x"] = sf0x_main;
	apps["sf1xx"] = sf1xx_main;
	apps["srf02"] = srf02_main;
	apps["teraranger"] = teraranger_main;
	apps["tfmini"] = tfmini_main;
	apps["ulanding_radar"] = ulanding_radar_main;
	apps["leddar_one"] = leddar_one_main;
	apps["vl53lxx"] = vl53lxx_main;
	apps["batt_smbus"] = batt_smbus_main;
	apps["camera_trigger"] = camera_trigger_main;
	apps["gps"] = gps_main;
	apps["pwm_out_sim"] = pwm_out_sim_main;
	apps["vmount"] = vmount_main;
	apps["sensors"] = sensors_main;
	apps["tone_alarm"] = tone_alarm_main;
	apps["esc_calib"] = esc_calib_main;
	apps["led_control"] = led_control_main;
	apps["mixer"] = mixer_main;
	apps["motor_ramp"] = motor_ramp_main;
	apps["param"] = param_main;
	apps["perf"] = perf_main;
	apps["pwm"] = pwm_main;
	apps["reboot"] = reboot_main;
	apps["sd_bench"] = sd_bench_main;
	apps["top"] = top_main;
	apps["listener"] = listener_main;
	apps["tune_control"] = tune_control_main;
	apps["ver"] = ver_main;
	apps["sf0x_tests"] = sf0x_tests_main;
	apps["rc_tests"] = rc_tests_main;
	apps["commander_tests"] = commander_tests_main;
	apps["controllib_test"] = controllib_test_main;
	apps["mavlink_tests"] = mavlink_tests_main;
	apps["mc_pos_control_tests"] = mc_pos_control_tests_main;
	apps["uorb_tests"] = uorb_tests_main;
	apps["tests"] = tests_main;
	apps["hello"] = hello_main;
	apps["hrt_test"] = hrt_test_main;
	apps["muorb_test"] = muorb_test_main;
	apps["vcdev_test"] = vcdev_test_main;
	apps["commander"] = commander_main;
	apps["send_event"] = send_event_main;
	apps["land_detector"] = land_detector_main;
	apps["load_mon"] = load_mon_main;
	apps["mavlink"] = mavlink_main;
	apps["navigator"] = navigator_main;
	apps["replay"] = replay_main;
	apps["simulator"] = simulator_main;
	apps["accelsim"] = accelsim_main;
	apps["measairspeedsim"] = measairspeedsim_main;
	apps["barosim"] = barosim_main;
	apps["gpssim"] = gpssim_main;
	apps["gyrosim"] = gyrosim_main;
	apps["attitude_estimator_q"] = attitude_estimator_q_main;
	apps["ekf2"] = ekf2_main;
	apps["local_position_estimator"] = local_position_estimator_main;
	apps["position_estimator_inav"] = position_estimator_inav_main;
	apps["wind_estimator"] = wind_estimator_main;
	apps["fw_att_control"] = fw_att_control_main;
	apps["fw_pos_control_l1"] = fw_pos_control_l1_main;
	apps["gnd_att_control"] = gnd_att_control_main;
	apps["gnd_pos_control"] = gnd_pos_control_main;
	apps["mc_att_control"] = mc_att_control_main;
	apps["mc_pos_control"] = mc_pos_control_main;
	apps["vtol_att_control"] = vtol_att_control_main;
	apps["logger"] = logger_main;
	apps["sdlog2"] = sdlog2_main;
	apps["dataman"] = dataman_main;
	apps["landing_target_estimator"] = landing_target_estimator_main;
	apps["bottle_drop"] = bottle_drop_main;
	apps["rover_steering_control"] = rover_steering_control_main;
	apps["uuv_example_app"] = uuv_example_app_main;
	apps["segway"] = segway_main;
	apps["px4_simple_app"] = px4_simple_app_main;
	apps["px4_mavlink_debug"] = px4_mavlink_debug_main;
	apps["ex_fixedwing_control"] = ex_fixedwing_control_main;
	apps["module"] = module_main;

	apps["shutdown"] = shutdown_main;
	apps["list_tasks"] = list_tasks_main;
	apps["list_files"] = list_files_main;
	apps["list_devices"] = list_devices_main;
	apps["list_topics"] = list_topics_main;
	apps["sleep"] = sleep_main;
	apps["wait_for_topic"] = wait_for_topic;
}

void list_builtins(apps_map_type &apps)
{
	printf("Builtin Commands:\n");
	for (apps_map_type::iterator it = apps.begin(); it != apps.end(); ++it) {
		printf("  %s\n", it->first.c_str());
	}
}

int shutdown_main(int argc, char *argv[])
{
	printf("Shutting down\n");
	exit(0);
}

int list_tasks_main(int argc, char *argv[])
{
	px4_show_tasks();
	return 0;
}

int list_devices_main(int argc, char *argv[])
{
	px4_show_devices();
	return 0;
}

int list_topics_main(int argc, char *argv[])
{
	px4_show_topics();
	return 0;
}

int list_files_main(int argc, char *argv[])
{
	px4_show_files();
	return 0;
}

int sleep_main(int argc, char *argv[])
{
        if (argc != 2) {
           PX4_WARN( "Usage: sleep <seconds>" );
           return 1;
        }

        unsigned long usecs = 1000000UL * atol(argv[1]);
        printf("Sleeping for %s s; (%lu us).\n", argv[1], usecs);
        usleep(usecs);
        return 0;
}

#include "uORB/uORB.h"

int wait_for_topic(int argc, char *argv[])
{
	if (argc < 2 || argc > 3) {
		printf("Usage: wait_for_topic <topic> [timeout_sec]\n");
		return 1;
	}

	struct orb_metadata meta = { .o_name = argv[1],
				     .o_size = 0,
				     .o_size_no_padding = 0,
				     .o_fields = nullptr };

	unsigned int timeout = (argc == 3) ? (unsigned int)atoi(argv[2]) : 0;
	unsigned int elapsed = 0;

	printf("Waiting for topic %s timeout %u\n", argv[1], timeout);

	while (orb_exists(&meta, 0) != 0 && (timeout && (elapsed < timeout)))
	{
		sleep(1);
		elapsed += 1;
	}

	return 0;
}

