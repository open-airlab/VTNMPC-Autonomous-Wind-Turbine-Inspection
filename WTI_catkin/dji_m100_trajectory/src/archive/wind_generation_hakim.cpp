/**
 * @file   wind_generation.cpp
 * @author Mohit Mehndiratta
 * @date   July 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <random>
#include <dynamic_reconfigure/server.h>
#include <dji_m100_trajectory/set_wind_generationConfig.h>
#define PY_SSIZE_T_CLEAN
#include <Python.h>
using namespace std;
using namespace Eigen;

double sampleTime = 0.01;
bool wind_start, noise_on;
Vector3d wind_component;
int wind_type, noise_type;
double max_wind_force, mean_wind_force, wind_time_period, noise_stddev, noise_time_period;

void dynamicReconfigureCallback(dji_m100_trajectory::set_wind_generationConfig &config, uint32_t level)
{
    wind_start = config.wind_start;
    wind_component = {config.wind_component_x, config.wind_component_y, config.wind_component_z};
    wind_type = config.wind_type;
    max_wind_force = config.max_wind_force;
    mean_wind_force = config.mean_wind_force;
    wind_time_period = config.wind_time_period;
    noise_on = config.noise_on;
    noise_type = config.noise_type;
    noise_stddev = config.noise_stddev;
    noise_time_period = config.noise_time_period;
}
std_msgs::Bool trajectory_start_flag;
void trajectory_start_cb(const std_msgs::Bool::ConstPtr& msg)
{
    trajectory_start_flag = *msg;
}

geometry_msgs::Vector3 prepare_wind_msg(Eigen::Vector3d& wind)
{
    geometry_msgs::Vector3 _wind_msg;
    _wind_msg.x = wind(0);
    _wind_msg.y = wind(1);
    _wind_msg.z = wind(2);
    return _wind_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wind_generation");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<dji_m100_trajectory::set_wind_generationConfig> server;
    dynamic_reconfigure::Server<dji_m100_trajectory::set_wind_generationConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    std::string trajectory_on_sub_topic, wind_pub_topic;

    nh.param("trajectory_on_sub_topic", trajectory_on_sub_topic, std::string("/trajectory_on"));
    nh.param("wind_pub_topic", wind_pub_topic, std::string("/wind_3d"));

    ros::Subscriber trajectory_start_sub = nh.subscribe<std_msgs::Bool>(trajectory_on_sub_topic, 1, trajectory_start_cb);
    ros::Publisher wind_pub = nh.advertise<geometry_msgs::Vector3>(wind_pub_topic, 1, true);

    ros::Rate rate(1/sampleTime);

    Vector3d wind_magnitude;
    geometry_msgs::Vector3 wind_msg;

    double t, t_last, t_loop, t_last_noise, t_loop_noise;
    int print_flag_traj_start = 1, print_flag_wind_start = 1,
        print_flag_const = 1, print_flag_sinus = 1, print_flag_sinus_comb1 = 1,
        print_flag_sinus_comb2 = 1, print_flag_sinus_comb3 = 1,
        print_flag_Gauss_noise = 1, print_flag_sine_noise = 1, print_flag_comb_noise = 1;

    std::default_random_engine rand_seed;
    rand_seed.seed(std::time(0));

    for (int i=0;i<(int)1/sampleTime;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        if (wind_start && trajectory_start_flag.data)
        {
            t = ros::Time::now().toSec();
            t_loop = t - t_last;
            t_loop_noise = t - t_last_noise;

            switch (wind_type) {
            case 0: // Constant wind force
                if (print_flag_const == 1)
                {
                    ROS_INFO("--------Constant wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 0;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array()*max_wind_force;
                break;

            case 1: // Sinusoidal wind force
                if (print_flag_sinus == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 0;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array()*mean_wind_force +
                                 wind_component.array()*max_wind_force*sin((2*M_PI/wind_time_period)*t_loop);
                break;

            case 2: // Sinusoidal combination wind force type 1
                if (print_flag_sinus_comb1 == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal combination (type 1) wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 0;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array()*mean_wind_force +
                                 wind_component.array()*max_wind_force*
                                 (std::pow(sin((2*M_PI/wind_time_period)*t_loop), 2) +
                                  sin((2*M_PI/wind_time_period)*t_loop));
                break;

            case 3: // Sinusoidal combination wind force type 2
                if (print_flag_sinus_comb2 == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal combination (type 2) wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 0;
                    print_flag_sinus_comb3 = 1;
                }
                wind_magnitude = wind_component.array()*mean_wind_force +
                                 wind_component.array()*max_wind_force*
                                 (0.5*std::pow(sin((4*M_PI/wind_time_period)*t_loop), 4) +
                                  std::pow(cos((2*M_PI/wind_time_period)*(t_loop)), 3) +
                                  std::pow(sin((2*M_PI/wind_time_period)*t_loop), 2) +
                                  sin((2*M_PI/wind_time_period)*t_loop));
                break;
            // Now its wind_model velocity
            /////
            /////
            /////
            /////
            case 4: // Sinusoidal combination wind force type 3
                if (print_flag_sinus_comb3 == 1)
                {
                    t_last = ros::Time::now().toSec();
                    t_loop = t - t_last;

                    ROS_INFO("--------Sinusoidal combination (type 3) wind force selected!--------");
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_const = 1;
                    print_flag_sinus = 1;
                    print_flag_sinus_comb1 = 1;
                    print_flag_sinus_comb2 = 1;
                    print_flag_sinus_comb3 = 0;
                }
                const char* n_argv[] = { "/wind_generation.exe", "wind_model", "wind_velocity"};
                argv = n_argv;
                argc=3;
                PyObject *pName, *pModule, *pFunc;
                PyObject *pArgs, *pValue;
                int i;
                Py_Initialize();
                pName = PyUnicode_DecodeFSDefault(argv[1]);
                pModule = PyImport_Import(pName);
                Py_DECREF(pName);
                    pFunc = PyObject_GetAttrString(pModule, argv[2]);
                    /* pFunc is a new reference */
                    if (pFunc && PyCallable_Check(pFunc)) {
                        pArgs = PyTuple_New(argc - 3);
                        for (i = 0; i < argc - 3; ++i) {
                            pValue = PyLong_FromLong(atoi(argv[i + 3]));
                            PyTuple_SetItem(pArgs, i, pValue);
                        }
                        pValue = PyObject_CallObject(pFunc, pArgs);
                        Py_DECREF(pArgs);
                        if (pValue != NULL) {
                            //printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                            wind_magnitude=wind_component.array()*PyFloat_AsDouble(pValue);
                            Py_DECREF(pValue);       
                        }
                    }                 
                break;
            default:
                break;
            }

            if (noise_on)
            {
                switch (noise_type) {
                case 0:
                {
                    if (print_flag_Gauss_noise == 1)
                    {
                        ROS_INFO("--------Gaussian noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 0;
                        print_flag_sine_noise = 1;
                        print_flag_comb_noise = 1;
                    }
                    std::normal_distribution<double> disturb(0, noise_stddev);
                    wind_magnitude.array() += wind_component.array()*disturb(rand_seed);
                    break;
                }

                case 1:
                {
                    if (print_flag_sine_noise == 1)
                    {
                        t_last_noise = ros::Time::now().toSec();
                        t_loop_noise = t - t_last_noise;

                        ROS_INFO("--------Sinusoidal noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 1;
                        print_flag_sine_noise = 0;
                        print_flag_comb_noise = 1;
                    }
                    wind_magnitude.array() += wind_component.array()*(noise_stddev*sin((2*M_PI/noise_time_period)*t_loop_noise));
                    break;
                }

                case 2:
                {
                    if (print_flag_comb_noise == 1)
                    {
                        t_last_noise = ros::Time::now().toSec();
                        t_loop_noise = t - t_last_noise;

                        ROS_INFO("--------Combination of Gaussian and sinusoidal noise added!--------");
                        print_flag_traj_start = 1;
                        print_flag_wind_start = 1;
                        print_flag_Gauss_noise = 1;
                        print_flag_sine_noise = 1;
                        print_flag_comb_noise = 0;
                    }
                    std::normal_distribution<double> disturb(0, noise_stddev);
                    wind_magnitude.array() += wind_component.array()*(
                                                    disturb(rand_seed) +
                                                    noise_stddev*sin((2*M_PI/noise_time_period)*t_loop_noise));
                    break;
                }

                default:
                    break;
                }
            }
            else
            {
                if (print_flag_Gauss_noise == 0 || print_flag_sine_noise == 0 || print_flag_sine_noise == 0)
                {
                    print_flag_traj_start = 1;
                    print_flag_wind_start = 1;
                    print_flag_Gauss_noise = 1;
                    print_flag_sine_noise = 1;
                    print_flag_comb_noise = 1;
                }

            }
        }
        else
        {
            if (!trajectory_start_flag.data && print_flag_traj_start == 1)
            {
                ROS_INFO("Waiting for trajectory start switch to begin!");
                print_flag_traj_start = 0;
            }
            else if(trajectory_start_flag.data && print_flag_traj_start == 0)
            {
                std::cout<<"traj_start --> 1\n";
                print_flag_traj_start = 1;
            }

            if (!wind_start && print_flag_wind_start == 1)
            {
                ROS_INFO("Waiting for wind start switch to begin!");
                print_flag_wind_start = 0;
            }
            else if(wind_start && print_flag_wind_start == 0)
            {
                std::cout<<"wind_start --> 1\n";
                print_flag_wind_start = 1;
            }
            wind_magnitude.setZero();
            print_flag_const = 1;
            print_flag_sinus = 1;
            print_flag_sinus_comb1 = 1;
            print_flag_sinus_comb2 = 1;
            print_flag_sinus_comb3 = 1;
            print_flag_Gauss_noise = 1;
            print_flag_sine_noise = 1;
            print_flag_comb_noise = 1;

        }

        wind_msg = prepare_wind_msg(wind_magnitude);
        wind_pub.publish(wind_msg);

        ros::spinOnce();
        rate.sleep();
    }
  return 0;
}
