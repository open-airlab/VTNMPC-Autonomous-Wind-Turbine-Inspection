#pragma once

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include "gp.h"
#include "gp_sparse.h"
#include "gp_utils.h"
#include "rprop.h"
#include "cg.h"
//#include <iomanip>

//extern double sampleTime;

using namespace libgp;

class GP_DISTURB_REG
{
	private:

		double sampleTime;
		int switch_xyz;
		bool is_prediction_init;
		size_t buffer_cnt;
		int input_dim;
		

		std::string final_covfun;
		
		struct compute_struct_
		{
			std::vector<double> g_k;
			std::vector<double> g_km1;
		} compute_struct;

		void create_gp_initial(ros::Rate rate,
                                       std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                                       std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz);

	public:
		struct data_struct_
		{
			double m;
			double g;
			double input_0;
			double mu_0;
			double var_0;
			std::string predInit_pub_topic;
			std::string mu_pub_topic;
			std::string var_pub_topic;
			std::string mu_p_2std_dev_topic;
			std::string mu_m_2std_dev_topic;
		} data_struct;
		
		struct gp_struct_
        	{
			std::string file_gp_initial;
			bool create_new_gp_initial;
			bool use_gp_initial;
			int gp_type;
			size_t data_history;
			int max_initial_rec_time_sec;
			size_t num_window_points_past;
			size_t num_predict_points_future;
			double rec_rate_factor;
			bool use_sparse_gp_initial;
			double sizefactor_sparse_gp_initial;	// Max = 1
          		Eigen::VectorXd ell_0;
		    	double sf_0;
		    	double sn_0;
			Eigen::VectorXd sx_0;
		    	double grad_updates;
		    	std::string covfun1;
		    	std::string covfun2;
                    	std::string hyper_optimize_method;

            	    	GaussianProcess *gp_initial, *gp, *gp_futureWindow;
                        SparseGaussianProcess *sgp_initial, *sgp_futureWindow;

      			Eigen::VectorXd params;

		} gp_struct;

		struct standardize_struct_
		{
			Eigen::VectorXd mean_inputs;
			Eigen::VectorXd std_dev_inputs;
			double mean_targets;
			double std_dev_targets;
		}standardize_struct;

		struct reg_struct_
		{
			std::vector<double> trainInput_km1;
			double trainTarget_k;
			std::vector<double> testInput_k;
		} reg_struct;
	
		struct meas_struct_
		{
			std::vector<double> vel_k;
			std::vector<double> vel_km1;
			std::vector<double> rate_k;
			std::vector<double> rate_km1;
			std::vector<double> control_k;
			std::vector<double> control_km1;
		} meas_struct;

		struct predict_struct_
		{
			std::vector<double> mu;
			std::vector<double> mu_unstandardized;
			std::vector<double> var;
			std::vector<double> var_unstandardized;
			std::vector<double> mu_p_2std_dev;
			std::vector<double> mu_m_2std_dev;
		} predict_struct;


		GP_DISTURB_REG(double samTime, int _switch_xyz,
                               struct data_struct_ _data_struct, struct gp_struct_ _gp_struct,
                               ros::Rate rate, std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                               std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz);
		~GP_DISTURB_REG();

		bool return_prediction_init_value();

		void reg_init(ros::Rate rate,
                              std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                              std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz);

		void reg_core(std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                              std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz);

		void publish_muVar();

	protected:
		ros::NodeHandle private_nh;

		void set_measurements(std::vector<double> &_current_vel_rates, std::vector<double> &_nmpc_cmd_rpy,
                                      std::vector<double> &_nmpc_cmd_Fz);
		void compute_disturb(std::vector<double> &_current_acc);
		void prepare_reg_data(std::vector<double> &trainInputvec,
                                      std::vector<double> &testInputvec,
                                      double &trainTargetscalar, std::vector<double> &g_k, std::vector<double> &g_km1);
		void prepare_standardized_reg_data(std::vector<double> &trainInputvec,
                                      		   std::vector<double> &testInputvec,
                                      		   double &trainTargetscalar, std::vector<double> &g_k, std::vector<double> &g_km1);
		void add_pattern(GaussianProcess*& _gp, std::vector<double> &trainInputvec, double &trainTargetscalar);
		void add_pattern(GaussianProcess*& _gp, const Eigen::VectorXd &trainInputvec, double trainTargetscalar);
		void add_pattern_withRemoval(size_t idx, GaussianProcess*& _gp, std::vector<double> &trainInputvec, double &trainTargetscalar);
		void merge_gp(GaussianProcess*& gp_1, GaussianProcess*& gp_2);
		void compute_muVar(size_t idx, double points[], GaussianProcess*& _gp);
		void compute_muVar(size_t idx, double points[], GaussianProcess*& _gp, SparseGaussianProcess*& _sgp);
		void compute_muVar_noisyInputs(size_t idx, double points[], GaussianProcess*& _gp, Eigen::VectorXd &sx2);
		void compute_muVar_noisyInputs(size_t idx, double points[], GaussianProcess*& _gp, SparseGaussianProcess*& _sgp, Eigen::VectorXd &sx2);
		void standardize_optimize(GaussianProcess*& _gp);

		// Publishers
		ros::Publisher predInit_pub;
		ros::Publisher mu_pub;
		ros::Publisher var_pub;
		ros::Publisher mu_p_2std_dev_pub;
		ros::Publisher mu_m_2std_dev_pub;

};
