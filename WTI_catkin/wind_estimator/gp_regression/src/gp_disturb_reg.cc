/**
 * @file   gp_disturb_reg.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include "gp_disturb_reg.h"

using namespace libgp;
using namespace std;

//double sampleTime = 0.02;

GP_DISTURB_REG::GP_DISTURB_REG(double samTime, int _switch_xyz,
                               struct data_struct_ _data_struct, struct gp_struct_ _gp_struct,
                               ros::Rate rate, std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                               std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz)
{
    sampleTime = samTime;
    switch_xyz = _switch_xyz;
    is_prediction_init = false;
    buffer_cnt = 0;

    data_struct = _data_struct;
    gp_struct = _gp_struct;

    final_covfun = "CovSum ( " + gp_struct.covfun1 + ", " +
                                 gp_struct.covfun2 + ")";
    std::cout<<"final_covfun = "<<final_covfun.c_str()<<"\n";

    // ----------
    // Publishers
    // ----------
    predInit_pub = private_nh.advertise<std_msgs::Bool>(data_struct.predInit_pub_topic, 1, true);
    mu_pub = private_nh.advertise<std_msgs::Float64MultiArray>(data_struct.mu_pub_topic, 1, true);
    var_pub = private_nh.advertise<std_msgs::Float64MultiArray>(data_struct.var_pub_topic, 1, true);
    mu_p_2std_dev_pub = private_nh.advertise<std_msgs::Float64MultiArray>(data_struct.mu_p_2std_dev_topic, 1, true);
    mu_m_2std_dev_pub = private_nh.advertise<std_msgs::Float64MultiArray>(data_struct.mu_m_2std_dev_topic, 1, true);

    switch (gp_struct.gp_type) {
    case 0:     // inputs = data*data_history
        input_dim = gp_struct.data_history;
        compute_struct.g_k.resize(1, 0);
        compute_struct.g_km1.resize(1, 0);
        break;

    case 1:     // inputs = data*data_history, del_data*data_history
        input_dim = 2*gp_struct.data_history;
        compute_struct.g_k.resize(2, 0);
        compute_struct.g_km1.resize(2, 0);
        break;

    default:    // inputs = data*data_history, del_data*data_history
        input_dim = gp_struct.data_history;
        compute_struct.g_k.resize(1, 0);
        compute_struct.g_km1.resize(1, 0);
        break;
    }

    for (int i=0; i<input_dim; ++i)
    {
        reg_struct.trainInput_km1.push_back(data_struct.input_0);
        reg_struct.testInput_k.push_back(data_struct.input_0);
    }
    reg_struct.trainTarget_k = data_struct.input_0;

    meas_struct.vel_k = {0, 0, 0};
    meas_struct.vel_km1 = {0, 0, 0};
    meas_struct.rate_k = {0, 0, 0};
    meas_struct.rate_km1 = {0, 0, 0};
    meas_struct.control_k = {0, 0, 0, 0};
    meas_struct.control_km1 = {0, 0, 0, 0};

    for (int i=0; i<gp_struct.num_predict_points_future; ++i)
    {
        predict_struct.mu.push_back(data_struct.mu_0);
        predict_struct.mu_unstandardized.push_back(data_struct.mu_0);
        predict_struct.var.push_back(data_struct.var_0);
        predict_struct.var_unstandardized.push_back(data_struct.var_0);
        predict_struct.mu_p_2std_dev.push_back(data_struct.mu_0 + 2*sqrt(data_struct.var_0));
        predict_struct.mu_m_2std_dev.push_back(data_struct.mu_0 - 2*sqrt(data_struct.var_0));
    }

    create_gp_initial(rate, _current_vel_rates, _current_acc,
                      _nmpc_cmd_rpy, _nmpc_cmd_Fz);
    if (!gp_struct.use_sparse_gp_initial)
        gp_struct.gp = new GaussianProcess(*gp_struct.gp_initial);
    else
    {
        gp_struct.sgp_initial = new SparseGaussianProcess(gp_struct.sizefactor_sparse_gp_initial*
                                                            gp_struct.gp_initial->get_sampleset_size(),
                                                          *gp_struct.gp_initial);

        gp_struct.sgp_initial->update_params_noisy_inp(gp_struct.sx_0);
        gp_struct.sgp_initial->sparsify();
        gp_struct.gp = new GaussianProcess(input_dim, final_covfun);
//        gp_struct.gp_initial->set_sampleset(*gp_struct.sgp_initial->get_sampleset());
//        gp_struct.gp = new GaussianProcess(*gp_struct.gp_initial);

        ROS_INFO_STREAM("Using sparse_gp_initial!");
    }

//    // TO BE: try better way of obtaining them
//    for (size_t i; i<gp_struct.num_predict_points_future; ++i)
//        gp_struct.sx_0(i) = exp(gp_struct.gp_initial->covf().get_loghyper()(gp_struct.gp_initial->covf().get_param_dim()-1));

    ROS_INFO_STREAM("Constructor of the class GP_DISTURB_REG is created");
}

GP_DISTURB_REG::~GP_DISTURB_REG()
{
    ROS_INFO_STREAM("Destructor of the class GP_DISTURB_REG");
}

bool GP_DISTURB_REG::return_prediction_init_value()
{
    return is_prediction_init;
}

void GP_DISTURB_REG::create_gp_initial(ros::Rate rate,
                                       std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                                       std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz)
{
    if (gp_struct.create_new_gp_initial)
    {
        ROS_INFO_STREAM("Creating gp_initial from training data");

        gp_struct.gp_initial = new GaussianProcess(input_dim, final_covfun);
        gp_struct.params.resize(gp_struct.gp_initial->covf().get_param_dim());
        for (int i=0; i<input_dim; ++i)
            gp_struct.params(i) = log(gp_struct.ell_0(i));
        gp_struct.params(input_dim) = log(gp_struct.sf_0);
        gp_struct.params(input_dim+1) = log(gp_struct.sn_0);
        gp_struct.gp_initial->covf().set_loghyper(gp_struct.params);

        double t_last = ros::Time::now().toSec();
        double t_loop = ros::Time::now().toSec() - t_last;

        int count = 0;

        if (gp_struct.use_gp_initial)
        {
            while(ros::ok() && t_loop<=gp_struct.max_initial_rec_time_sec)
            {
                set_measurements(_current_vel_rates, _nmpc_cmd_rpy, _nmpc_cmd_Fz);
                compute_disturb(_current_acc);
                prepare_reg_data(reg_struct.trainInput_km1, reg_struct.testInput_k,
                                 reg_struct.trainTarget_k, compute_struct.g_k, compute_struct.g_km1);
                if (count > gp_struct.data_history)
                    add_pattern(gp_struct.gp_initial, reg_struct.trainInput_km1, reg_struct.trainTarget_k);

                if (std::fmod((double)(t_loop - std::floor(t_loop)), (double)(1)) < sampleTime)
                    std::cout<<"Training time for gp_initial: " << (int)t_loop << " (sec)"<<"\n";

                t_loop = ros::Time::now().toSec() - t_last;
                count++;
                ros::spinOnce();
                rate.sleep();
            }

            standardize_optimize(gp_struct.gp_initial);
//            // Standardization of the training data
//            // TO BE: select the most applicable one
//            gp_struct.gp_initial->standardize_sampleset(standardize_struct.mean_inputs,standardize_struct.std_dev_inputs,
//                                                        standardize_struct.mean_targets,standardize_struct.std_dev_targets);

//            if(gp_struct.hyper_optimize_method == "CG")
//            {
//                libgp::CG cg;
//                cg.maximize(*&gp_struct.gp_initial, gp_struct.grad_updates, 0);
//            }
//            else if(gp_struct.hyper_optimize_method == "RProp")
//            {
//                libgp::RProp rprop;
//                rprop.maximize(*&gp_struct.gp_initial, gp_struct.grad_updates, 0);
//            }
//    //        gp_struct.params = gp_struct.gp_initial->covf().get_loghyper();
//    //        gp_struct.params(gp_struct.params.size()-1) = log(0.1);
//    //        gp_struct.gp_initial->covf().set_loghyper(gp_struct.params);
//            std::cout<<"params_"<<gp_struct.hyper_optimize_method<<" = ";
//            for (int i=0; i<gp_struct.params.size(); ++i)
//                std::cout<<exp(gp_struct.gp_initial->covf().get_loghyper()(i))<<", ";
//            std::cout<<"\n";
        }
        else
            std::cout<<"gp_initial (LSTM) is not utilized for regression.\n";

        gp_struct.gp_initial->write(const_cast<char*>(gp_struct.file_gp_initial.c_str()));
    }
    else
    {
        ROS_INFO_STREAM("Creating gp_initial from file: " << gp_struct.file_gp_initial.c_str());

        gp_struct.gp_initial = new GaussianProcess(const_cast<char*>(gp_struct.file_gp_initial.c_str()));

        standardize_struct.mean_inputs = gp_struct.gp_initial->get_sampleset()->mean_x();
        standardize_struct.std_dev_inputs = gp_struct.gp_initial->get_sampleset()->std_dev_x();
        standardize_struct.mean_targets = gp_struct.gp_initial->get_sampleset()->mean_y();
        standardize_struct.std_dev_targets = gp_struct.gp_initial->get_sampleset()->std_dev_y();
    }
    ROS_INFO_STREAM("gp_initial created!");

//    std::cout<<"params_"<<gp_struct.hyper_optimize_method<<" = ";
//    for (int i=0; i<gp_struct.gp_initial->covf().get_param_dim(); ++i)
//        std::cout<<exp(gp_struct.gp_initial->covf().get_loghyper()(i))<<", ";
//    std::cout<<"\n";
//    std::cout<<"mean_inputs = "<<standardize_struct.mean_inputs.mean()<<"\n";
//    std::cout<<"std_dev_inputs = "<<standardize_struct.std_dev_inputs.mean()<<"\n";
//    std::cout<<"mean_targets = "<<standardize_struct.mean_targets<<"\n";
//    std::cout<<"std_dev_targets = "<<standardize_struct.std_dev_targets<<"\n";
}

void GP_DISTURB_REG::reg_init(ros::Rate rate,
                              std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                              std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz)
{
    if (gp_struct.use_gp_initial)
    {
        while(ros::ok() && buffer_cnt<gp_struct.num_window_points_past)
        {
            set_measurements(_current_vel_rates, _nmpc_cmd_rpy, _nmpc_cmd_Fz);
            compute_disturb(_current_acc);
            prepare_standardized_reg_data(reg_struct.trainInput_km1, reg_struct.testInput_k,
                                          reg_struct.trainTarget_k, compute_struct.g_k, compute_struct.g_km1);

            // add training patterns while initializing
            add_pattern(gp_struct.gp, reg_struct.trainInput_km1, reg_struct.trainTarget_k);

            // TO DO: check if can be deleted?
            for (int i=0; i<gp_struct.num_predict_points_future; ++i)
                predict_struct.mu[i] = reg_struct.trainTarget_k;
            predict_struct.mu_unstandardized = predict_struct.mu;
//            publish_muVar(predict_struct);

            buffer_cnt++;
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        while(ros::ok() && buffer_cnt<gp_struct.num_window_points_past)
        {
            set_measurements(_current_vel_rates, _nmpc_cmd_rpy, _nmpc_cmd_Fz);
            compute_disturb(_current_acc);
            prepare_reg_data(reg_struct.trainInput_km1, reg_struct.testInput_k,
                             reg_struct.trainTarget_k, compute_struct.g_k, compute_struct.g_km1);

            if (buffer_cnt > gp_struct.data_history)
                add_pattern(gp_struct.gp, reg_struct.trainInput_km1, reg_struct.trainTarget_k);

            // TO DO: check if can be deleted?
            for (int i=0; i<gp_struct.num_predict_points_future; ++i)
                predict_struct.mu[i] = reg_struct.trainTarget_k;
            predict_struct.mu_unstandardized = predict_struct.mu;
//            publish_muVar(predict_struct);

            buffer_cnt++;
            ros::spinOnce();
            rate.sleep();
        }
        standardize_optimize(gp_struct.gp);
    }

    if(buffer_cnt < gp_struct.num_window_points_past)
    {
        ROS_ERROR_STREAM("ERROR: Couldn't fill the buffer! ");
        exit(0);
    }
    else
    {
        gp_struct.gp->update_params_noisy_inp(gp_struct.sx_0);
        is_prediction_init = true;
    }

    ROS_INFO_STREAM("GP_DISTURB_REG is initialized correctly");
}

void GP_DISTURB_REG::reg_core(std::vector<double> &_current_vel_rates, std::vector<double> &_current_acc,
                              std::vector<double> &_nmpc_cmd_rpy, std::vector<double> &_nmpc_cmd_Fz)
{
    set_measurements(_current_vel_rates, _nmpc_cmd_rpy, _nmpc_cmd_Fz);

    compute_disturb(_current_acc);

    prepare_standardized_reg_data(reg_struct.trainInput_km1, reg_struct.testInput_k,
                                  reg_struct.trainTarget_k, compute_struct.g_k, compute_struct.g_km1);

    // add training patterns with removal from the front of the moving window
    if (!gp_struct.use_sparse_gp_initial)
    {
        add_pattern_withRemoval(gp_struct.gp_initial->get_sampleset_size(), gp_struct.gp,
                                reg_struct.trainInput_km1, reg_struct.trainTarget_k);
    }
    else
    {
        add_pattern_withRemoval(0, gp_struct.gp,
                                reg_struct.trainInput_km1, reg_struct.trainTarget_k);
    }

//    // Merge current gp_pastWindow with gp_inital
////    gp_struct.gp = merge_gp(gp_struct.gp_initial, gp_struct.gp_pastWindow);
//    merge_gp(gp_struct.gp, gp_struct.gp_pastWindow);

    std::vector<double> trainInput, testInput, g_k, g_km1;
    double trainTarget;

    trainInput = reg_struct.trainInput_km1;
    testInput = reg_struct.testInput_k;
    trainTarget = reg_struct.trainTarget_k;
    g_k = compute_struct.g_k;
    g_km1 = compute_struct.g_km1;


//    // To compute the paramters for noisy inputs: This just has to be done ONCE!
//    // TO DO: make it efficient
//    if (gp_struct.gp->get_hyperpar_needs_update())
//    {
//        double points[input_dim];
//        for (int col_idx=0; col_idx<input_dim; ++col_idx)
//            points[col_idx] = testInput[col_idx];
//        gp_struct.gp->f_noisy_inp(points, gp_struct.sx_0);
//    }
    gp_struct.gp_futureWindow = new GaussianProcess(*gp_struct.gp);

    // Adding recursive data to the rest of gp_futureWindow
    for(size_t row_idx = 0; row_idx < gp_struct.num_predict_points_future; ++row_idx)
    {
        double points[input_dim];
        for (int col_idx=0; col_idx<input_dim; ++col_idx)
            points[col_idx] = testInput[col_idx];

//        if (row_idx == 0)
//            std::cout<<"mu_kp1 = "<<gp_struct.gp_futureWindow->f(points)<<"\n";

        if (row_idx > 0)
        {
            if (!gp_struct.use_sparse_gp_initial)
                compute_muVar_noisyInputs(row_idx, points, gp_struct.gp_futureWindow, gp_struct.sx_0);
            else
                compute_muVar_noisyInputs(row_idx, points, gp_struct.gp_futureWindow, gp_struct.sgp_initial, gp_struct.sx_0);
        }
        else
        {
            if (!gp_struct.use_sparse_gp_initial)
                compute_muVar(row_idx, points, gp_struct.gp_futureWindow);
            else
                compute_muVar(row_idx, points, gp_struct.gp_futureWindow, gp_struct.sgp_initial);
        }

        // compute corresponding g_k and g_km1
        g_km1 = g_k;
        switch (gp_struct.gp_type) {
        case 0:
            g_k.at(0) = predict_struct.mu[row_idx];
//            g_km1.at(0) = trainTarget;
            break;

        case 1:
//            g_km1.at(0) = trainTarget;
            g_k.at(0) = predict_struct.mu[row_idx];

//            g_km1.at(1) = g_k.at(1);
            g_k.at(1) = g_k.at(0) - g_km1.at(0);
            break;

        default:
            break;
        }

//        prepare_reg_data(trainInput, testInput, trainTarget, predict_struct.mu[row_idx], trainTarget_last);
        prepare_reg_data(trainInput, testInput, trainTarget, g_k, g_km1);
        add_pattern(gp_struct.gp_futureWindow, trainInput, trainTarget);
    }

//    string path = ros::package::getPath("gp_regression") + "/data/gp/gp_futureWindow.dat";
//    gp_struct.gp_futureWindow->write(const_cast<char*>(path.c_str()));

    publish_muVar();
//    delete gp_struct.gp, gp_struct.gp_futureWindow;
    delete gp_struct.gp_futureWindow;
}

void GP_DISTURB_REG::set_measurements(std::vector<double> &_current_vel_rates, std::vector<double> &_nmpc_cmd_rpy,
                                      std::vector<double> &_nmpc_cmd_Fz)
{
    meas_struct.vel_km1 = meas_struct.vel_k;
    meas_struct.rate_km1 = meas_struct.rate_k;
    meas_struct.control_km1 = meas_struct.control_k;

    for (int i=0; i<meas_struct.vel_k.size(); ++i)
    {
        meas_struct.vel_k[i] = _current_vel_rates[i];
        meas_struct.rate_k[i] = _current_vel_rates[i+3];
        meas_struct.control_k[i] = _nmpc_cmd_rpy[i];
    }
    meas_struct.control_k[3] = _nmpc_cmd_Fz[0];

}

void GP_DISTURB_REG::compute_disturb(std::vector<double> &_current_acc)
{
//    if (gp_struct.gp_type == 1)
//        compute_struct.g_km1.at(1) = compute_struct.g_k.at(0) - compute_struct.g_km1.at(0);

//    compute_struct.g_km1.at(0) = compute_struct.g_k.at(0);
    compute_struct.g_km1 = compute_struct.g_k;
    switch (switch_xyz)
    {
        case 0:
            compute_struct.g_k.at(0) = ( _current_acc[0] -
                                       (meas_struct.rate_km1[2]*meas_struct.vel_km1[1] -
                                        meas_struct.rate_km1[1]*meas_struct.vel_km1[2] +
                                        data_struct.g*sin(meas_struct.control_km1[1])) );
            break;

        case 1:
            compute_struct.g_k.at(0) = ( _current_acc[1] -
                                       (meas_struct.rate_km1[0]*meas_struct.vel_km1[2] -
                                        meas_struct.rate_km1[2]*meas_struct.vel_km1[0] -
                                        data_struct.g*sin(meas_struct.control_km1[0])*cos(meas_struct.control_km1[1])) );
            break;

        case 2:
            compute_struct.g_k.at(0) = ( (_current_acc[2] - data_struct.g) -
                                       (meas_struct.rate_km1[1]*meas_struct.vel_km1[0] -
                                        meas_struct.rate_km1[0]*meas_struct.vel_km1[1] -
                                        data_struct.g*cos(meas_struct.control_km1[0])*cos(meas_struct.control_km1[1]) +
                                        (1.0/data_struct.m)*meas_struct.control_km1[3]) );
//            std::cout<<"_current_acc[2] = "<<_current_acc[2]<<"\n";
//            std::cout<<"first_term = "<<meas_struct.rate_km1[1]*meas_struct.vel_km1[0]<<"\n";
//            std::cout<<"second_term = "<<meas_struct.rate_km1[0]*meas_struct.vel_km1[1]<<"\n";
//            std::cout<<"third_term = "<<data_struct.g*cos(meas_struct.control_km1[0])*cos(meas_struct.control_km1[1])<<"\n";
//            std::cout<<"fourth_term = "<<(((double)1.0)/data_struct.m)*meas_struct.control_km1[3]<<"\n";
//            std::cout<<"Fz_dist = "<<compute_struct.g_k.at(0)<<"\n";
//            compute_struct.g_k.at(0) = meas_struct.control_km1[3] /
//                                      ( (_current_acc[2] - data_struct.g) -
//                                      (meas_struct.rate_km1[1]*meas_struct.vel_km1[0] -
//                                       meas_struct.rate_km1[0]*meas_struct.vel_km1[1] -
//                                       data_struct.g*cos(meas_struct.control_km1[0])*cos(meas_struct.control_km1[1])) );
            break;

        default:
            std::cout<<"Enter a valid switch_xyz number (x:1, y:2, z:3)\n";
            exit(0);
    }
    if (gp_struct.gp_type == 1)
    {
//        compute_struct.g_km1.at(1) = compute_struct.g_k.at(1);
        compute_struct.g_k.at(1) = compute_struct.g_k.at(0) - compute_struct.g_km1.at(0);
    }

}

void GP_DISTURB_REG::prepare_reg_data(std::vector<double> &trainInputvec,
                                      std::vector<double> &testInputvec,
                                      double &trainTargetscalar,
                                      std::vector<double> &g_k, std::vector<double> &g_km1)
{
    switch (gp_struct.gp_type) {
    case 0:
        trainInputvec.insert(trainInputvec.begin(), g_km1.at(0));
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(), g_k.at(0));
        testInputvec.pop_back();
        break;

    case 1:
        trainInputvec.insert(trainInputvec.begin(), g_km1.at(0));
        trainInputvec.erase(trainInputvec.begin()+gp_struct.data_history);
        trainInputvec.insert(trainInputvec.begin()+gp_struct.data_history, g_km1.at(1));
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(), g_k.at(0));
        testInputvec.erase(testInputvec.begin()+gp_struct.data_history);
        testInputvec.insert(testInputvec.begin()+gp_struct.data_history, g_k.at(1));
        testInputvec.pop_back();
        break;

    default:
        trainInputvec.insert(trainInputvec.begin(), g_km1.at(0));
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(), g_k.at(0));
        testInputvec.pop_back();
        break;
    }


    assert(trainInputvec.size() == input_dim &&
           testInputvec.size() == input_dim);

    trainTargetscalar = g_k.at(0);
}

// Standardized values are given as inputs.
void GP_DISTURB_REG::prepare_standardized_reg_data(std::vector<double> &trainInputvec,
                                                   std::vector<double> &testInputvec,
                                                   double &trainTargetscalar,
                                                   std::vector<double> &g_k, std::vector<double> &g_km1)
{

    switch (gp_struct.gp_type) {
    case 0:
        // Since all the dimensions of input take the same type of data, only one mean value is used here!
        trainInputvec.insert(trainInputvec.begin(),
                             (g_km1.at(0)-standardize_struct.mean_inputs.mean())/
                             standardize_struct.std_dev_inputs.mean());
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(),
                            (g_k.at(0)-standardize_struct.mean_inputs.mean())/
                            standardize_struct.std_dev_inputs.mean());
        testInputvec.pop_back();
        break;

    case 1:
        trainInputvec.insert(trainInputvec.begin(),
                             (g_km1.at(0)-standardize_struct.mean_inputs(0))/
                             standardize_struct.std_dev_inputs(0));
        trainInputvec.erase(trainInputvec.begin()+gp_struct.data_history);
        trainInputvec.insert(trainInputvec.begin()+gp_struct.data_history,
                             (g_km1.at(1)-standardize_struct.mean_inputs(gp_struct.data_history))/
                             standardize_struct.std_dev_inputs(gp_struct.data_history));
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(),
                            (g_k.at(0)-standardize_struct.mean_inputs(0))/
                            standardize_struct.std_dev_inputs(0));
        testInputvec.erase(testInputvec.begin()+gp_struct.data_history);
        testInputvec.insert(testInputvec.begin()+gp_struct.data_history,
                            (g_k.at(1)-standardize_struct.mean_inputs(gp_struct.data_history))/
                            standardize_struct.std_dev_inputs(gp_struct.data_history));
        testInputvec.pop_back();
        break;

    default:
        // Since all the dimensions of input take the same type of data, only one mean value is used here!
        trainInputvec.insert(trainInputvec.begin(),
                             (g_km1.at(0)-standardize_struct.mean_inputs.mean())/
                             standardize_struct.std_dev_inputs.mean());
        trainInputvec.pop_back();

        testInputvec.insert(testInputvec.begin(),
                            (g_k.at(0)-standardize_struct.mean_inputs.mean())/
                            standardize_struct.std_dev_inputs.mean());
        testInputvec.pop_back();
        break;
    }

    assert(trainInputvec.size() == input_dim &&
           testInputvec.size() == input_dim);

    trainTargetscalar = (g_k.at(0)-standardize_struct.mean_targets)/
                         standardize_struct.std_dev_targets;

}

void GP_DISTURB_REG::add_pattern(GaussianProcess*& _gp, std::vector<double> &trainInputvec,
                                 double &trainTargetscalar)
{
    // add training patterns
    double points[input_dim];
    for (int col_idx=0; col_idx<input_dim; ++col_idx)
        points[col_idx] = trainInputvec[col_idx];
    _gp->add_pattern(points,trainTargetscalar);
}
void GP_DISTURB_REG::add_pattern(GaussianProcess*& _gp, const Eigen::VectorXd &trainInputvec,
                                 double trainTargetscalar)
{
  // add training patterns
    double points[input_dim];
    for (int col_idx=0; col_idx<input_dim; ++col_idx)
        points[col_idx] = trainInputvec(col_idx);
    _gp->add_pattern(points,trainTargetscalar);
}

void GP_DISTURB_REG::add_pattern_withRemoval(size_t idx, GaussianProcess*& _gp,
                                             std::vector<double> &trainInputvec,
                                             double &trainTargetscalar)
{
    // Always remove data from the starting location of the moving window
//    _gp->get_sampleset()->remove(idx);
    _gp->remove_sample(idx);

    add_pattern(_gp, trainInputvec, trainTargetscalar);
}

//GaussianProcess* GP_DISTURB_REG::merge_gp(GaussianProcess*& gp_1, GaussianProcess*& gp_2)
//{
//    GaussianProcess *gp_12 = new GaussianProcess(*gp_1);
//    for (int i=0; i<gp_2->get_sampleset_size(); i++)
//    {
//        add_pattern(gp_12, gp_2->get_sampleset()->x(i), gp_2->get_sampleset()->y(i));
//    }
//    return gp_12;
//}
void GP_DISTURB_REG::merge_gp(GaussianProcess*& gp_1, GaussianProcess*& gp_2)
{
    for (int i=0; i<gp_2->get_sampleset_size(); i++)
    {
        add_pattern(gp_1, gp_2->get_sampleset()->x(i), gp_2->get_sampleset()->y(i));
    }
}

void GP_DISTURB_REG::compute_muVar(size_t idx, double points[], GaussianProcess*& _gp)
{
    predict_struct.mu[idx] = _gp->f(points);
    predict_struct.var[idx] = _gp->var(points);

    predict_struct.mu_unstandardized[idx] = predict_struct.mu[idx]*standardize_struct.std_dev_targets +
                                            standardize_struct.mean_targets;
    predict_struct.var_unstandardized[idx] = predict_struct.var[idx]*standardize_struct.std_dev_targets +
                                             standardize_struct.mean_targets;
    predict_struct.mu_p_2std_dev[idx] = predict_struct.mu_unstandardized[idx] + 2*sqrt(predict_struct.var_unstandardized[idx]);
    predict_struct.mu_m_2std_dev[idx] = predict_struct.mu_unstandardized[idx] - 2*sqrt(predict_struct.var_unstandardized[idx]);

}
void GP_DISTURB_REG::compute_muVar(size_t idx, double points[], GaussianProcess*& _gp, SparseGaussianProcess*&  _sgp)
{
    double gp_var = _gp->var(points), sgp_var =  _sgp->var(points);
    double sqrt_gp_var = sqrt(_gp->var(points)), sqrt_sgp_var = sqrt( _sgp->var(points));
    // TO BE: find a better combination of the two
//    predict_struct.mu[idx] = 0.3*gp->f(points) + 0.7* _sgp->f(points);
//    predict_struct.var[idx] = 0.3*gp->var(points) + 0.7* _sgp->var(points);
    predict_struct.mu[idx] = (sqrt_gp_var/(sqrt_gp_var+sqrt_sgp_var))*_gp->f(points) +
                             (sqrt_sgp_var/(sqrt_gp_var+sqrt_sgp_var))* _sgp->f(points);
    predict_struct.var[idx] = (sqrt_gp_var/(sqrt_gp_var+sqrt_sgp_var))*gp_var +
                              (sqrt_sgp_var/(sqrt_gp_var+sqrt_sgp_var))*sgp_var;

    predict_struct.mu_unstandardized[idx] = predict_struct.mu[idx]*standardize_struct.std_dev_targets +
                                            standardize_struct.mean_targets;
    predict_struct.var_unstandardized[idx] = predict_struct.var[idx]*standardize_struct.std_dev_targets +
                                             standardize_struct.mean_targets;
    predict_struct.mu_p_2std_dev[idx] = predict_struct.mu_unstandardized[idx] + 2*sqrt(predict_struct.var_unstandardized[idx]);
    predict_struct.mu_m_2std_dev[idx] = predict_struct.mu_unstandardized[idx] - 2*sqrt(predict_struct.var_unstandardized[idx]);
}

void GP_DISTURB_REG::compute_muVar_noisyInputs(size_t idx, double points[], GaussianProcess*& _gp,
                                               Eigen::VectorXd &sx2)
{
    predict_struct.mu[idx] = _gp->f_noisy_inp(points, sx2);

//    if (idx == _gp_struct.num_predict_points_future-1)
//    {
//        predict_struct.var[idx] = _gp->var_noisy_inp(points, predict_struct.mu[idx], sx2);
////        std::cout<<"predict_struct.var["<<idx<<"] = "<<predict_struct.var[idx]<<"\n";
//    }
//    else
        predict_struct.var[idx] = _gp->var(points);

    predict_struct.mu_unstandardized[idx] = predict_struct.mu[idx]*standardize_struct.std_dev_targets +
                                            standardize_struct.mean_targets;
    predict_struct.var_unstandardized[idx] = predict_struct.var[idx]*standardize_struct.std_dev_targets +
                                             standardize_struct.mean_targets;
    predict_struct.mu_p_2std_dev[idx] = predict_struct.mu_unstandardized[idx] + 2*sqrt(predict_struct.var_unstandardized[idx]);
    predict_struct.mu_m_2std_dev[idx] = predict_struct.mu_unstandardized[idx] - 2*sqrt(predict_struct.var_unstandardized[idx]);
}
void GP_DISTURB_REG::compute_muVar_noisyInputs(size_t idx, double points[], GaussianProcess*& _gp, SparseGaussianProcess*& _sgp,
                                               Eigen::VectorXd &sx2)
{
    double gp_var = _gp->var(points), sgp_var =  _sgp->var(points);
    double sqrt_gp_var = sqrt(_gp->var(points)), sqrt_sgp_var = sqrt( _sgp->var(points));

    // TO BE: find a better combination of the two
//    predict_struct.mu[idx] = 0.3*gp->f_noisy_inp(points, sx2) + 0.7* _sgp->f_noisy_inp(points, sx2);
    predict_struct.mu[idx] = (sqrt_gp_var/(sqrt_gp_var+sqrt_sgp_var))*_gp->f_noisy_inp(points, sx2) +
                             (sqrt_sgp_var/(sqrt_gp_var+sqrt_sgp_var))* _sgp->f_noisy_inp(points, sx2);

//    if (idx == gp_struct.num_predict_points_future-1)
//    {
//        predict_struct.var[idx] = 0.5*(gp->var_noisy_inp(points, predict_struct.mu[idx], sx2) +
//                                        _sgp->var_noisy_inp(points, predict_struct.mu[idx], sx2));
////        std::cout<<"predict_struct.var["<<idx<<"] = "<<predict_struct.var[idx]<<"\n";
//    }
//    else
//        predict_struct.var[idx] = 0.3*gp->var(points) + 0.7* _sgp->var(points);
        predict_struct.var[idx] = (sqrt_gp_var/(sqrt_gp_var+sqrt_sgp_var))*gp_var +
                                  (sqrt_sgp_var/(sqrt_gp_var+sqrt_sgp_var))*sgp_var;

    predict_struct.mu_unstandardized[idx] = predict_struct.mu[idx]*standardize_struct.std_dev_targets +
                                            standardize_struct.mean_targets;
    predict_struct.var_unstandardized[idx] = predict_struct.var[idx]*standardize_struct.std_dev_targets +
                                             standardize_struct.mean_targets;
    predict_struct.mu_p_2std_dev[idx] = predict_struct.mu_unstandardized[idx] + 2*sqrt(predict_struct.var_unstandardized[idx]);
    predict_struct.mu_m_2std_dev[idx] = predict_struct.mu_unstandardized[idx] - 2*sqrt(predict_struct.var_unstandardized[idx]);
}

void GP_DISTURB_REG::standardize_optimize(GaussianProcess*& _gp)
{
    // Standardization of the training data
    // TO BE: select the most applicable one
    _gp->standardize_sampleset(standardize_struct.mean_inputs,standardize_struct.std_dev_inputs,
                               standardize_struct.mean_targets,standardize_struct.std_dev_targets);

    if(gp_struct.hyper_optimize_method == "CG")
    {
        libgp::CG cg;
        cg.maximize(*&_gp, gp_struct.grad_updates, 0);
    }
    else if(gp_struct.hyper_optimize_method == "RProp")
    {
        libgp::RProp rprop;
        rprop.maximize(*&_gp, gp_struct.grad_updates, 0);
    }
//        gp_struct.params = _gp->covf().get_loghyper();
//        gp_struct.params(gp_struct.params.size()-1) = log(0.1);
//        _gp->covf().set_loghyper(gp_struct.params);
    std::cout<<"params_"<<gp_struct.hyper_optimize_method<<" = ";
    for (int i=0; i<gp_struct.params.size(); ++i)
        std::cout<<exp(_gp->covf().get_loghyper()(i))<<", ";
    std::cout<<"\n";
}

void GP_DISTURB_REG::publish_muVar()
{
    std_msgs::Bool predInit_msg;
    predInit_msg.data = is_prediction_init;
    predInit_pub.publish(predInit_msg);

    std_msgs::Float64MultiArray mu_msg;
    mu_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mu_msg.layout.dim[0].size = predict_struct.mu_unstandardized.size();
    mu_msg.layout.dim[0].stride = 1;
    mu_msg.layout.dim[0].label = "mu_test";
    mu_msg.data.clear();
    mu_msg.data.insert(mu_msg.data.end(), predict_struct.mu_unstandardized.begin(),
                       predict_struct.mu_unstandardized.end());
    mu_pub.publish(mu_msg);

    std_msgs::Float64MultiArray var_msg;
    var_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    var_msg.layout.dim[0].size = predict_struct.var_unstandardized.size();
    var_msg.layout.dim[0].stride = 1;
    var_msg.layout.dim[0].label = "var_test";
    var_msg.data.clear();
//    for(size_t itr = 0; itr < predict_struct.var.size(); ++itr)
//    {
//        predict_struct.var.at(itr) = sqrt(predict_struct.var.at(itr));
//    }
    var_msg.data.insert(var_msg.data.end(), predict_struct.var_unstandardized.begin(),
                        predict_struct.var_unstandardized.end());
    var_pub.publish(var_msg);

    std_msgs::Float64MultiArray mu_p_2std_dev_msg;
    mu_p_2std_dev_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mu_p_2std_dev_msg.layout.dim[0].size = predict_struct.mu_p_2std_dev.size();
    mu_p_2std_dev_msg.layout.dim[0].stride = 1;
    mu_p_2std_dev_msg.layout.dim[0].label = "mu_p_2std_dev_test";
    mu_p_2std_dev_msg.data.clear();
    mu_p_2std_dev_msg.data.insert(mu_p_2std_dev_msg.data.end(), predict_struct.mu_p_2std_dev.begin(), predict_struct.mu_p_2std_dev.end());
    mu_p_2std_dev_pub.publish(mu_p_2std_dev_msg);

    std_msgs::Float64MultiArray mu_m_2std_dev_msg;
    mu_m_2std_dev_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mu_m_2std_dev_msg.layout.dim[0].size = predict_struct.mu_m_2std_dev.size();
    mu_m_2std_dev_msg.layout.dim[0].stride = 1;
    mu_m_2std_dev_msg.layout.dim[0].label = "mu_m_2std_dev_test";
    mu_m_2std_dev_msg.data.clear();
    mu_m_2std_dev_msg.data.insert(mu_m_2std_dev_msg.data.end(), predict_struct.mu_m_2std_dev.begin(), predict_struct.mu_m_2std_dev.end());
    mu_m_2std_dev_pub.publish(mu_m_2std_dev_msg);

}
