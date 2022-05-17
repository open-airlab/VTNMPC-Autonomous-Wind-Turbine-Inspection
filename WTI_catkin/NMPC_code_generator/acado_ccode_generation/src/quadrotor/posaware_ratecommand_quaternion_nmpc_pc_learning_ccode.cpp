/**
*    \author Mohit Mehndiratta
*    \date   2021
*/

#include <acado_code_generation.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>

int main()
{
    USING_NAMESPACE_ACADO

    // Variables:
    DifferentialState x;  // the body position w.r.t X_I
    DifferentialState y;  // the body position w.r.t Y_I
    DifferentialState z;  // the body position w.r.t Z_I
    DifferentialState u;  // the translation velocity along X_B
    DifferentialState v;  // the translation velocity along Y_B
    DifferentialState w;  // the translation velocity along Z_B
    DifferentialState q_x;
    DifferentialState q_y;
    DifferentialState q_z;
    DifferentialState q_w;

    OnlineData Fx_dist;  // the external disturbance force along X_B
    OnlineData Fy_dist;  // the external disturbance force along Y_B
    OnlineData Fz_dist;  // the external disturbance force along Z_B

    OnlineData px;  // x-position of the inspection point
    OnlineData py;  // y-position of the inspection point
    OnlineData pz;  // z-position of the inspection point

    DifferentialState aux_state_px, aux_state_py, aux_state_pz;

    Control p_rate;  // the roll rate
    Control q_rate;  // the pitch rate
    Control r_rate;  // the yaw rate
    Control Fz;      // the external force along Z_B

    const double m = 3.8;   // kg
    const double g = 9.81;  // m/s^2

    // Model equations:
    DifferentialEquation f;

    //    f << dot(x) ==
    //        (1 - 2 * q_y * q_y - 2 * q_z * q_z) * u + 2 * (q_x * q_y + q_w * q_z) * v + 2 * (q_x * q_z - q_w * q_y) * w;
    //    f << dot(y) ==
    //        2 * (q_x * q_y - q_w * q_z) * u + (1 - 2 * q_x * q_x - 2 * q_z * q_z) * v + 2 * (q_y * q_z + q_w * q_x) * w;
    //    f << dot(z) ==
    //        2 * (q_x * q_z + q_w * q_y) * u + 2 * (q_y * q_z - q_w * q_x) * v + (1 - 2 * q_x * q_x - 2 * q_y * q_y) * w;

    f << dot(x) ==
        (1 - 2 * q_y * q_y - 2 * q_z * q_z) * u + 2 * (q_x * q_y - q_w * q_z) * v + 2 * (q_x * q_z + q_w * q_y) * w;
    f << dot(y) ==
        2 * (q_x * q_y + q_w * q_z) * u + (1 - 2 * q_x * q_x - 2 * q_z * q_z) * v + 2 * (q_y * q_z - q_w * q_x) * w;
    f << dot(z) ==
        2 * (q_x * q_z - q_w * q_y) * u + 2 * (q_y * q_z + q_w * q_x) * v + (1 - 2 * q_x * q_x - 2 * q_y * q_y) * w;

    f << dot(u) == r_rate * v - q_rate * w - g * 2 * (q_x * q_z - q_w * q_y) + Fx_dist;
    f << dot(v) == p_rate * w - r_rate * u - g * 2 * (q_y * q_z + q_w * q_x) + Fy_dist;
    f << dot(w) == q_rate * u - p_rate * v - g * (1 - 2 * q_x * q_x - 2 * q_y * q_y) + (1 / m) * (Fz) + Fz_dist;

    f << dot(q_x) == 0.5 * (p_rate * q_w + r_rate * q_y - q_rate * q_z);
    f << dot(q_y) == 0.5 * (q_rate * q_w - r_rate * q_x + p_rate * q_z);
    f << dot(q_z) == 0.5 * (r_rate * q_w + q_rate * q_x - p_rate * q_y);
    f << dot(q_w) == 0.5 * (-p_rate * q_x - q_rate * q_y - r_rate * q_z);

    f << dot(aux_state_px) == px;
    f << dot(aux_state_py) == py;
    f << dot(aux_state_pz) == pz;

    // equation for s
    IntermediateState n1 = (px - x);  //vector n components n=[n1;n2;n3]
    IntermediateState n2 = (py - y);
    IntermediateState n3 = (pz - z);
    IntermediateState norm_n = sqrt(n1 * n1 + n2 * n2) + 0.001;  // Constant added for numerical stability
    IntermediateState s, s_dot;                                  // relative distance to the inspection point?
                                                                 // s_dot assumes px, py, pz velocities are negligible
    //s_dot = (1 / norm_n) * (-sin(psi) * r_rate * n1 + cos(psi) * (0 - u) + cos(psi) * r_rate * n2 + sin(psi) * (0 - v));

    //quaternion objective
      s = (1 / norm_n) * ((1 - 2 * q_y * q_y - 2 * q_z * q_z) * n1 + 2 * (q_x * q_y + q_w * q_z) * n2);
   // s = (1 / norm_n) * ((1 - 2 * q_z * q_z) * n1 + (2 * q_w * q_z) * n2);
   // s_dot = (1 / norm_n) * (1.0);

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x << y << z << u << v << w << q_x << q_y << q_z << q_w << s << s_dot << p_rate << q_rate << r_rate << Fz;
    hN << x << y << z << u << v << w << q_x << q_y << q_z << q_w;

    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    //
    // Optimal Control Problem
    //
    double N = 30;
    double Ts = 0.01;
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo(-100 * M_PI / 180 <= p_rate <= 100 * M_PI / 180);
    ocp.subjectTo(-100 * M_PI / 180 <= q_rate <= 100 * M_PI / 180);
    ocp.subjectTo(-100 * M_PI / 180 <= r_rate <= 100 * M_PI / 180);
    ocp.subjectTo(0.3 * m * g <= Fz <= 2 * m * g);

    // Export the code:
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK4);
    mpc.set(NUM_INTEGRATOR_STEPS, 2 * N);

    mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(MAX_NUM_QP_ITERATIONS, 1000);

    // 	mpc.set( SPARSE_QP_SOLUTION, SPARSE_SOLVER );
    // 	mpc.set( QP_SOLVER, QP_QPDUNES );

    mpc.set(HOTSTART_QP, YES);

    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);  // Possible to Change Constraints Afterwards (only with qpOASES)

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Optionally set custom module name:
    mpc.set(CG_MODULE_NAME, "nmpc");
    mpc.set(CG_MODULE_PREFIX, "NMPC");

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/posaware_ratecommand_quaternion_NMPC_PC_learning";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mpc.exportCode(path_dir) != SUCCESSFUL_RETURN)
            ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mpc.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}
