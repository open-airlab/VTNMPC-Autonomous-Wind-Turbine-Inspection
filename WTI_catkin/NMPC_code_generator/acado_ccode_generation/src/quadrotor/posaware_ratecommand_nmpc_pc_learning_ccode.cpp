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
    DifferentialState x;    // the body position w.r.t X_I
    DifferentialState y;    // the body position w.r.t Y_I
    DifferentialState z;    // the body position w.r.t Z_I
    DifferentialState u;    // the translation velocity along X_B
    DifferentialState v;    // the translation velocity along Y_B
    DifferentialState w;    // the translation velocity along Z_B
    DifferentialState psi;  // the yaw angle

    OnlineData p_rate;  // the roll rate
    OnlineData q_rate;  // the pitch rate

    OnlineData Fx_dist;  // the external disturbance force along X_B
    OnlineData Fy_dist;  // the external disturbance force along Y_B
    OnlineData Fz_dist;  // the external disturbance force along Z_B
                         //    OnlineData m;                       // the mass of the UAV

    OnlineData px;  // x-position of the inspection point
    OnlineData py;  // y-position of the inspection point
    OnlineData pz;  // z-position of the inspection point
    DifferentialState aux_state_px, aux_state_py, aux_state_pz;

    Control phi;     // the roll angle
    Control theta;   // the pitch angle
    Control r_rate;  // the yaw rate
    Control Fz;      // the external force along Z_B

    const double m = 3.8;   // kg
    const double g = 9.81;  // m/s^2

    // Model equations:
    DifferentialEquation f;

    f << dot(x) == (cos(theta) * cos(psi)) * u + (sin(phi) * sin(theta) * cos(psi) - sin(psi) * cos(phi)) * v +
                       (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;
    f << dot(y) == (cos(theta) * sin(psi)) * u + (sin(phi) * sin(theta) * sin(psi) + cos(psi) * cos(phi)) * v +
                       (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;
    f << dot(z) == (-sin(theta)) * u + (sin(phi) * cos(theta)) * v + (cos(phi) * cos(theta)) * w;

    f << dot(u) == r_rate * v - q_rate * w + g * sin(theta) + Fx_dist;
    f << dot(v) == p_rate * w - r_rate * u - g * sin(phi) * cos(theta) + Fy_dist;
    f << dot(w) == q_rate * u - p_rate * v - g * cos(phi) * cos(theta) + (1 / m) * (Fz) + Fz_dist;

    //    f << dot(phi) == p_rate + sin(phi) * tan(theta) * q_rate + cos(phi) * tan(theta) * r_rate;
    //    f << dot(theta) == cos(phi) * q_rate - sin(phi) * r_rate;
    f << dot(psi) == sin(phi) / cos(theta) * q_rate + cos(phi) / cos(theta) * r_rate;

    f << dot(aux_state_px) == px;
    f << dot(aux_state_py) == py;
    f << dot(aux_state_pz) == pz;

    // equation for s
    IntermediateState n1 = (px - x);  //vector n components n=[n1;n2;n3]
    IntermediateState n2 = (py - y);
    IntermediateState n3 = (pz - z);
    IntermediateState norm_n = sqrt(n1 * n1 + n2 * n2) + 0.001;  // Constant added for numerical stability
    IntermediateState s, s_dot;                                  // relative distance to the inspection point?
    //    s = (1 / norm_n) * (cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta) * n1 - cos(theta) * sin(psi) +
    //                        cos(psi) * sin(phi) * sin(theta) * n2 - cos(phi) * sin(theta) * n3);
    //simple s and s_dot
    s = (1 / norm_n) * (cos(psi) * n1 + sin(psi) * n2);
    // s_dot assumes px, py, pz velocities are negligible
    s_dot = (1 / norm_n) * (-sin(psi) * r_rate * n1 + cos(psi) * (0 - u) + cos(psi) * r_rate * n2 + sin(psi) * (0 - v));

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x << y << z << u << v << w << psi << s - 1 << s_dot << phi << theta << r_rate << Fz;
    hN << x << y << z << u << v << w << psi;

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

    ocp.subjectTo(-40 * M_PI / 180 <= phi <= 40 * M_PI / 180);
    ocp.subjectTo(-40 * M_PI / 180 <= theta <= 40 * M_PI / 180);
    //    ocp.subjectTo(-60 * M_PI / 180 <= r_rate <= 60 * M_PI / 180);
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
    std::string path_dir = path + "/solver/posaware_ratecommand_NMPC_PC_learning";
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
