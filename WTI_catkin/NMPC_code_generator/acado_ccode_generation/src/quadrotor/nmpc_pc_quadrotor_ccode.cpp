/**
*    \author Mohit Mehndiratta
*    \date   2017
*/

#include <acado_code_generation.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>

int main( )
{
	USING_NAMESPACE_ACADO
//    using namespace Eigen;

//    using Eigen::Matrix3d;

	// Variables:
    DifferentialState x;                // the body position w.r.t X_I
    DifferentialState y;                // the body position w.r.t Y_I
    DifferentialState z;                // the body position w.r.t Z_I
    DifferentialState u;                // the translation velocity along X_B
    DifferentialState v;                // the translation velocity along Y_B
    DifferentialState w;                // the translation velocity along Z_B

    OnlineData p_rate;                  // the roll rate
    OnlineData q_rate;                  // the pitch rate
    OnlineData r_rate;                  // the yaw rate

    Control phi;                        // the roll angle
    Control theta;                      // the pitch angle
    Control psi;                        // the yaw angle
    IntermediateState Fx;                         // the external force along X_B
    IntermediateState Fy;                         // the external force along Y_B
    Control Fz;                         // the external force along Z_B

    const double m = 3.8;             // kg
    const double g = 9.81;              // m/s^2

    Fx = 0;
    Fy = 0;

	// Model equations:
	DifferentialEquation f; 

    f << dot(x) == (cos(theta)*cos(psi)) * u + (sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi)) * v + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * w;
    f << dot(y) == (cos(theta)*sin(psi)) * u + (sin(phi)*sin(theta)*sin(psi) + cos(psi)*cos(phi)) * v + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * w;
    f << dot(z) == (        -sin(theta)) * u + (                             sin(phi)*cos(theta)) * v + (                             cos(phi)*cos(theta)) * w;

    f << dot(u) == r_rate*v - q_rate*w + g*sin(theta) + (1/m)*Fx;
    f << dot(v) == p_rate*w - r_rate*u - g*sin(phi)*cos(theta) + (1/m)*Fy;
    f << dot(w) == q_rate*u - p_rate*v - g*cos(phi)*cos(theta) + (1/m)*Fz;

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x << y << z << u << v << w << phi << theta << psi<< Fz;
    hN << x << y << z << u << v << w;

	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );

	//
	// Optimal Control Problem
	//
    double N = 30;
	double Ts = 0.01;
    OCP ocp(0.0, N*Ts, N);

	ocp.subjectTo( f );

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo( -40 *M_PI/180 <= phi <= 40 *M_PI/180);
    ocp.subjectTo( -40 *M_PI/180 <= theta <= 40 *M_PI/180);
//    ocp.subjectTo( 0.5*m*g <= Fz <= 1.8*m*g);
    ocp.subjectTo( 0.3*m*g <= Fz <= 2*m*g);
//    ocp.subjectTo( -1.0 <= dot(Fz) <= 1.0);

    // Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
	mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE, INT_RK4 );
    mpc.set( NUM_INTEGRATOR_STEPS, 2 * N );

	mpc.set( SPARSE_QP_SOLUTION, CONDENSING );
	mpc.set( QP_SOLVER, QP_QPOASES );
	
// 	mpc.set( SPARSE_QP_SOLUTION, SPARSE_SOLVER );
// 	mpc.set( QP_SOLVER, QP_QPDUNES );
	
	mpc.set( HOTSTART_QP, YES );

    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES); // Possible to Change Constraints Afterwards (only with qpOASES)

    mpc.set( GENERATE_TEST_FILE, NO);
    mpc.set( GENERATE_MAKE_FILE, NO);
	mpc.set( GENERATE_MATLAB_INTERFACE, NO );
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO );

	// Optionally set custom module name:
    mpc.set( CG_MODULE_NAME, "nmpc" );
    mpc.set( CG_MODULE_PREFIX, "NMPC" );

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/NMPC_PC_quadrotor";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mpc.exportCode(path_dir) != SUCCESSFUL_RETURN) ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mpc.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;

}
