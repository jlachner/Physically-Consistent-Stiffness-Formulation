/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS,"
without warranty of any kind, including without limitation the warranties
of merchantability, fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable
for any particular purpose. In no event shall KUKA be responsible for loss
or damages arising from the installation or use of the Software,
including but not limited to any indirect, punitive, special, incidental
or consequential damages of any character including, without limitation,
damages for loss of goodwill, work stoppage, computer failure or malfunction,
or any and all other commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by KUKA.
Should the Software prove defective, KUKA is not liable for the entire cost
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2015
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned
to KUKA Roboter GmbH immediately upon request.
This material and the information illustrated or contained herein may not be used,
reproduced, stored in a retrieval system, or transmitted in whole
or in part in any way - electronic, mechanical, photocopying, recording,
or otherwise, without the prior written consent of KUKA Roboter GmbH.



\file
\version {1.9}
*/
#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include "friLBRClient.h"
#include "exp_robots.h"
#include "exp_trajs.h"

#include <fstream>

#include "AtiForceTorqueSensor.h"

#include <boost/thread.hpp>
#include <boost/chrono.hpp>


using namespace KUKA::FRI;
/**
 * \brief Template client implementation.
 */
class MyLBRClient : public LBRClient
{

public:

    /**
    * \brief Constructor.
    */
    MyLBRClient(double freqHz, double amplitude);

    /**
    * \brief Destructor.
    */
    ~MyLBRClient();

    /**
    * \brief Callback for FRI state changes.
    *
    * @param oldState
    * @param newState
    */
    virtual void onStateChange(ESessionState oldState, ESessionState newState);

    /**
    * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void monitor();

    /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void waitForCommand();


    /**
    * \brief Callback for the FRI state 'Commanding Active'.
    *
    * If you do not want to change the default-behavior, you do not have to implement this method.
    */
    virtual void command();


private:

    // Create iiwa as child of primitive class
    iiwa14 *myLBR;

    // Double values to get measured robot values and command robot values
    double torques[7];
    double qInitial[7];
    double xInitial[7];
    double qApplied[7];
    double qCurr[7];
    double qOld[7];
    double q_arr[7];
    double dq_arr[7];
    double tauExternal[7];


    // Time parameters for control loop
    double sampleTime;
    double currentTime;
    double t_pressed;

    int N_data;
    int N_curr;
    int n_step;

    Eigen::MatrixXd q_data;
    Eigen::MatrixXd dx_data;
    Eigen::VectorXd q_init0;
    Eigen::VectorXd q_init1;

    bool is_pressed;

    // Minimum-jerk trajectory to move to that location
    MinimumJerkTrajectory *mjt_q;

    // Choose the body you want to control and the position on this body
    signed int bodyIndex;
    Eigen::Vector3d pointPosition;

    // Current position and velocity as Eigen vector
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd dq_0;
    Eigen::VectorXd tauExt;

    // Command torque vectors (with and without constraints)
    Eigen::VectorXd tau_motion;
    Eigen::VectorXd tau_previous;
    Eigen::VectorXd tau_prev_prev;
    Eigen::VectorXd tau_total;

    // DECLARE VARIABLES FOR YOUR CONTROLLER HERE!!!
    Eigen::MatrixXd M;
    Eigen::MatrixXd M_inv;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::MatrixXd J;
    
    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kr;
    Eigen::MatrixXd K;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Bp;
    Eigen::MatrixXd Br;
    Eigen::MatrixXd Kq;
    Eigen::MatrixXd Kq_last;
    Eigen::MatrixXd Bq_last;
    Eigen::MatrixXd K_kin;
    Eigen::MatrixXd Bq;
    Eigen::MatrixXd Kq_thread;
    Eigen::MatrixXd Bq_thread;
    Eigen::MatrixXd K_fin_thread;
    Eigen::MatrixXd K_kin_thread;
    Eigen::VectorXd q_0;

    Eigen::MatrixXd Kq_test;
    Eigen::MatrixXd Bq_test;
    
    Eigen::MatrixXd dJH1_thread;
    Eigen::MatrixXd dJH2_thread;
    Eigen::MatrixXd dJH3_thread;
    Eigen::MatrixXd dJH4_thread;
    Eigen::MatrixXd dJH5_thread;
    Eigen::MatrixXd dJH6_thread;
    Eigen::MatrixXd dJH7_thread;

    Eigen::MatrixXd dJH1;
    Eigen::MatrixXd dJH2;
    Eigen::MatrixXd dJH3;
    Eigen::MatrixXd dJH4;
    Eigen::MatrixXd dJH5;
    Eigen::MatrixXd dJH6;
    Eigen::MatrixXd dJH7;

    void impedanceOptimizer();
    boost::thread impedanceOptimizerThread;
    boost::mutex mutex;
    
    // Force-Torque Sensor
    AtiForceTorqueSensor *ftSensor;
    double* f_sens_ee;
    Eigen::VectorXd f_ext_ee;
    Eigen::VectorXd m_ext_ee;
    Eigen::VectorXd f_ext_0;
    Eigen::VectorXd m_ext_0;
    Eigen::VectorXd F_ext_0;

};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
