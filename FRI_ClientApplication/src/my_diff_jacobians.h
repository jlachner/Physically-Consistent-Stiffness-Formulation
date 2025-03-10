#include <Eigen/Dense>
#include <cmath>


Eigen::MatrixXd dJH_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7);
Eigen::MatrixXd dJH_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7);

Eigen::MatrixXd dJS_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJS_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 

Eigen::MatrixXd dJB_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 
Eigen::MatrixXd dJB_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7); 