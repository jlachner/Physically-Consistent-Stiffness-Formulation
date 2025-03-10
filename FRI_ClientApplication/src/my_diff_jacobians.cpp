#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

Eigen::MatrixXd dJS_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t25 = t2 * t5 * t9;
    double t26 = t4 * t5 * t9;
    double t28 = t2 * t9 * t11;
    double t29 = t5 * t8 * t9;
    double t30 = t4 * t9 * t11;
    double t31 = t6 * t9 * t10;
    double t32 = t3 * (9.0 / 25.0);
    double t33 = t5 * (39.0 / 50.0);
    double t34 = t7 * (59.0 / 50.0);
    double t41 = t2 * t9 * (9.0 / 25.0);
    double t42 = t8 * t9 * (9.0 / 25.0);
    double t22 = t3 * t14;
    double t23 = t3 * t16;
    double t24 = t3 * t17;
    double t27 = t3 * t19;
    double t36 = -t26;
    double t39 = -t29;
    double t43 = t30 * (39.0 / 50.0);
    double t46 = t15 + t30;
    double t50 = t33 - 39.0 / 50.0;
    double t51 = t34 - 59.0 / 50.0;
    double t35 = -t22;
    double t37 = -t27;
    double t44 = t16 + t24;
    double t45 = t17 + t23;
    double t49 = t18 + t36;
    double t54 = t7 * t46;
    double t64 = t3 * t50;
    double t66 = t2 * t9 * t50;
    double t67 = t8 * t9 * t50;
    double t82 = t46 * t51;
    double t47 = t14 + t37;
    double t48 = t19 + t35;
    double t52 = t5 * t44;
    double t53 = t6 * t45;
    double t55 = t11 * t44;
    double t56 = t12 * t45;
    double t60 = t6 * t49;
    double t63 = t12 * t49;
    double t86 = t32 + t43 + t64 - 9.0 / 25.0;
    double t57 = t5 * t48;
    double t58 = t6 * t47;
    double t59 = -t53;
    double t61 = t11 * t48;
    double t62 = t12 * t47;
    double t68 = t55 * (39.0 / 50.0);
    double t71 = t20 + t52;
    double t72 = t39 + t55;
    double t73 = t21 + t60;
    double t80 = -t7 * (t29 - t55);
    double t65 = -t58;
    double t69 = -t68;
    double t70 = t61 * (39.0 / 50.0);
    double t74 = t25 + t61;
    double t75 = t6 * t71;
    double t76 = t12 * t71;
    double t81 = t13 * t73;
    double t83 = -t6 * (t28 - t57);
    double t84 = -t12 * (t28 - t57);
    double t79 = t7 * t74;
    double t85 = t81 * (59.0 / 50.0);
    double t87 = t62 + t75;
    double t88 = t54 + t81;
    double t89 = t56 + t83;
    double t90 = t65 + t76;
    double t92 = t59 + t84;
    double t91 = t13 * t87;
    double t93 = t13 * t89;
    double t98 = t82 + t85 + t86;
    double t94 = -t91;
    double t95 = -t93;
    double t96 = t80 + t94;
    double t97 = t79 + t95;

    MatrixXd dJ_T_dq1_mat( 6, 7 );
    dJ_T_dq1_mat << 0.0, t8 * (9.0 / 25.0), -t41, t16 * (-21.0 / 50.0) - t17 * (9.0 / 25.0) - t23 * (9.0 / 25.0) - t24 * (21.0 / 50.0), t25 * (-9.0 / 25.0) + t11 * t14 * (21.0 / 50.0) - t11 * t19 * (9.0 / 25.0) + t14 * t18 * (9.0 / 25.0) - t18 * t19 * (21.0 / 50.0), t62 * (59.0 / 50.0) + t75 * (59.0 / 50.0) - t86 * (t53 + t12 * (t28 - t57)) - (t31 - t63) * (t41 + t66 + t70),
                  t97 * t98 - t88 * (t41 + t66 + t70 - t93 * (59.0 / 50.0) + t51 * t74), 0.0, t2 * (-9.0 / 25.0), -t42, t14 * (9.0 / 25.0) - t19 * (21.0 / 50.0) + t22 * (21.0 / 50.0) - t27 * (9.0 / 25.0), t29 * (-9.0 / 25.0) + t11 * t16 * (9.0 / 25.0) + t11 * t17 * (21.0 / 50.0) + t16 * t18 * (21.0 / 50.0) + t17 * t18 * (9.0 / 25.0),
                  t56 * (59.0 / 50.0) - t6 * (t28 - t57) * (59.0 / 50.0) + t86 * (t58 - t76) - (t31 - t63) * (t42 + t67 + t69), t98 * (t91 + t7 * (t29 - t55)) - t88 * (t42 + t67 + t69 + t91 * (59.0 / 50.0) + t51 * (t29 - t55)), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -t2, -t8 * t9, t47, t72, t90, t96, 0.0,
                  -t8, t2 * t9, t45, t74, t92, t97, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return dJ_T_dq1_mat.transpose( );
}


Eigen::MatrixXd dJS_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = q1 + q3;
    double t15 = -q3;
    double t16 = t3 * t5;
    double t17 = t5 * t9;
    double t18 = t9 * t11;
    double t19 = q1 + t15;
    double t20 = t9 * t10 * t12;
    double t22 = t3 * t4 * t11;
    double t23 = t3 * t6 * t11;
    double t21 = t4 * t16;
    double t24 = t4 * t18;
    double t25 = t4 * t6 * t17;
    double t26 = -t25;
    double t27 = t16 + t24;
    double t28 = t18 + t21;
    double t29 = t20 + t23 + t26;

    MatrixXd dJ_T_dq2_mat( 6, 7 );
    dJ_T_dq2_mat << 0.0, 0.0, t3 * t8 * (-9.0 / 25.0), t9 * (cos(t14) * (13.0 / 2.0) + cos(t19) / 2.0) * (-3.0 / 50.0), t8 * t16 * (-9.0 / 25.0) - t8 * t24 * (9.0 / 25.0) - t2 * t10 * t18 * (21.0 / 50.0), 
                    t2 * t20 * (-2.0 / 5.0) - t2 * t23 * (2.0 / 5.0) + t2 * t25 * (2.0 / 5.0) + t2 * t4 * t6 * t9 * (21.0 / 50.0) - t6 * t8 * t9 * t10 * (9.0 / 25.0) + t3 * t8 * t11 * t12 * (9.0 / 25.0) - t2 * t10 * t12 * t17 * (21.0 / 50.0) - t4 * t8 * t12 * t17 * (9.0 / 25.0),
                    t7 * t8 * t16 * (-9.0 / 25.0) - t7 * t8 * t24 * (9.0 / 25.0) - t8 * t13 * t20 * (9.0 / 25.0) - t8 * t13 * t23 * (9.0 / 25.0) + t8 * t13 * t25 * (9.0 / 25.0) - t2 * t7 * t10 * t18 * (21.0 / 50.0) + t2 * t4 * t9 * t12 * t13 * (21.0 / 50.0) + t2 * t6 * t9 * t10 * t13 * (2.0 / 5.0) - t2 * t3 * t11 * t12 * t13 * (2.0 / 5.0) + t2 * t4 * t12 * t13 * t17 * (2.0 / 5.0) + t2 * t6 * t10 * t13 * t17 * (21.0 / 50.0),
                    0.0, 0.0, t2 * t3 * (9.0 / 25.0), t9 * (sin(t14) * (13.0 / 2.0) + sin(t19) / 2.0) * (-3.0 / 50.0), t2 * t16 * (9.0 / 25.0) + t2 * t24 * (9.0 / 25.0) - t8 * t10 * t18 * (21.0 / 50.0),
                    t8 * t20 * (-2.0 / 5.0) - t8 * t23 * (2.0 / 5.0) + t8 * t25 * (2.0 / 5.0) + t2 * t6 * t9 * t10 * (9.0 / 25.0) + t4 * t6 * t8 * t9 * (21.0 / 50.0) - t2 * t3 * t11 * t12 * (9.0 / 25.0) + t2 * t4 * t12 * t17 * (9.0 / 25.0) - t8 * t10 * t12 * t17 * (21.0 / 50.0),
                    t2 * t7 * t16 * (9.0 / 25.0) + t2 * t7 * t24 * (9.0 / 25.0) + t2 * t13 * t20 * (9.0 / 25.0) + t2 * t13 * t23 * (9.0 / 25.0) - t2 * t13 * t25 * (9.0 / 25.0) - t7 * t8 * t10 * t18 * (21.0 / 50.0) + t4 * t8 * t9 * t12 * t13 * (21.0 / 50.0) + t6 * t8 * t9 * t10 * t13 * (2.0 / 5.0) - t3 * t8 * t11 * t12 * t13 * (2.0 / 5.0) + t4 * t8 * t12 * t13 * t17 * (2.0 / 5.0) + t6 * t8 * t10 * t13 * t17 * (21.0 / 50.0),
                    0.0, 0.0, 0.0, t3 * t4 * (-21.0 / 50.0), t3 * t10 * t11 * (-21.0 / 50.0), t6 * t18 * (2.0 / 5.0) + t6 * t21 * (2.0 / 5.0) + t3 * t4 * t6 * (21.0 / 50.0) - t3 * t10 * t12 * (2.0 / 5.0) - t10 * t12 * t16 * (21.0 / 50.0),
                    t12 * t13 * t18 * (2.0 / 5.0) + t12 * t13 * t21 * (2.0 / 5.0) - t3 * t7 * t10 * t11 * (21.0 / 50.0) + t3 * t4 * t12 * t13 * (21.0 / 50.0) + t3 * t6 * t10 * t13 * (2.0 / 5.0) + t6 * t10 * t13 * t16 * (21.0 / 50.0), 0.0, 0.0, t2 * t3, -t2 * t9 * t10,
                    t2 * t27, -t12 * (t2 * t3 * t11 - t2 * t4 * t17) + t2 * t6 * t9 * t10, t2 * t7 * t27 + t2 * t13 * t29, 0.0, 0.0, t3 * t8, -t8 * t9 * t10, t8 * t27, -t12 * (t3 * t8 * t11 - t4 * t8 * t17) + t6 * t8 * t9 * t10,
                    t7 * t8 * t27 + t8 * t13 * t29, 0.0, 0.0, -t9, -t3 * t10, -t17 + t22, t12 * t28 + t3 * t6 * t10, -t13 * (t6 * t28 - t3 * t10 * t12) - t7 * (t17 - t22);

    return dJ_T_dq2_mat.transpose( );
}

MatrixXd dJS_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t2 * t10;
    double t16 = t4 * t8;
    double t17 = t3 * t11;
    double t18 = t8 * t10;
    double t22 = t4 * t5 * t9;
    double t23 = t4 * t6 * t9;
    double t25 = t6 * t9 * t10;
    double t26 = t3 * (9.0 / 25.0);
    double t27 = t5 * (39.0 / 50.0);
    double t29 = t5 * t9 * t10 * t12;
    double t34 = t4 * t9 * t11 * (39.0 / 50.0);
    double t19 = t3 * t14;
    double t20 = t3 * t15;
    double t21 = t3 * t16;
    double t24 = t3 * t18;
    double t30 = -t22;
    double t33 = -t29;
    double t40 = t27 - 39.0 / 50.0;
    double t28 = -t19;
    double t31 = -t24;
    double t35 = t15 + t21;
    double t36 = t16 + t20;
    double t39 = t17 + t30;
    double t45 = t3 * t40;
    double t46 = t23 + t33;
    double t37 = t14 + t31;
    double t38 = t18 + t28;
    double t41 = t6 * t35;
    double t42 = t5 * t12 * t36;
    double t44 = t12 * t39;
    double t50 = t26 + t34 + t45 - 9.0 / 25.0;
    double t43 = t6 * t38;
    double t47 = t5 * t12 * t37;
    double t49 = t42 + t43;

    MatrixXd dJ_T_dq3_mat( 6, 7 );
    dJ_T_dq3_mat << 0.0, 0.0, 0.0, t15 * (-9.0 / 25.0) - t16 * (21.0 / 50.0) - t20 * (21.0 / 50.0) - t21 * (9.0 / 25.0), t11 * (t14 * 6.0 - t18 * 7.0 + t19 * 7.0 - t24 * 6.0) * (3.0 / 50.0),
                  -t50 * (t41 + t47) - t12 * t38 * (59.0 / 50.0) - t46 * (t8 * t9 * (9.0 / 25.0) - t11 * t35 * (39.0 / 50.0) + t8 * t9 * t40) + t5 * t6 * t36 * (59.0 / 50.0) + t11 * t37 * (t25 - t44) * (39.0 / 50.0) + t9 * t10 * t11 * (t12 * (t5 * t35 + t8 * t9 * t11) - t6 * t37) * (39.0 / 50.0),
                  t7 * t11 * t14 * (9.0 / 25.0) - t7 * t11 * t18 * (21.0 / 50.0) + t6 * t13 * t18 * (2.0 / 5.0) - t6 * t13 * t19 * (2.0 / 5.0) + t7 * t14 * t17 * (21.0 / 50.0) + t12 * t13 * t15 * (9.0 / 25.0) + t12 * t13 * t16 * (21.0 / 50.0) - t7 * t17 * t18 * (9.0 / 25.0) + t12 * t13 * t20 * (21.0 / 50.0) + t12 * t13 * t21 * (9.0 / 25.0) - t5 * t6 * t13 * t14 * (9.0 / 25.0) + t5 * t6 * t13 * t18 * (21.0 / 50.0) - t5 * t6 * t13 * t19 * (21.0 / 50.0) + t5 * t12 * t13 * t16 * (2.0 / 5.0) + t5 * t6 * t13 * t24 * (9.0 / 25.0) + t5 * t12 * t13 * t20 * (2.0 / 5.0),
                  0.0, 0.0, 0.0, t14 * (21.0 / 50.0) - t18 * (9.0 / 25.0) + t19 * (9.0 / 25.0) - t24 * (21.0 / 50.0), t11 * (t35 * 7.0 + t36 * 6.0) * (3.0 / 50.0),
                  t12 * t35 * (59.0 / 50.0) - t49 * t50 + t46 * (t2 * t9 * (9.0 / 25.0) + t11 * t38 * (39.0 / 50.0) + t2 * t9 * t40) - t5 * t6 * t37 * (59.0 / 50.0) + t11 * t36 * (t25 - t44) * (39.0 / 50.0) + t9 * t10 * t11 * (t12 * (t5 * t38 - t2 * t9 * t11) - t6 * t36) * (39.0 / 50.0),
                  t7 * t11 * t15 * (21.0 / 50.0) - t6 * t13 * t15 * (2.0 / 5.0) + t7 * t11 * t16 * (9.0 / 25.0) + t7 * t15 * t17 * (9.0 / 25.0) - t12 * t13 * t14 * (21.0 / 50.0) - t6 * t13 * t21 * (2.0 / 5.0) + t7 * t16 * t17 * (21.0 / 50.0) + t12 * t13 * t18 * (9.0 / 25.0) - t12 * t13 * t19 * (9.0 / 25.0) + t12 * t13 * t24 * (21.0 / 50.0) - t5 * t6 * t13 * t15 * (21.0 / 50.0) - t5 * t6 * t13 * t16 * (9.0 / 25.0) - t5 * t6 * t13 * t20 * (9.0 / 25.0) - t5 * t12 * t13 * t14 * (2.0 / 5.0) - t5 * t6 * t13 * t21 * (21.0 / 50.0) + t5 * t12 * t13 * t24 * (2.0 / 5.0),
                  0.0, 0.0, 0.0, t9 * t10 * (21.0 / 50.0), t4 * t9 * t11 * (-21.0 / 50.0),
                  t9 * (t4 * t12 * 20.0 + t6 * t10 * 21.0 + t4 * t5 * t12 * 21.0 + t5 * t6 * t10 * 20.0) * (-1.0 / 50.0), t9 * (t4 * t7 * t11 * 21.0 - t4 * t6 * t13 * 20.0 + t10 * t12 * t13 * 21.0 - t4 * t5 * t6 * t13 * 21.0 + t5 * t10 * t12 * t13 * 20.0) * (-1.0 / 50.0), 0.0, 0.0, 0.0, -t18 + t19,
                  t11 * t36, t49, t13 * (t12 * t38 - t5 * t6 * t36) + t7 * t11 * t36, 0.0, 0.0, 0.0, t35, -t11 * t37, -t41 - t47, -t13 * (t12 * t35 - t5 * t6 * t37) - t7 * t11 * t37, 0.0, 0.0, 0.0, -t4 * t9, -t9 * t10 * t11,
                  t9 * (t4 * t6 - t5 * t10 * t12), t13 * (t5 * t25 + t4 * t9 * t12) - t7 * t9 * t10 * t11;

    return dJ_T_dq3_mat.transpose( );
}

Eigen::MatrixXd dJS_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t3 * t5;
    double t15 = t2 * t10;
    double t16 = t3 * t11;
    double t17 = t8 * t10;
    double t18 = t8 * t9 * t11;
    double t19 = t2 * t3 * t4;
    double t20 = t3 * t4 * t8;
    double t21 = t2 * t5 * t9;
    double t22 = t4 * t5 * t9;
    double t23 = t2 * t9 * t11;
    double t24 = t5 * t8 * t9;
    double t25 = t4 * t9 * t11;
    double t26 = -t19;
    double t27 = -t23;
    double t29 = t15 + t20;
    double t30 = t14 + t25;
    double t31 = t17 + t26;
    double t32 = t5 * t29;
    double t33 = t11 * t29;
    double t34 = t5 * t31;
    double t35 = t11 * t31;
    double t37 = t21 + t35;
    double t38 = t27 + t34;

    MatrixXd dJ_T_dq4_mat( 6, 7 );
    dJ_T_dq4_mat << 0.0, 0.0, 0.0, 0.0, t18 * (9.0 / 25.0) + t5 * t15 * (9.0 / 25.0) + t14 * t15 * (21.0 / 50.0) + t4 * t5 * t8 * (21.0 / 50.0) + t4 * t8 * t14 * (9.0 / 25.0),
                    t6 * t21 * (-2.0 / 5.0) + t12 * t24 * (9.0 / 25.0) - t6 * t11 * t17 * (2.0 / 5.0) - t11 * t12 * t15 * (9.0 / 25.0) - t12 * t15 * t16 * (21.0 / 50.0) + t2 * t4 * t6 * t16 * (2.0 / 5.0) - t4 * t8 * t11 * t12 * (21.0 / 50.0) - t4 * t8 * t12 * t16 * (9.0 / 25.0),
                    t7 * t18 * (9.0 / 25.0) + t5 * t7 * t15 * (9.0 / 25.0) + t7 * t14 * t15 * (21.0 / 50.0) - t6 * t13 * t24 * (9.0 / 25.0) - t12 * t13 * t21 * (2.0 / 5.0) + t4 * t5 * t7 * t8 * (21.0 / 50.0) + t4 * t7 * t8 * t14 * (9.0 / 25.0) + t6 * t11 * t13 * t15 * (9.0 / 25.0) + t6 * t13 * t15 * t16 * (21.0 / 50.0) - t11 * t12 * t13 * t17 * (2.0 / 5.0) + t4 * t6 * t8 * t11 * t13 * (21.0 / 50.0) + t2 * t4 * t12 * t13 * t16 * (2.0 / 5.0) + t4 * t6 * t8 * t13 * t16 * (9.0 / 25.0),
                    0.0, 0.0, 0.0, 0.0, t23 * (-9.0 / 25.0) + t5 * t17 * (9.0 / 25.0) + t14 * t17 * (21.0 / 50.0) - t2 * t4 * t5 * (21.0 / 50.0) - t2 * t4 * t14 * (9.0 / 25.0),
                    t6 * t24 * (-2.0 / 5.0) - t12 * t21 * (9.0 / 25.0) + t6 * t11 * t15 * (2.0 / 5.0) - t11 * t12 * t17 * (9.0 / 25.0) - t12 * t16 * t17 * (21.0 / 50.0) + t2 * t4 * t11 * t12 * (21.0 / 50.0) + t2 * t4 * t12 * t16 * (9.0 / 25.0) + t4 * t6 * t8 * t16 * (2.0 / 5.0),
                    t7 * t23 * (-9.0 / 25.0) + t5 * t7 * t17 * (9.0 / 25.0) + t7 * t14 * t17 * (21.0 / 50.0) + t6 * t13 * t21 * (9.0 / 25.0) - t12 * t13 * t24 * (2.0 / 5.0) - t2 * t4 * t5 * t7 * (21.0 / 50.0) - t2 * t4 * t7 * t14 * (9.0 / 25.0) + t6 * t11 * t13 * t17 * (9.0 / 25.0) + t11 * t12 * t13 * t15 * (2.0 / 5.0) + t6 * t13 * t16 * t17 * (21.0 / 50.0) - t2 * t4 * t6 * t11 * t13 * (21.0 / 50.0) - t2 * t4 * t6 * t13 * t16 * (9.0 / 25.0) + t4 * t8 * t12 * t13 * t16 * (2.0 / 5.0),
                    0.0, 0.0, 0.0, 0.0, t5 * t9 * t10 * (-21.0 / 50.0), t6 * t14 * (-2.0 / 5.0) - t6 * t25 * (2.0 / 5.0) + t9 * t10 * t11 * t12 * (21.0 / 50.0), t12 * t13 * t14 * (-2.0 / 5.0) - t12 * t13 * t25 * (2.0 / 5.0) - t5 * t7 * t9 * t10 * (21.0 / 50.0) - t6 * t9 * t10 * t11 * t13 * (21.0 / 50.0),
                    0.0, 0.0, 0.0, 0.0, t38, -t12 * t37, -t7 * (t23 - t34) + t6 * t13 * t37, 0.0, 0.0, 0.0, 0.0, -t18 - t32, -t12 * (t24 - t33), -t7 * (t18 + t32) + t6 * t13 * (t24 - t33), 0.0, 0.0, 0.0, 0.0, -t16 + t22, -t12 * t30, -t7 * (t16 - t22) + t6 * t13 * t30;

    return dJ_T_dq4_mat.transpose( );
}

Eigen::MatrixXd dJS_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = sin(q1);
    double t8 = sin(q2);
    double t9 = sin(q3);
    double t10 = sin(q4);
    double t11 = sin(q5);
    double t12 = sin(q6);
    double t13 = t2 * t4;
    double t14 = t2 * t9;
    double t15 = t4 * t7;
    double t16 = t3 * t10;
    double t17 = t7 * t9;
    double t18 = t7 * t8 * t10;
    double t19 = t8 * t9 * t11;
    double t23 = t4 * t5 * t8;
    double t25 = t2 * t8 * t10;
    double t26 = t3 * (9.0 / 25.0);
    double t27 = t5 * (39.0 / 50.0);
    double t32 = t2 * t8 * (9.0 / 25.0);
    double t33 = t7 * t8 * (9.0 / 25.0);
    double t34 = t4 * t8 * t10 * (39.0 / 50.0);
    double t20 = t3 * t13;
    double t21 = t3 * t14;
    double t22 = t3 * t15;
    double t24 = t3 * t17;
    double t29 = -t23;
    double t40 = t27 - 39.0 / 50.0;
    double t28 = -t20;
    double t30 = -t24;
    double t35 = t14 + t22;
    double t36 = t15 + t21;
    double t39 = t16 + t29;
    double t46 = t3 * t40;
    double t47 = t2 * t8 * t40;
    double t48 = t7 * t8 * t40;
    double t37 = t13 + t30;
    double t38 = t17 + t28;
    double t41 = t5 * t35;
    double t42 = t11 * t36;
    double t44 = t6 * t39;
    double t49 = t10 * t35 * (39.0 / 50.0);
    double t57 = t26 + t34 + t46 - 9.0 / 25.0;
    double t43 = t5 * t38;
    double t45 = t11 * t37;
    double t50 = -t49;
    double t51 = t10 * t38 * (39.0 / 50.0);
    double t52 = t18 + t41;
    double t53 = t19 + t44;
    double t54 = t6 * t52;
    double t56 = -t6 * (t25 - t43);
    double t60 = t33 + t48 + t50;
    double t61 = t32 + t47 + t51;
    double t58 = t45 + t54;
    double t59 = t42 + t56;

    MatrixXd dJ_T_dq5_mat( 6, 7 );
    dJ_T_dq5_mat << 0.0, 0.0, 0.0, 0.0, 0.0, t6 * t36 * (59.0 / 50.0) + t53 * t60 - t57 * t58 + t11 * (t25 - t43) * (59.0 / 50.0),
                  (t12 * (t6 * t13 * -18.0 + t6 * t17 * 21.0 - t6 * t20 * 21.0 + t11 * t15 * 20.0 + t11 * t18 * 18.0 + t6 * t24 * 18.0 - t6 * t25 * 20.0 + t11 * t21 * 20.0 + t5 * t6 * t17 * 20.0 + t5 * t11 * t14 * 18.0 - t5 * t6 * t20 * 20.0 + t5 * t11 * t15 * 21.0 + t5 * t11 * t21 * 21.0 + t5 * t11 * t22 * 18.0)) / 50.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t6 * t37 * (-59.0 / 50.0) + t11 * t52 * (59.0 / 50.0) - t53 * t61 - t57 * t59, t12 * (t6 * t14 * 21.0 + t6 * t15 * 18.0 + t6 * t18 * 20.0 + t11 * t13 * 20.0 + t6 * t21 * 18.0 + t6 * t22 * 21.0 - t11 * t24 * 20.0 + t11 * t25 * 18.0 + t5 * t6 * t14 * 20.0 + t5 * t11 * t13 * 21.0 + t5 * t6 * t22 * 20.0 - t5 * t11 * t17 * 18.0 + t5 * t11 * t20 * 18.0 - t5 * t11 * t24 * 21.0) * (-1.0 / 50.0), 0.0, 0.0, 0.0, 0.0, 0.0,
                  t11 * t39 * (59.0 / 50.0) + t58 * t61 + t59 * t60 - t6 * t8 * t9 * (59.0 / 50.0), t12 * (t19 * 20.0 + t6 * t16 * 20.0 + t5 * t19 * 21.0 - t6 * t23 * 20.0 - t4 * t6 * t8 * 21.0) * (-1.0 / 50.0), 0.0, 0.0, 0.0, 0.0, 0.0, t59,
                  -t12 * (t6 * t36 + t11 * (t25 - t43)), 0.0, 0.0, 0.0, 0.0, 0.0, -t58, t12 * (t6 * t37 - t11 * t52), 0.0, 0.0, 0.0, 0.0, 0.0, -t53, -t12 * (t11 * t39 - t6 * t8 * t9);

    return dJ_T_dq5_mat.transpose( );
}

Eigen::MatrixXd dJS_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t10;
    double t15 = t8 * t10;
    double t16 = t2 * t3 * t4;
    double t17 = t3 * t4 * t8;
    double t18 = -t16;
    double t19 = t14 + t17;
    double t20 = t15 + t18;

    MatrixXd dJ_T_dq6_mat( 6, 7 );
    dJ_T_dq6_mat << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t7 * t12 * t15 * (21.0 / 50.0) - t7 * t12 * t16 * (21.0 / 50.0) - t11 * t13 * t14 * (9.0 / 25.0) - t11 * t13 * t17 * (9.0 / 25.0) - t2 * t4 * t7 * t12 * (9.0 / 25.0) - t4 * t6 * t7 * t8 * (2.0 / 5.0) - t3 * t6 * t7 * t14 * (2.0 / 5.0) - t5 * t6 * t7 * t14 * (9.0 / 25.0) - t5 * t6 * t7 * t17 * (9.0 / 25.0) + t5 * t8 * t9 * t13 * (9.0 / 25.0) - t4 * t8 * t11 * t13 * (21.0 / 50.0) + t3 * t7 * t12 * t15 * (9.0 / 25.0) + t5 * t7 * t12 * t15 * (2.0 / 5.0) - t5 * t7 * t12 * t16 * (2.0 / 5.0) - t3 * t11 * t13 * t14 * (21.0 / 50.0) - t4 * t5 * t6 * t7 * t8 * (21.0 / 50.0) - t3 * t5 * t6 * t7 * t14 * (21.0 / 50.0) - t2 * t7 * t9 * t11 * t12 * (2.0 / 5.0) - t6 * t7 * t8 * t9 * t11 * (9.0 / 25.0),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t7 * t12 * t14 * (-21.0 / 50.0) - t7 * t12 * t17 * (21.0 / 50.0) - t11 * t13 * t15 * (9.0 / 25.0) + t11 * t13 * t16 * (9.0 / 25.0) + t2 * t4 * t6 * t7 * (2.0 / 5.0) - t2 * t5 * t9 * t13 * (9.0 / 25.0) + t2 * t4 * t11 * t13 * (21.0 / 50.0) - t3 * t6 * t7 * t15 * (2.0 / 5.0) - t4 * t7 * t8 * t12 * (9.0 / 25.0) - t5 * t6 * t7 * t15 * (9.0 / 25.0) + t5 * t6 * t7 * t16 * (9.0 / 25.0) - t3 * t7 * t12 * t14 * (9.0 / 25.0) - t5 * t7 * t12 * t14 * (2.0 / 5.0) - t5 * t7 * t12 * t17 * (2.0 / 5.0) - t3 * t11 * t13 * t15 * (21.0 / 50.0) + t2 * t4 * t5 * t6 * t7 * (21.0 / 50.0) + t2 * t6 * t7 * t9 * t11 * (9.0 / 25.0) - t3 * t5 * t6 * t7 * t15 * (21.0 / 50.0) - t7 * t8 * t9 * t11 * t12 * (2.0 / 5.0),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t4 * t7 * t9 * t12 * (21.0 / 50.0) + t6 * t7 * t9 * t10 * (2.0 / 5.0) - t3 * t7 * t11 * t12 * (2.0 / 5.0) + t9 * t10 * t11 * t13 * (21.0 / 50.0) + t4 * t5 * t7 * t9 * t12 * (2.0 / 5.0) + t5 * t6 * t7 * t9 * t10 * (21.0 / 50.0),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  -t7 * (t6 * (t5 * t20 - t2 * t9 * t11) + t12 * (t4 * t8 + t3 * t14)) - t13 * (t11 * t20 + t2 * t5 * t9),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t7 * (t6 * (t5 * t19 + t8 * t9 * t11) + t12 * (t2 * t4 - t3 * t15)) + t13 * (t11 * t19 - t5 * t8 * t9),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  t7 * (t6 * (t3 * t11 - t4 * t5 * t9) + t9 * t10 * t12) - t13 * (t3 * t5 + t4 * t9 * t11);

    return dJ_T_dq6_mat.transpose( );
}

Eigen::MatrixXd dJS_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Initialize a 7x6 matrix with zeros
    Eigen::MatrixXd dJ_T_dq7_mat = Eigen::MatrixXd::Zero(7, 6);

    return dJ_T_dq7_mat;
}


Eigen::MatrixXd dJH_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t22 = t3 * 1050.0;
    double t26 = t2 * t5 * t9;
    double t27 = t4 * t5 * t9;
    double t29 = t2 * t9 * t11;
    double t30 = t5 * t8 * t9;
    double t31 = t4 * t9 * t11;
    double t32 = t6 * t9 * t10;
    double t33 = t3 * (9.0 / 25.0);
    double t34 = t5 * (39.0 / 50.0);
    double t35 = t7 * (59.0 / 50.0);
    double t43 = t2 * t9 * (9.0 / 25.0);
    double t44 = t8 * t9 * (9.0 / 25.0);
    double t23 = t3 * t14;
    double t24 = t3 * t16;
    double t25 = t3 * t17;
    double t28 = t3 * t19;
    double t36 = t15 * 1000.0;
    double t38 = -t27;
    double t41 = -t30;
    double t45 = t7 * t15 * 199.0;
    double t46 = t31 * 1000.0;
    double t47 = t7 * t31 * 199.0;
    double t48 = t6 * t13 * t18 * 199.0;
    double t49 = t13 * t21 * 199.0;
    double t50 = t31 * (39.0 / 50.0);
    double t53 = t15 + t31;
    double t54 = t6 * t13 * t27 * 199.0;
    double t58 = t34 - 39.0 / 50.0;
    double t59 = t35 - 59.0 / 50.0;
    double t37 = -t23;
    double t39 = -t28;
    double t51 = t16 + t25;
    double t52 = t17 + t24;
    double t57 = t18 + t38;
    double t60 = -t54;
    double t72 = t3 * t58;
    double t74 = t2 * t9 * t58;
    double t75 = t8 * t9 * t58;
    double t80 = t7 * t53 * 1.2596;
    double t89 = t53 * t59;
    double t55 = t14 + t39;
    double t56 = t19 + t37;
    double t61 = t5 * t51;
    double t62 = t6 * t52;
    double t63 = t11 * t51;
    double t64 = t12 * t52;
    double t68 = t6 * t57;
    double t71 = t12 * t57;
    double t83 = -t80;
    double t100 = t33 + t50 + t72 - 9.0 / 25.0;
    double t108 = t22 + t36 + t45 + t46 + t47 + t48 + t49 + t60;
    double t65 = t5 * t56;
    double t66 = t6 * t55;
    double t67 = -t62;
    double t69 = t11 * t56;
    double t70 = t12 * t55;
    double t76 = t63 * (39.0 / 50.0);
    double t79 = t20 + t61;
    double t81 = t41 + t63;
    double t82 = t21 + t68;
    double t94 = t7 * (t30 - t63) * (-1.2596);
    double t99 = t59 * (t30 - t63);
    double t73 = -t66;
    double t77 = -t76;
    double t78 = t69 * (39.0 / 50.0);
    double t84 = t26 + t69;
    double t85 = t6 * t79;
    double t86 = t12 * t79;
    double t90 = -t6 * (t29 - t65);
    double t91 = -t12 * (t29 - t65);
    double t92 = t13 * t82 * 0.0796;
    double t93 = t7 * t84 * 1.2596;
    double t95 = -t92;
    double t97 = t59 * t84;
    double t101 = t70 + t85;
    double t102 = t64 + t90;
    double t103 = t73 + t86;
    double t104 = t67 + t91;
    double t96 = -t93;
    double t105 = t13 * t101 * 0.0796;
    double t107 = t13 * t102 * 0.0796;
    double t109 = t83 + t89 + t95 + t100;
    double t106 = -t105;
    double t111 = t43 + t74 + t78 + t96 + t97 + t107;
    double t110 = t44 + t75 + t77 + t94 + t99 + t106;
    
    Eigen::MatrixXd dJ_T_dq1_mat( 6, 7 );
    dJ_T_dq1_mat << t111, t8 * t108 * (-0.0004), t11 * t14 * (2.0 / 5.0) - t18 * t19 * (2.0 / 5.0) + t7 * t11 * t14 * 0.0796 + t12 * t13 * t16 * 0.0796 - t7 * t18 * t19 * 0.0796 + t12 * t13 * t25 * 0.0796 - t5 * t6 * t13 * t14 * 0.0796 + t6 * t13 * t15 * t19 * 0.0796, t20 * (2.0 / 5.0) + t5 * t16 * (2.0 / 5.0) + t7 * t20 * 0.0796 + t15 * t17 * (2.0 / 5.0) + t5 * t7 * t16 * 0.0796 + t7 * t15 * t17 * 0.0796 - t6 * t13 * t30 * 0.0796 + t6 * t11 * t13 * t16 * 0.0796 + t6 * t13 * t17 * t18 * 0.0796,
                t13 * (-t6 * t14 + t12 * t20 + t6 * t28 + t5 * t12 * t16 + t12 * t15 * t17) * 0.0796, t70 * (59.0 / 50.0) + t85 * (59.0 / 50.0) - t100 * (t62 + t12 * (t29 - t65)) + t109 * (t62 + t12 * (t29 - t65)) + t111 * (t32 - t71) - (t32 - t71) * (t43 + t74 + t78), 0.0, t110, (t2 * t108) / 2500.0, t11 * t17 * (2.0 / 5.0) + t16 * t18 * (2.0 / 5.0) + t7 * t11 * t17 * 0.0796 + t7 * t16 * t18 * 0.0796 + t12 * t13 * t19 * 0.0796 - t12 * t13 * t23 * 0.0796 - t5 * t6 * t13 * t17 * 0.0796 - t6 * t13 * t15 * t16 * 0.0796,
                t29 * (-2.0 / 5.0) + t5 * t19 * (2.0 / 5.0) - t14 * t15 * (2.0 / 5.0) - t7 * t29 * 0.0796 + t5 * t7 * t19 * 0.0796 - t7 * t14 * t15 * 0.0796 + t6 * t13 * t26 * 0.0796 + t6 * t11 * t13 * t19 * 0.0796 - t6 * t13 * t14 * t18 * 0.0796, t13 * (t6 * t17 + t6 * t24 + t12 * t29 - t5 * t12 * t19 + t12 * t14 * t15) * (-0.0796), t64 * (59.0 / 50.0) - t6 * (t29 - t65) * (59.0 / 50.0) + t110 * (t32 - t71) + t100 * (t66 - t86) - t109 * (t66 - t86) - (t32 - t71) * (t44 + t75 + t77), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -t2, -t8 * t9, t55, t81, t103, -t13 * t101 - t7 * (t30 - t63), 0.0, -t8, t2 * t9, t52, t84, t104, t7 * t84 - t13 * t102, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return dJ_T_dq1_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{
    
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t3 * t5;
    double t15 = t5 * t9;
    double t16 = t9 * t11;
    double t17 = t9 * t10 * t12;
    double t18 = t3 * 1050.0;
    double t19 = t9 * 1050.0;
    double t21 = t3 * t4 * t11;
    double t22 = t3 * t6 * t10;
    double t23 = t3 * t6 * t11;
    double t26 = t6 * t9 * t10;
    double t27 = t3 * t11 * t12;
    double t29 = t3 * t11 * 1000.0;
    double t31 = t10 * t11 * 1000.0;
    double t40 = t3 * t7 * t11 * 199.0;
    double t42 = t7 * t10 * t11 * 199.0;
    double t43 = t4 * t12 * t13 * 199.0;
    double t58 = t5 * t6 * t10 * t13 * 199.0;
    double t59 = t3 * t10 * t12 * t13 * 199.0;
    double t20 = t4 * t14;
    double t24 = t13 * t14;
    double t25 = t4 * t16;
    double t28 = t14 * 1000.0;
    double t30 = t15 * 1000.0;
    double t32 = t4 * t6 * t15;
    double t33 = t7 * t23;
    double t34 = t4 * t12 * t15;
    double t36 = t7 * t17;
    double t37 = -t27;
    double t38 = -t29;
    double t39 = t7 * t14 * 199.0;
    double t41 = t7 * t15 * 199.0;
    double t48 = -t40;
    double t49 = t21 * 1000.0;
    double t51 = -t43;
    double t53 = t7 * t21 * 199.0;
    double t57 = t13 * t23 * 199.0;
    double t60 = t6 * t13 * t16 * 199.0;
    double t62 = t13 * t17 * 199.0;
    double t66 = -t58;
    double t67 = -t59;
    double t35 = t13 * t25;
    double t44 = t7 * t32;
    double t45 = -t32;
    double t46 = -t33;
    double t47 = -t36;
    double t50 = t4 * t30;
    double t52 = t25 * 1000.0;
    double t54 = t4 * t41;
    double t55 = t6 * t24 * 199.0;
    double t56 = t7 * t25 * 199.0;
    double t61 = -t49;
    double t63 = t14 + t25;
    double t64 = t16 + t20;
    double t65 = -t53;
    double t68 = t6 * t13 * t20 * 199.0;
    double t69 = t13 * t32 * 199.0;
    double t73 = t26 + t34 + t37;
    double t74 = t31 + t42 + t51 + t66;
    double t70 = t6 * t35 * 199.0;
    double t71 = -t69;
    double t72 = t17 + t23 + t45;
    double t75 = t24 + t35 + t44 + t46 + t47;
    double t78 = t19 + t30 + t41 + t60 + t61 + t65 + t67 + t68;
    double t76 = t38 + t48 + t50 + t54 + t55 + t70;
    double t77 = t18 + t28 + t39 + t52 + t56 + t57 + t62 + t71;

    Eigen::MatrixXd dJ_T_dq2_mat( 6, 7 );
    dJ_T_dq2_mat << t8 * t77 * (-0.0004), t2 * t78 * (-0.0004), t2 * t9 * t74 * (-0.0004), (t2 * t76) / 2500.0, t2 * t13 * t73 * 0.0796, t2 * t75 * (-0.0796),
                    0.0, (t2 * t77) / 2500.0, t8 * t78 * (-0.0004), t8 * t9 * t74 * (-0.0004), (t8 * t76) / 2500.0, t8 * t13 * t73 * 0.0796, t8 * t75 * (-0.0796),
                    0.0, 0.0, t3 * (-21.0 / 50.0) - t14 * (2.0 / 5.0) - t25 * (2.0 / 5.0) - t7 * t14 * 0.0796 - t13 * t17 * 0.0796 - t7 * t25 * 0.0796 - t13 * t23 * 0.0796 + t13 * t32 * 0.0796, t3 * t74 * (-0.0004), t16 * (2.0 / 5.0) + t20 * (2.0 / 5.0) + t7 * t16 * 0.0796 + t7 * t20 * 0.0796 - t6 * t13 * t15 * 0.0796 + t6 * t13 * t21 * 0.0796,
                    t13 * (t22 + t12 * t16 + t12 * t20) * 0.0796, t13 * t15 * 0.0796 - t13 * t21 * 0.0796 - t6 * t7 * t16 * 0.0796 - t6 * t7 * t20 * 0.0796 + t3 * t7 * t10 * t12 * 0.0796, 0.0, 0.0, 0.0, t2 * t3, -t2 * t9 * t10, t2 * t63, t2 * t26 - t12 * (t2 * t3 * t11 - t2 * t4 * t15),
                    t2 * t7 * t63 + t2 * t13 * t72, 0.0, 0.0, t3 * t8, -t8 * t9 * t10, t8 * t63, t8 * t26 - t12 * (t3 * t8 * t11 - t4 * t8 * t15), t7 * t8 * t63 + t8 * t13 * t72, 0.0, 0.0, -t9, -t3 * t10, -t15 + t21, t22 + t12 * t64, -t13 * (t6 * t64 - t3 * t10 * t12) - t7 * (t15 - t21);

    return dJ_T_dq2_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t4 * t6;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t8 * t10;
    double t19 = t5 * 1000.0;
    double t24 = t5 * t10 * t12;
    double t25 = t5 * t7 * 199.0;
    double t26 = t7 * (59.0 / 50.0);
    double t27 = t10 * t11 * 1000.0;
    double t31 = t7 * t10 * t11 * 199.0;
    double t32 = t4 * t12 * t13 * 199.0;
    double t33 = t6 * t11 * t13 * 199.0;
    double t35 = t5 * t6 * t10 * t13 * 199.0;
    double t20 = t3 * t14;
    double t21 = t3 * t16;
    double t22 = t3 * t17;
    double t23 = t3 * t18;
    double t30 = -t24;
    double t34 = -t32;
    double t38 = -t35;
    double t42 = t26 - 59.0 / 50.0;
    double t49 = t19 + t25 + t33;
    double t28 = -t20;
    double t29 = -t23;
    double t36 = t16 + t22;
    double t37 = t17 + t21;
    double t41 = t15 + t30;
    double t52 = t27 + t31 + t34 + t38;
    double t39 = t14 + t29;
    double t40 = t18 + t28;
    double t43 = t12 * t36;
    double t44 = t5 * t6 * t37;
    double t45 = t12 * t40;
    double t46 = t5 * t6 * t39;
    double t48 = -t46;
    double t50 = t43 + t48;

    Eigen::MatrixXd dJ_T_dq3_mat( 6, 7 );

    dJ_T_dq3_mat << t11 * t39 * (-39.0 / 50.0) + t13 * t50 * 0.0796 + t7 * t11 * t39 * 1.2596 - t11 * t39 * t42, t2 * t9 * t52 * (-0.0004), t11 * t18 * (-2.0 / 5.0) + t11 * t20 * (2.0 / 5.0) - t7 * t11 * t18 * 0.0796 + t7 * t11 * t20 * 0.0796 + t12 * t13 * t17 * 0.0796 + t12 * t13 * t21 * 0.0796 + t5 * t6 * t13 * t18 * 0.0796 - t5 * t6 * t13 * t20 * 0.0796, (t37 * t49) / 2500.0, t13 * (t6 * t18 + t6 * t28 + t5 * t12 * t17 + t5 * t12 * t21) * 0.0796,
                    t7 * t12 * t18 * 0.0796 - t7 * t12 * t20 * 0.0796 - t11 * t13 * t17 * 0.0796 - t11 * t13 * t21 * 0.0796 - t5 * t7 * t8 * t15 * 0.0796 - t5 * t6 * t7 * t21 * 0.0796, 0.0, t11 * t37 * (-39.0 / 50.0) - t13 * (t44 - t45) * 0.0796 + t7 * t11 * t37 * 1.2596 - t11 * t37 * t42, t8 * t9 * t52 * (-0.0004),
                    t11 * t16 * (2.0 / 5.0) + t11 * t22 * (2.0 / 5.0) + t7 * t11 * t16 * 0.0796 - t12 * t13 * t14 * 0.0796 + t7 * t11 * t22 * 0.0796 + t12 * t13 * t23 * 0.0796 - t5 * t6 * t13 * t16 * 0.0796 - t3 * t5 * t8 * t13 * t15 * 0.0796, t39 * t49 * (-0.0004), t13 * (t6 * t16 + t3 * t8 * t15 + t5 * t12 * t14 + t5 * t12 * t29) * (-0.0796),
                    t7 * t12 * t16 * (-0.0796) + t11 * t13 * t14 * 0.0796 - t7 * t12 * t22 * 0.0796 - t11 * t13 * t23 * 0.0796 + t5 * t6 * t7 * t14 * 0.0796 - t5 * t6 * t7 * t23 * 0.0796, 0.0, 0.0, t3 * t52 * (-0.0004), t9 * (t4 * t11 * 1000.0 + t4 * t7 * t11 * 199.0 - t5 * t13 * t15 * 199.0 + t10 * t12 * t13 * 199.0) * (-0.0004), t9 * t10 * t49 * (-0.0004), t9 * t13 * t41 * 0.0796, t9 * (t4 * t7 * t12 + t10 * t11 * t13 + t5 * t6 * t7 * t10) * 0.0796,
                    0.0, 0.0, 0.0, 0.0, -t18 + t20, t11 * t37, t6 * t40 + t5 * t12 * t37, -t13 * (t44 - t45) + t7 * t11 * t37, 0.0, 0.0, 0.0, t36, -t11 * t39, -t6 * t36 - t5 * t12 * t39, -t13 * t50 - t7 * t11 * t39, 0.0, 0.0, 0.0, -t4 * t9, -t9 * t10 * t11, t9 * t41, t13 * (t4 * t9 * t12 + t5 * t6 * t9 * t10) - t7 * t9 * t10 * t11;

    return dJ_T_dq3_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t3 * t11;
    double t18 = t8 * t10;
    double t19 = t8 * t9 * t11;
    double t20 = t5 * 1000.0;
    double t22 = t3 * t4 * t8;
    double t23 = t2 * t5 * t9;
    double t24 = t4 * t5 * t9;
    double t26 = t2 * t9 * t11;
    double t27 = t5 * t8 * t9;
    double t28 = t4 * t9 * t11;
    double t29 = t5 * t7 * 199.0;
    double t39 = t6 * t11 * t13 * 199.0;
    double t21 = t3 * t14;
    double t25 = t3 * t18;
    double t30 = t17 * 1000.0;
    double t33 = -t26;
    double t34 = -t27;
    double t35 = t19 * (2.0 / 5.0);
    double t37 = t5 * t16 * (2.0 / 5.0);
    double t38 = t7 * t17 * 199.0;
    double t41 = t4 * t9 * t20;
    double t42 = t4 * t8 * t15 * (2.0 / 5.0);
    double t43 = t7 * t24 * 199.0;
    double t44 = t6 * t13 * t15 * 199.0;
    double t45 = t16 + t22;
    double t46 = t15 + t28;
    double t47 = t6 * t13 * t28 * 199.0;
    double t52 = t5 * t7 * t16 * 0.0796;
    double t53 = t7 * t19 * 0.0796;
    double t56 = t4 * t7 * t8 * t15 * 0.0796;
    double t57 = t6 * t13 * t27 * 0.0796;
    double t58 = t6 * t11 * t13 * t16 * 0.0796;
    double t60 = t4 * t6 * t8 * t13 * t17 * 0.0796;
    double t62 = t20 + t29 + t39;
    double t31 = -t21;
    double t32 = -t25;
    double t36 = -t30;
    double t40 = -t38;
    double t50 = t5 * t45;
    double t51 = t11 * t45;
    double t59 = -t57;
    double t48 = t14 + t32;
    double t49 = t18 + t31;
    double t65 = t36 + t40 + t41 + t43 + t44 + t47;
    double t66 = t35 + t37 + t42 + t52 + t53 + t56 + t58 + t59 + t60;
    double t54 = t5 * t49;
    double t55 = t11 * t49;
    double t63 = t23 + t55;
    double t64 = t33 + t54;

    Eigen::MatrixXd dJ_T_dq4_mat( 6, 7 );

    dJ_T_dq4_mat << t66, (t2 * t65) / 2500.0, (t62 * (t4 * t8 + t3 * t16)) / 2500.0, -t48 * (t17 * (-2.0 / 5.0) + t24 * (2.0 / 5.0) - t7 * t17 * 0.0796 + t7 * t24 * 0.0796 + t6 * t13 * t15 * 0.0796 + t6 * t13 * t28 * 0.0796) - t9 * t10 * t66, t12 * t13 * (t23 + t11 * t18 - t14 * t17) * (-0.0796), t13 * t26 * 0.0796 - t5 * t13 * t18 * 0.0796 + t6 * t7 * t23 * 0.0796 + t13 * t14 * t15 * 0.0796 + t6 * t7 * t11 * t18 * 0.0796 - t6 * t7 * t14 * t17 * 0.0796, 0.0,
                    t26 * (-2.0 / 5.0) + t5 * t18 * (2.0 / 5.0) - t14 * t15 * (2.0 / 5.0) - t7 * t26 * 0.0796 + t5 * t7 * t18 * 0.0796 - t7 * t14 * t15 * 0.0796 + t6 * t13 * t23 * 0.0796 + t6 * t11 * t13 * t18 * 0.0796 - t6 * t13 * t14 * t17 * 0.0796, (t8 * t65) / 2500.0, t48 * t62 * (-0.0004),
                    t27 * (-2.0 / 5.0) + t11 * t16 * (2.0 / 5.0) - t7 * t27 * 0.0796 + t4 * t8 * t17 * (2.0 / 5.0) + t7 * t11 * t16 * 0.0796 - t6 * t13 * t19 * 0.0796 + t4 * t7 * t8 * t17 * 0.0796 - t5 * t6 * t13 * t16 * 0.0796 - t4 * t6 * t8 * t13 * t15 * 0.0796, t12 * t13 * (t34 + t11 * t16 + t4 * t8 * t17) * 0.0796, t13 * t19 * 0.0796 + t5 * t13 * t16 * 0.0796 + t6 * t7 * t27 * 0.0796 + t4 * t8 * t13 * t15 * 0.0796 - t6 * t7 * t11 * t16 * 0.0796 - t4 * t6 * t7 * t8 * t17 * 0.0796, 0.0, 0.0,
                    t4 * t15 * (2.0 / 5.0) + t9 * t11 * (2.0 / 5.0) + t4 * t7 * t15 * 0.0796 + t7 * t9 * t11 * 0.0796 - t5 * t6 * t9 * t13 * 0.0796 + t4 * t6 * t13 * t17 * 0.0796, t9 * t10 * t62 * (-0.0004), t15 * (-2.0 / 5.0) - t28 * (2.0 / 5.0) - t7 * t15 * 0.0796 - t7 * t28 * 0.0796 - t6 * t13 * t17 * 0.0796 + t6 * t13 * t24 * 0.0796, t12 * t13 * t46 * (-0.0796), t13 * t17 * 0.0796 - t13 * t24 * 0.0796 + t6 * t7 * t15 * 0.0796 + t6 * t7 * t28 * 0.0796, 0.0, 0.0,
                    0.0, 0.0, 0.0, t64, -t12 * t63, -t7 * (t26 - t54) + t6 * t13 * t63, 0.0, 0.0, 0.0, 0.0, -t19 - t50, -t12 * (t27 - t51), -t7 * (t19 + t50) + t6 * t13 * (t27 - t51), 0.0, 0.0, 0.0, 0.0, -t17 + t24, -t12 * t46, -t7 * (t17 - t24) + t6 * t13 * t46;

    return dJ_T_dq4_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t25 = t2 * t5 * t9;
    double t26 = t4 * t5 * t9;
    double t28 = t2 * t9 * t11;
    double t29 = t5 * t8 * t9;
    double t30 = t4 * t9 * t11;
    double t31 = t6 * t9 * t10;
    double t22 = t3 * t14;
    double t23 = t3 * t16;
    double t24 = t3 * t17;
    double t27 = t3 * t19;
    double t33 = -t26;
    double t36 = -t29;
    double t40 = t15 + t30;
    double t32 = -t22;
    double t34 = -t27;
    double t38 = t16 + t24;
    double t39 = t17 + t23;
    double t43 = t18 + t33;
    double t41 = t14 + t34;
    double t42 = t19 + t32;
    double t44 = t5 * t38;
    double t45 = t6 * t39;
    double t49 = t12 * t43;
    double t46 = t5 * t42;
    double t47 = t6 * t41;
    double t51 = t20 + t44;
    double t52 = t12 * t51;

    Eigen::MatrixXd dJ_T_dq5_mat( 6, 7 );
    dJ_T_dq5_mat << t13 * (t47 - t52) * (-0.0796), t2 * t13 * (t31 - t49) * 0.0796, t3 * t13 * (t47 - t52) * (-0.0796) + t8 * t9 * t13 * (t31 - t49) * 0.0796, t12 * t13 * (t25 + t11 * t19 - t14 * t18) * (-0.0796), t13 * (t29 - t11 * t38) * (t31 - t49) * 0.0796 - t13 * t40 * (t47 - t52) * (-0.0796), t7 * (t6 * t17 + t6 * t23 + t12 * t28 - t5 * t12 * t19 + t12 * t14 * t15) * (-0.0796), 0.0, t13 * (t45 + t12 * (t28 - t46)) * (-0.0796),
                    t8 * t13 * (t31 - t49) * 0.0796, t3 * t13 * (t45 + t12 * (t28 - t46)) * (-0.0796) - t2 * t9 * t13 * (t31 - t49) * 0.0796, t12 * t13 * (t36 + t11 * t16 + t17 * t18) * 0.0796, t13 * (t25 + t11 * t42) * (t31 - t49) * (-0.0796) - t13 * t40 * (t45 + t12 * (t28 - t46)) * 0.0796, t7 * (-t6 * t14 + t12 * t20 + t6 * t27 + t5 * t12 * t16 + t12 * t15 * t17) * (-0.0796), 0.0, 0.0, t13 * (t3 * t6 * t10 + t4 * t12 * t15 + t9 * t11 * t12) * 0.0796, t9 * t13 * (t4 * t6 - t5 * t10 * t12) * 0.0796,
                    t12 * t13 * t40 * (-0.0796), t13 * (t21 + t6 * t18 + t6 * t33) * (-0.0796), t7 * (t31 - t12 * t18 + t12 * t26) * 0.0796, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t12 * t39 - t6 * (t28 - t46), -t13 * (t45 + t12 * (t28 - t46)), 0.0, 0.0, 0.0, 0.0, 0.0, -t12 * t41 - t6 * t51, t13 * (t47 - t52), 0.0, 0.0, 0.0, 0.0, 0.0, -t21 - t6 * t43, t13 * (t31 - t49);

    return dJ_T_dq5_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t25 = t2 * t5 * t9;
    double t26 = t4 * t5 * t9;
    double t28 = t2 * t9 * t11;
    double t29 = t5 * t8 * t9;
    double t30 = t4 * t9 * t11;
    double t31 = t6 * t9 * t10;
    double t22 = t3 * t14;
    double t23 = t3 * t16;
    double t24 = t3 * t17;
    double t27 = t3 * t19;
    double t33 = -t26;
    double t40 = t15 + t30;
    double t32 = -t22;
    double t34 = -t27;
    double t38 = t16 + t24;
    double t39 = t17 + t23;
    double t43 = t18 + t33;
    double t53 = t13 * t40 * 0.0796;
    double t41 = t14 + t34;
    double t42 = t19 + t32;
    double t44 = t5 * t38;
    double t45 = t11 * t38;
    double t46 = t12 * t39;
    double t48 = t6 * t43;
    double t51 = t12 * t43;
    double t47 = t5 * t42;
    double t49 = t11 * t42;
    double t50 = t12 * t41;
    double t52 = t20 + t44;
    double t56 = t21 + t48;
    double t64 = t13 * (t29 - t45) * (-0.0796);
    double t57 = t25 + t49;
    double t58 = t6 * t52;
    double t61 = -t6 * (t28 - t47);
    double t63 = t7 * t56 * 0.0796;
    double t62 = t13 * t57 * 0.0796;
    double t65 = t50 + t58;
    double t66 = t46 + t61;
    double t67 = t7 * t65 * 0.0796;
    double t68 = t7 * t66 * 0.0796;
    double t70 = t64 + t67;
    double t71 = t62 + t68;

    Eigen::MatrixXd dJ_T_dq6_mat( 6, 7 );

    dJ_T_dq6_mat << -t67 + t13 * (t29 - t45) * 0.0796, -t2 * (t53 - t63), -t3 * t70 - t8 * t9 * (t53 - t63), t41 * (t53 - t63) + t9 * t10 * t70, -t40 * t70 - (t29 - t45) * (t53 - t63), -t70 * (t31 - t51) - (t53 - t63) * (t6 * t41 - t12 * t52), 0.0,
                    -t71, -t8 * (t53 - t63), -t3 * t71 + t2 * t9 * (t53 - t63), t39 * (t53 - t63) + t9 * t10 * t71, -t40 * t71 + t57 * (t53 - t63), -(t6 * t39 + t12 * (t28 - t47)) * (t53 - t63) - t71 * (t31 - t51), 0.0, 0.0,
                    t5 * t9 * t13 * 0.0796 - t4 * t13 * t18 * 0.0796 + t3 * t7 * t10 * t12 * 0.0796 - t4 * t6 * t7 * t15 * 0.0796 - t6 * t7 * t9 * t11 * 0.0796, t9 * (t4 * t7 * t12 + t10 * t11 * t13 + t5 * t6 * t7 * t10) * 0.0796, t13 * t18 * 0.0796 - t13 * t26 * 0.0796 + t6 * t7 * t15 * 0.0796 + t6 * t7 * t30 * 0.0796, t7 * (t31 - t12 * t18 + t12 * t26) * 0.0796, t7 * t15 * (-0.0796) - t13 * t21 * 0.0796 - t7 * t30 * 0.0796 - t6 * t13 * t18 * 0.0796 + t6 * t13 * t26 * 0.0796,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -t13 * t57 - t7 * t66, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t7 * t65 - t13 * (t29 - t45), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -t13 * t40 + t7 * t56;

    return dJ_T_dq6_mat.transpose( );
}

Eigen::MatrixXd dJH_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    using namespace Eigen;

    Eigen::MatrixXd dJ_T_dq7_mat = Eigen::MatrixXd::Zero(7, 6);

    return dJ_T_dq7_mat;
}


Eigen::MatrixXd dJB_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{
    // Initialize a 7x6 matrix with zeros
    Eigen::MatrixXd dJ_T_dq1_mat = Eigen::MatrixXd::Zero(7, 6);

    return dJ_T_dq1_mat;
}

Eigen::MatrixXd dJB_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = t2 * t4;
    double t17 = t3 * t5;
    double t18 = t2 * t11;
    double t19 = t4 * t9;
    double t20 = t3 * t12;
    double t21 = t5 * t10;
    double t22 = t9 * t11;
    double t23 = t10 * t12;
    double t25 = t10 * t11 * t13;
    double t36 = t3 * t6 * t11;
    double t42 = t3 * t11 * t13;
    double t43 = t6 * t10 * t11;
    double t24 = t9 * t23;
    double t26 = t3 * t16;
    double t27 = t2 * t17;
    double t28 = t4 * t17;
    double t29 = t3 * t18;
    double t30 = t3 * t19;
    double t31 = t2 * t20;
    double t32 = t2 * t21;
    double t33 = t9 * t17;
    double t34 = t4 * t20;
    double t35 = t4 * t21;
    double t37 = t3 * t22;
    double t38 = t2 * t23;
    double t39 = t9 * t20;
    double t40 = t9 * t21;
    double t41 = t4 * t23;
    double t44 = t16 * t21;
    double t45 = t16 * t23;
    double t46 = t19 * t21;
    double t47 = t6 * t10 * t18;
    double t49 = t19 * t23;
    double t50 = t10 * t13 * t18;
    double t51 = t6 * t10 * t22;
    double t54 = t10 * t13 * t22;
    double t48 = -t26;
    double t52 = -t34;
    double t53 = -t35;
    double t55 = -t37;
    double t60 = -t44;
    double t61 = -t46;
    double t64 = t18 + t30;
    double t65 = t19 + t29;
    double t66 = t17 + t41;
    double t67 = t23 + t28;
    double t80 = t27 + t45;
    double t81 = t33 + t49;
    double t68 = t16 + t55;
    double t69 = t22 + t48;
    double t70 = t20 + t53;
    double t71 = t21 + t52;
    double t72 = t5 * t64;
    double t73 = t6 * t65;
    double t74 = t6 * t67;
    double t75 = t7 * t66;
    double t76 = t12 * t64;
    double t77 = t13 * t65;
    double t78 = t13 * t67;
    double t79 = t14 * t66;
    double t92 = t31 + t60;
    double t93 = t39 + t61;
    double t94 = t7 * t80;
    double t95 = t14 * t80;
    double t96 = t7 * t81;
    double t97 = t14 * t81;
    double t82 = t5 * t69;
    double t83 = t6 * t68;
    double t85 = t6 * t70;
    double t86 = t7 * t71;
    double t87 = t12 * t69;
    double t88 = t13 * t68;
    double t89 = t13 * t70;
    double t90 = t14 * t71;
    double t100 = t6 * t92;
    double t101 = t13 * t92;
    double t102 = t6 * t93;
    double t103 = t13 * t93;
    double t104 = t24 + t72;
    double t105 = t36 + t78;
    double t121 = -t14 * (t42 - t74);
    double t122 = -t14 * (t40 - t76);
    double t107 = t25 + t85;
    double t108 = t32 + t87;
    double t110 = t8 * t105;
    double t111 = t6 * t104;
    double t112 = t15 * t105;
    double t113 = t13 * t104;
    double t124 = -t6 * (t38 - t82);
    double t126 = t50 + t100;
    double t127 = t54 + t102;
    double t139 = t86 + t121;
    double t151 = -t15 * (t90 + t7 * (t42 - t74));
    double t152 = t8 * (t90 + t7 * (t42 - t74));
    double t116 = t7 * t108;
    double t119 = t14 * t108;
    double t120 = t7 * t107;
    double t123 = t14 * t107;
    double t130 = t7 * t126;
    double t131 = t14 * t126;
    double t132 = t7 * t127;
    double t133 = t14 * t127;
    double t136 = t88 + t111;
    double t138 = t77 + t124;
    double t160 = t110 + t151;
    double t162 = t112 + t152;
    double t134 = -t130;
    double t135 = -t132;
    double t137 = t75 + t123;
    double t142 = t7 * t136;
    double t143 = t14 * t136;
    double t146 = t7 * t138;
    double t147 = t14 * t138;
    double t153 = t94 + t131;
    double t154 = t96 + t133;
    double t150 = -t147;
    double t155 = t95 + t134;
    double t156 = t97 + t135;
    double t157 = t122 + t142;
    double t159 = t119 + t146;
    double t161 = t116 + t150;
    double et1 = t8 * t42 * (-2.0 / 5.0) - t15 * t36 * 0.0796 + t6 * t8 * t23 * (2.0 / 5.0) + t6 * t8 * t28 * (2.0 / 5.0) - t8 * t14 * t21 * 0.0796 - t13 * t15 * t23 * 0.0796 + t8 * t14 * t34 * 0.0796 - t13 * t15 * t28 * 0.0796 - t7 * t8 * t42 * 0.0796 - t7 * t15 * t36 * (2.0 / 5.0) + t3 * t4 * t6 * t8 * (21.0 / 50.0) + t6 * t7 * t8 * t23 * 0.0796 + t6 * t7 * t8 * t28 * 0.0796 - t8 * t11 * t13 * t17 * (21.0 / 50.0) - t7 * t13 * t15 * t23 * (2.0 / 5.0) - t11 * t14 * t15 * t20 * (21.0 / 50.0) - t7 * t13 * t15 * t28 * (2.0 / 5.0) - t3 * t4 * t7 * t13 * t15 * (21.0 / 50.0);
    double et2 = t6 * t7 * t11 * t15 * t17 * (-21.0 / 50.0);

    Eigen::MatrixXd dJ_T_dq2_mat( 6, 7 );

    dJ_T_dq2_mat << t8 * t36 * 0.0796 - t15 * t42 * (2.0 / 5.0) + t6 * t15 * t23 * (2.0 / 5.0) + t8 * t13 * t23 * 0.0796 + t6 * t15 * t28 * (2.0 / 5.0) + t8 * t13 * t28 * 0.0796 - t14 * t15 * t21 * 0.0796 + t7 * t8 * t36 * (2.0 / 5.0) + t14 * t15 * t34 * 0.0796 - t7 * t15 * t42 * 0.0796 + t3 * t4 * t6 * t15 * (21.0 / 50.0) + t6 * t7 * t15 * t23 * 0.0796 + t7 * t8 * t13 * t23 * (2.0 / 5.0) + t8 * t11 * t14 * t20 * (21.0 / 50.0) + t6 * t7 * t15 * t28 * 0.0796 + t7 * t8 * t13 * t28 * (2.0 / 5.0) - t11 * t13 * t15 * t17 * (21.0 / 50.0) + t3 * t4 * t7 * t8 * t13 * (21.0 / 50.0) + t6 * t7 * t8 * t11 * t17 * (21.0 / 50.0),
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, et1 + et2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t14 * t36 * (2.0 / 5.0) - t7 * t11 * t20 * (21.0 / 50.0) + t13 * t14 * t23 * (2.0 / 5.0) + t13 * t14 * t28 * (2.0 / 5.0) + t3 * t4 * t13 * t14 * (21.0 / 50.0) + t6 * t11 * t14 * t17 * (21.0 / 50.0),
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t162, 0.0, 0.0, 0.0, 0.0, 0.0,
                    -t139 * (t15 * (t43 - t89) - t8 * (t79 - t120)) + t137 * t162 - t153 * (t15 * (t73 + t13 * (t38 - t82)) + t8 * t159) - (t8 * t156 - t15 * (t51 - t103)) * (t143 + t7 * (t40 - t76)) - t161 * (t8 * t155 - t15 * (t47 - t101)) + t154 * (t8 * t157 + t15 * (t83 - t113)),
                    t160, 0.0, 0.0, 0.0, 0.0, 0.0,
                    -t139 * (t8 * (t43 - t89) + t15 * (t79 - t120)) + t137 * t160 - t153 * (t8 * (t73 + t13 * (t38 - t82)) - t15 * t159) + (t15 * t156 + t8 * (t51 - t103)) * (t143 + t7 * (t40 - t76)) + t161 * (t15 * t155 + t8 * (t47 - t101)) - t154 * (t15 * t157 - t8 * (t83 - t113)),
                    -t86 + t14 * (t42 - t74), 0.0, 0.0, 0.0, 0.0, 0.0, t154 * (t143 + t7 * (t40 - t76)) * 2.0 - t137 * t139 * 2.0 + t153 * t161 * 2.0;

    return dJ_T_dq2_mat.transpose( );
}

Eigen::MatrixXd dJB_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = t2 * t4;
    double t17 = t3 * t5;
    double t18 = t4 * t6;
    double t19 = t2 * t11;
    double t20 = t4 * t9;
    double t21 = t3 * t12;
    double t22 = t9 * t11;
    double t23 = t9 * t10 * t12;
    double t24 = t10 * t11 * t13;
    double t25 = t11 * t12 * t14;
    double t29 = t2 * t5 * t10;
    double t30 = t4 * t5 * t10;
    double t32 = t4 * t7 * t13;
    double t34 = t2 * t10 * t12;
    double t35 = t5 * t9 * t10;
    double t36 = t4 * t10 * t12;
    double t37 = t4 * t10 * t13;
    double t38 = t6 * t10 * t11;
    double t39 = t5 * t11 * t13;
    double t40 = t5 * t6 * t7 * t11;
    double t44 = t7 * t10 * t11 * t12;
    double t26 = t3 * t16;
    double t27 = t3 * t19;
    double t28 = t3 * t20;
    double t31 = t10 * t18;
    double t33 = t3 * t22;
    double t41 = t5 * t38;
    double t43 = t5 * t24;
    double t45 = -t30;
    double t46 = t10 * t25;
    double t51 = -t39;
    double t53 = -t44;
    double t56 = t17 + t36;
    double t99 = t25 + t32 + t40;
    double t42 = -t26;
    double t47 = -t33;
    double t52 = -t43;
    double t54 = t19 + t28;
    double t55 = t20 + t27;
    double t59 = t21 + t45;
    double t60 = t18 + t51;
    double t64 = t7 * t56;
    double t68 = t14 * t56;
    double t69 = t37 + t41;
    double t57 = t16 + t47;
    double t58 = t22 + t42;
    double t61 = t5 * t54;
    double t62 = t6 * t54;
    double t63 = t6 * t55;
    double t65 = t12 * t54;
    double t66 = t13 * t54;
    double t67 = t13 * t55;
    double t72 = t7 * t12 * t55;
    double t73 = t12 * t14 * t55;
    double t78 = t6 * t59;
    double t82 = t13 * t59;
    double t84 = t31 + t52;
    double t85 = t7 * t69;
    double t88 = t14 * t69;
    double t70 = t5 * t63;
    double t71 = t5 * t67;
    double t74 = t5 * t58;
    double t75 = t6 * t57;
    double t76 = t6 * t58;
    double t79 = t12 * t58;
    double t80 = t13 * t57;
    double t81 = t13 * t58;
    double t90 = t7 * t12 * t57;
    double t91 = t12 * t14 * t57;
    double t94 = t8 * t84;
    double t96 = t15 * t84;
    double t98 = t23 + t61;
    double t101 = t24 + t78;
    double t111 = -t14 * (t35 - t65);
    double t113 = t46 + t85;
    double t118 = t53 + t88;
    double t86 = t5 * t75;
    double t89 = t5 * t80;
    double t102 = t29 + t79;
    double t103 = t6 * t98;
    double t104 = t13 * t98;
    double t110 = t7 * t101;
    double t112 = t14 * t101;
    double t114 = -t6 * (t34 - t74);
    double t119 = t8 * t113;
    double t120 = t15 * t113;
    double t123 = t71 + t76;
    double t133 = -t14 * (t70 - t81);
    double t149 = -t8 * (t63 + t13 * (t34 - t74));
    double t154 = t15 * (t63 + t13 * (t34 - t74));
    double t161 = -t8 * (t73 + t7 * (t70 - t81));
    double t165 = t15 * (t73 + t7 * (t70 - t81));
    double t95 = -t86;
    double t107 = t7 * t102;
    double t109 = t14 * t102;
    double t121 = -t120;
    double t122 = t62 + t89;
    double t127 = t8 * t123;
    double t129 = t15 * t123;
    double t134 = t80 + t103;
    double t135 = t64 + t112;
    double t136 = t67 + t114;
    double t144 = -t8 * (t75 - t104);
    double t145 = -t8 * (t68 - t110);
    double t153 = t15 * (t75 - t104);
    double t155 = t96 + t119;
    double t157 = t72 + t133;
    double t124 = t66 + t95;
    double t126 = t8 * t122;
    double t128 = t15 * t122;
    double t139 = t7 * t134;
    double t140 = t14 * t134;
    double t142 = t7 * t136;
    double t143 = t14 * t136;
    double t156 = t94 + t121;
    double t177 = t129 + t161;
    double t178 = t127 + t165;
    double t130 = t7 * t124;
    double t132 = t14 * t124;
    double t150 = -t143;
    double t167 = t111 + t139;
    double t169 = t109 + t142;
    double t159 = t90 + t132;
    double t163 = -t8 * (t91 - t130);
    double t166 = t15 * (t91 - t130);
    double t170 = t8 * t167;
    double t171 = t15 * t167;
    double t173 = t107 + t150;
    double t174 = t8 * t169;
    double t175 = t15 * t169;
    double t179 = t128 + t163;
    double t180 = t126 + t166;
    double t181 = t144 + t171;
    double t182 = t153 + t170;
    double t183 = t149 + t175;
    double t184 = t154 + t174;

    Eigen::MatrixXd dJ_T_dq3_mat( 6, 7 );

    dJ_T_dq3_mat << t10 * (t8 * t18 * -199 + t15 * t25 * 199 + t8 * t39 * 199 + t15 * t32 * 199 + t15 * t40 * 199 + t4 * t13 * t15 * 1000 + t6 * t11 * t15 * 1050 - t7 * t8 * t18 * 1000 + t7 * t8 * t39 * 1000 + t4 * t5 * t13 * t15 * 1050 + t5 * t6 * t11 * t15 * 1000 - t4 * t8 * t12 * t14 * 1050 - t5 * t7 * t8 * t18 * 1050 + t7 * t8 * t11 * t13 * 1050) * (-0.0004),
                      t8 * t25 * (-21.0 / 50.0) - t15 * t18 * (21.0 / 50.0) - t8 * t32 * (21.0 / 50.0) - t8 * t40 * (21.0 / 50.0) + t15 * t39 * (21.0 / 50.0) - t6 * t8 * t11 * 0.0796 - t5 * t15 * t18 * (2.0 / 5.0) + t11 * t13 * t15 * (2.0 / 5.0) - t5 * t8 * t32 * (2.0 / 5.0) - t4 * t5 * t8 * t13 * 0.0796 - t6 * t7 * t8 * t11 * (2.0 / 5.0) - t4 * t12 * t14 * t15 * 0.0796 - t5 * t7 * t15 * t18 * 0.0796 + t7 * t11 * t13 * t15 * 0.0796,
                      0.0, 0.0, 0.0, 0.0, 0.0,
                      t10 * (t8 * t25 * 199 + t15 * t18 * 199 + t8 * t32 * 199 + t8 * t40 * 199 - t15 * t39 * 199 + t4 * t8 * t13 * 1000 + t6 * t8 * t11 * 1050 + t7 * t15 * t18 * 1000 - t7 * t15 * t39 * 1000 + t4 * t5 * t8 * t13 * 1050 + t5 * t6 * t8 * t11 * 1000 + t4 * t12 * t14 * t15 * 1050 + t5 * t7 * t15 * t18 * 1050 - t7 * t11 * t13 * t15 * 1050) * (-0.0004),
                      t8 * t18 * (-21.0 / 50.0) + t15 * t25 * (21.0 / 50.0) + t8 * t39 * (21.0 / 50.0) + t15 * t32 * (21.0 / 50.0) + t15 * t40 * (21.0 / 50.0) - t5 * t8 * t18 * (2.0 / 5.0) + t6 * t11 * t15 * 0.0796 + t8 * t11 * t13 * (2.0 / 5.0) + t5 * t15 * t32 * (2.0 / 5.0) + t4 * t5 * t13 * t15 * 0.0796 - t4 * t8 * t12 * t14 * 0.0796 - t5 * t7 * t8 * t18 * 0.0796 + t6 * t7 * t11 * t15 * (2.0 / 5.0) + t7 * t8 * t11 * t13 * 0.0796,
                      0.0, 0.0, 0.0, 0.0, 0.0,
                      t10 * (t14 * t18 * -20 + t14 * t39 * 20 + t4 * t7 * t12 * 21 - t5 * t14 * t18 * 21 + t11 * t13 * t14 * 21) * (-1.0 / 50.0),
                      t7 * t11 * t12 * (21.0 / 50.0) - t4 * t13 * t14 * (21.0 / 50.0) - t6 * t11 * t14 * (2.0 / 5.0) - t4 * t5 * t13 * t14 * (2.0 / 5.0) - t5 * t6 * t11 * t14 * (21.0 / 50.0),
                      0.0, 0.0, 0.0, 0.0, 0.0,
                      t10 * t15 * t60 + t8 * t10 * t99,
                      -t6 * t11 * t15 - t4 * t5 * t13 * t15 + t4 * t8 * t12 * t14 + t5 * t7 * t8 * t18 - t7 * t8 * t11 * t13,
                      0.0, t55 * t177 + t54 * t182 + t57 * t179 + t58 * t184 - t4 * t10 * (t145 + t15 * (t38 - t82)) - t10 * t11 * t155,
                      t56 * t155 + t102 * t177 - t179 * (t35 - t65) - t12 * t55 * t184 - t12 * t57 * t182 - t10 * t11 * t12 * (t145 + t15 * (t38 - t82)),
                      -t177 * (t63 + t13 * (t34 - t74)) + t84 * (t145 + t15 * (t38 - t82)) - t122 * t182 - t123 * t184 + t155 * (t38 - t82) - t179 * (t75 - t104),
                      -t179 * (t140 + t7 * (t35 - t65)) + t135 * t155 - t157 * t184 - t159 * t182 + t173 * t177 - (t145 + t15 * (t38 - t82)) * (t44 - t88),
                      t8 * t10 * t60 - t10 * t15 * t99,
                      -t6 * t8 * t11 - t4 * t5 * t8 * t13 - t4 * t12 * t14 * t15 - t5 * t7 * t15 * t18 + t7 * t11 * t13 * t15,
                      0.0, t55 * t178 - t54 * t181 + t57 * t180 - t58 * t183 - t4 * t10 * (t8 * (t38 - t82) + t15 * (t68 - t110)) - t10 * t11 * t156,
                      t56 * t156 + t102 * t178 - t180 * (t35 - t65) + t12 * t55 * t183 + t12 * t57 * t181 - t10 * t11 * t12 * (t8 * (t38 - t82) + t15 * (t68 - t110)),
                      -t178 * (t63 + t13 * (t34 - t74)) + t84 * (t8 * (t38 - t82) + t15 * (t68 - t110)) + t122 * t181 + t123 * t183 + t156 * (t38 - t82) - t180 * (t75 - t104),
                      -t180 * (t140 + t7 * (t35 - t65)) + t135 * t156 + t157 * t183 + t159 * t181 + t173 * t178 - (t8 * (t38 - t82) + t15 * (t68 - t110)) * (t44 - t88),
                      t118, -t4 * t7 * t12 + t5 * t14 * t18 - t11 * t13 * t14,
                      0.0, 0.0,
                      t102 * t157 - t56 * (t44 - t88) - t159 * (t35 - t65) - t12 * t57 * (t140 + t7 * (t35 - t65)) + t12 * t55 * t173 - t10 * t11 * t12 * t135,
                      -t157 * (t63 + t13 * (t34 - t74)) - t122 * (t140 + t7 * (t35 - t65)) + t84 * t135 + t123 * t173 - t159 * (t75 - t104) - (t38 - t82) * (t44 - t88),
                      t159 * (t140 + t7 * (t35 - t65)) * -2.0 + t157 * t173 * 2.0 - t135 * (t44 - t88) * 2.0;

    return dJ_T_dq3_mat.transpose( );
}


Eigen::MatrixXd dJB_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = t2 * t4;
    double t17 = t3 * t5;
    double t18 = t2 * t11;
    double t19 = t4 * t9;
    double t20 = t3 * t12;
    double t21 = t9 * t11;
    double t22 = t9 * t10 * t12;
    double t23 = t10 * t11 * t13;
    double t27 = t2 * t5 * t10;
    double t28 = t4 * t5 * t10;
    double t30 = t2 * t10 * t12;
    double t31 = t5 * t9 * t10;
    double t32 = t4 * t10 * t12;
    double t33 = t6 * t10 * t11;
    double t24 = t3 * t16;
    double t25 = t3 * t18;
    double t26 = t3 * t19;
    double t29 = t3 * t21;
    double t35 = -t28;
    double t42 = t17 + t32;
    double t34 = -t24;
    double t36 = -t29;
    double t40 = t18 + t26;
    double t41 = t19 + t25;
    double t45 = t20 + t35;
    double t48 = t7 * t42;
    double t51 = t14 * t42;
    double t54 = t8 * t13 * t42;
    double t55 = t13 * t15 * t42;
    double t43 = t16 + t36;
    double t44 = t21 + t34;
    double t46 = t5 * t40;
    double t47 = t6 * t41;
    double t49 = t12 * t40;
    double t50 = t13 * t41;
    double t52 = t6 * t48;
    double t53 = t6 * t51;
    double t59 = t6 * t45;
    double t60 = t7 * t45;
    double t63 = t13 * t45;
    double t64 = t14 * t45;
    double t67 = -t55;
    double t56 = t5 * t44;
    double t57 = t6 * t43;
    double t61 = t12 * t44;
    double t62 = t13 * t43;
    double t69 = t22 + t46;
    double t71 = t23 + t59;
    double t83 = -t14 * (t31 - t49);
    double t93 = -t13 * t15 * (t31 - t49);
    double t99 = t6 * t7 * (t31 - t49);
    double t101 = t8 * t13 * (t31 - t49);
    double t103 = t52 + t64;
    double t72 = t27 + t61;
    double t73 = t6 * t69;
    double t74 = t7 * t69;
    double t75 = t13 * t69;
    double t76 = t14 * t69;
    double t82 = t7 * t71;
    double t84 = t14 * t71;
    double t88 = t6 * t83;
    double t91 = -t6 * (t30 - t56);
    double t92 = -t7 * (t30 - t56);
    double t105 = t8 * t103;
    double t106 = t15 * t103;
    double t122 = -t8 * (t47 + t13 * (t30 - t56));
    double t127 = t15 * (t47 + t13 * (t30 - t56));
    double t79 = t7 * t72;
    double t81 = t14 * t72;
    double t87 = t8 * t13 * t72;
    double t90 = t13 * t15 * t72;
    double t107 = t62 + t73;
    double t108 = t48 + t84;
    double t109 = t50 + t91;
    double t117 = -t8 * (t57 - t75);
    double t118 = -t8 * (t51 - t82);
    double t126 = t15 * (t57 - t75);
    double t128 = t54 + t106;
    double t129 = t74 + t88;
    double t130 = t67 + t105;
    double t131 = t76 + t99;
    double t86 = t6 * t81;
    double t98 = t6 * t79;
    double t112 = t7 * t107;
    double t113 = t14 * t107;
    double t115 = t7 * t109;
    double t116 = t14 * t109;
    double t133 = t8 * t131;
    double t134 = t15 * t131;
    double t123 = -t116;
    double t132 = t86 + t92;
    double t136 = -t8 * (t98 + t14 * (t30 - t56));
    double t138 = t83 + t112;
    double t140 = t81 + t115;
    double t148 = t93 + t133;
    double t149 = t101 + t134;
    double t141 = t8 * t138;
    double t142 = t15 * t138;
    double t144 = t79 + t123;
    double t145 = t8 * t140;
    double t146 = t15 * t140;
    double t150 = t90 + t136;
    double t152 = t117 + t142;
    double t153 = t126 + t141;
    double t154 = t122 + t146;
    double t155 = t127 + t145;

    Eigen::MatrixXd dJ_T_dq4_mat( 6, 7 );

    dJ_T_dq4_mat << t6 * t15 * t17 * (-2.0 / 5.0) - t8 * t13 * t17 * 0.0796 - t14 * t15 * t20 * 0.0796 + t12 * t15 * t23 * (21.0 / 50.0) - t6 * t15 * t32 * (2.0 / 5.0) - t8 * t13 * t32 * 0.0796 + t14 * t15 * t28 * 0.0796 - t6 * t7 * t15 * t17 * 0.0796 - t7 * t8 * t13 * t17 * (2.0 / 5.0) - t6 * t7 * t15 * t32 * 0.0796 - t7 * t8 * t12 * t33 * (21.0 / 50.0) - t7 * t8 * t13 * t32 * (2.0 / 5.0) + t5 * t8 * t10 * t11 * t14 * (21.0 / 50.0),
                      t4 * t5 * t8 * t14 * (21.0 / 50.0) + t4 * t12 * t13 * t15 * (21.0 / 50.0) + t6 * t11 * t12 * t15 * (2.0 / 5.0) + t8 * t11 * t12 * t13 * 0.0796 - t5 * t11 * t14 * t15 * 0.0796 - t4 * t6 * t7 * t8 * t12 * (21.0 / 50.0) + t6 * t7 * t11 * t12 * t15 * 0.0796 + t7 * t8 * t11 * t12 * t13 * (2.0 / 5.0),
                      t5 * t6 * t15 * (-2.0 / 5.0) - t5 * t8 * t13 * 0.0796 - t12 * t14 * t15 * 0.0796 - t5 * t6 * t7 * t15 * 0.0796 - t5 * t7 * t8 * t13 * (2.0 / 5.0),
                      0.0, 0.0, 0.0, 0.0,
                      t6 * t8 * t17 * (-2.0 / 5.0) - t8 * t14 * t20 * 0.0796 + t8 * t12 * t23 * (21.0 / 50.0) + t13 * t15 * t17 * 0.0796 - t6 * t8 * t32 * (2.0 / 5.0) + t8 * t14 * t28 * 0.0796 + t13 * t15 * t32 * 0.0796 - t6 * t7 * t8 * t17 * 0.0796 + t7 * t13 * t15 * t17 * (2.0 / 5.0) - t6 * t7 * t8 * t32 * 0.0796 + t7 * t12 * t15 * t33 * (21.0 / 50.0) + t7 * t13 * t15 * t32 * (2.0 / 5.0) - t5 * t10 * t11 * t14 * t15 * (21.0 / 50.0),
                      t4 * t8 * t12 * t13 * (21.0 / 50.0) + t6 * t8 * t11 * t12 * (2.0 / 5.0) - t4 * t5 * t14 * t15 * (21.0 / 50.0) - t5 * t8 * t11 * t14 * 0.0796 - t11 * t12 * t13 * t15 * 0.0796 + t4 * t6 * t7 * t12 * t15 * (21.0 / 50.0) + t6 * t7 * t8 * t11 * t12 * 0.0796 - t7 * t11 * t12 * t13 * t15 * (2.0 / 5.0),
                      t5 * t6 * t8 * (-2.0 / 5.0) + t5 * t13 * t15 * 0.0796 - t8 * t12 * t14 * 0.0796 - t5 * t6 * t7 * t8 * 0.0796 + t5 * t7 * t13 * t15 * (2.0 / 5.0),
                      0.0, 0.0, 0.0, 0.0,
                      t13 * t14 * t17 * (-2.0 / 5.0) - t12 * t14 * t33 * (21.0 / 50.0) - t13 * t14 * t32 * (2.0 / 5.0) - t5 * t7 * t10 * t11 * (21.0 / 50.0),
                      t4 * t5 * t7 * (-21.0 / 50.0) - t4 * t6 * t12 * t14 * (21.0 / 50.0) + t11 * t12 * t13 * t14 * (2.0 / 5.0),
                      t5 * t13 * t14 * (-2.0 / 5.0),
                      0.0, 0.0, 0.0, 0.0, t130,
                      t11 * (t5 * t8 * t14 + t12 * t13 * t15 - t6 * t7 * t8 * t12),
                      -t3 * (t55 - t105) - t2 * t10 * t150 + t9 * t10 * t148,
                      -t41 * t150 - t43 * t148 + t10 * t11 * (t55 - t105),
                      -t45 * (t118 + t15 * (t33 - t63)) - t69 * t153 - t72 * t150 - t42 * (t55 - t105) + t148 * (t31 - t49) + t155 * (t30 - t56),
                      t150 * (t47 + t13 * (t30 - t56)) + t148 * (t57 - t75) - (t33 - t63) * (t55 - t105) - t13 * t42 * (t118 + t15 * (t33 - t63)) + t13 * t72 * t155 - t13 * t153 * (t31 - t49),
                      t148 * (t113 + t7 * (t31 - t49)) - t129 * t153 - t132 * t155 - t144 * t150 + (t118 + t15 * (t33 - t63)) * (t53 - t60) - t108 * (t55 - t105),
                      -t128, t11 * (t8 * t12 * t13 - t5 * t14 * t15 + t6 * t7 * t12 * t15),
                      -t3 * t128 - t2 * t10 * (t87 + t15 * (t98 + t14 * (t30 - t56))) - t9 * t10 * t149,
                      t43 * t149 - t41 * (t87 + t15 * (t98 + t14 * (t30 - t56))) + t10 * t11 * t128,
                      -t45 * (t8 * (t33 - t63) + t15 * (t51 - t82)) - t42 * t128 + t69 * t152 - t149 * (t31 - t49) - t154 * (t30 - t56) - t72 * (t87 + t15 * (t98 + t14 * (t30 - t56))),
                      -t128 * (t33 - t63) - t149 * (t57 - t75) + (t47 + t13 * (t30 - t56)) * (t87 + t15 * (t98 + t14 * (t30 - t56))) - t13 * t42 * (t8 * (t33 - t63) + t15 * (t51 - t82)) - t13 * t72 * t154 + t13 * t152 * (t31 - t49),
                      -t149 * (t113 + t7 * (t31 - t49)) - t108 * t128 + t129 * t152 + t132 * t154 + (t8 * (t33 - t63) + t15 * (t51 - t82)) * (t53 - t60) - t144 * (t87 + t15 * (t98 + t14 * (t30 - t56))),
                      t53 - t60, -t11 * (t5 * t7 + t6 * t12 * t14),
                      -t7 * t12 + t5 * t6 * t14,
                      t43 * t129 + t41 * t132 - t10 * t11 * (t53 - t60),
                      -t69 * (t113 + t7 * (t31 - t49)) - t45 * t108 + t72 * t132 + t42 * (t53 - t60) - t129 * (t31 - t49) - t144 * (t30 - t56),
                      -t132 * (t47 + t13 * (t30 - t56)) - t129 * (t57 - t75) + (t33 - t63) * (t53 - t60) - t13 * t42 * t108 - t13 * t72 * t144 - t13 * (t113 + t7 * (t31 - t49)) * (t31 - t49),
                      t129 * (t113 + t7 * (t31 - t49)) * -2.0 + t132 * t144 * 2.0 + t108 * (t53 - t60) * 2.0;

    return dJ_T_dq4_mat.transpose( );
}

Eigen::MatrixXd dJB_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = t2 * t4;
    double t17 = t3 * t5;
    double t18 = t2 * t11;
    double t19 = t4 * t9;
    double t20 = t3 * t12;
    double t21 = t9 * t11;
    double t22 = t9 * t10 * t12;
    double t23 = t10 * t11 * t13;
    double t27 = t2 * t5 * t10;
    double t28 = t4 * t5 * t10;
    double t30 = t2 * t10 * t12;
    double t31 = t5 * t9 * t10;
    double t32 = t4 * t10 * t12;
    double t33 = t6 * t10 * t11;
    double t24 = t3 * t16;
    double t25 = t3 * t18;
    double t26 = t3 * t19;
    double t29 = t3 * t21;
    double t35 = -t28;
    double t42 = t17 + t32;
    double t34 = -t24;
    double t36 = -t29;
    double t40 = t18 + t26;
    double t41 = t19 + t25;
    double t45 = t20 + t35;
    double t48 = t7 * t42;
    double t51 = t14 * t42;
    double t43 = t16 + t36;
    double t44 = t21 + t34;
    double t46 = t5 * t40;
    double t47 = t6 * t41;
    double t49 = t12 * t40;
    double t50 = t13 * t41;
    double t55 = t6 * t45;
    double t58 = t13 * t45;
    double t52 = t5 * t44;
    double t53 = t6 * t43;
    double t56 = t12 * t44;
    double t57 = t13 * t43;
    double t61 = t22 + t46;
    double t63 = t23 + t55;
    double t74 = -t14 * (t31 - t49);
    double t79 = -t8 * (t33 - t58);
    double t80 = -t15 * (t33 - t58);
    double t83 = t7 * t15 * (t33 - t58);
    double t64 = t27 + t56;
    double t65 = t6 * t61;
    double t66 = t13 * t61;
    double t72 = t7 * t63;
    double t73 = t8 * t63;
    double t75 = t14 * t63;
    double t76 = t15 * t63;
    double t77 = -t6 * (t30 - t52);
    double t81 = t7 * t79;
    double t82 = t7 * t80;
    double t105 = -t8 * (t47 + t13 * (t30 - t52));
    double t113 = t15 * (t47 + t13 * (t30 - t52));
    double t69 = t7 * t64;
    double t71 = t14 * t64;
    double t84 = t57 + t65;
    double t85 = t48 + t75;
    double t86 = t50 + t77;
    double t97 = -t8 * (t53 - t66);
    double t99 = -t8 * (t51 - t72);
    double t109 = t15 * (t53 - t66);
    double t110 = t7 * t105;
    double t114 = t7 * t113;
    double t115 = t76 + t81;
    double t116 = t73 + t83;
    double t89 = t7 * t84;
    double t90 = t8 * t84;
    double t91 = t14 * t84;
    double t92 = t15 * t84;
    double t94 = t7 * t86;
    double t95 = t8 * t86;
    double t96 = t14 * t86;
    double t98 = t15 * t86;
    double t103 = t7 * t97;
    double t112 = t7 * t109;
    double t106 = -t96;
    double t117 = t74 + t89;
    double t119 = t71 + t94;
    double t127 = t92 + t103;
    double t128 = t90 + t112;
    double t129 = t98 + t110;
    double t130 = t95 + t114;
    double t120 = t8 * t117;
    double t121 = t15 * t117;
    double t123 = t69 + t106;
    double t124 = t8 * t119;
    double t125 = t15 * t119;
    double t131 = t97 + t121;
    double t132 = t109 + t120;
    double t133 = t105 + t125;
    double t134 = t113 + t124;

    Eigen::MatrixXd dJ_T_dq5_mat( 6, 7 );
    dJ_T_dq5_mat << t8 * t23 * (-7.96e-2) - t15 * t33 * (2.0 / 5.0) - t6 * t8 * t20 * 7.96e-2 - t7 * t8 * t23 * (2.0 / 5.0) + t6 * t8 * t28 * 7.96e-2 + t13 * t15 * t20 * (2.0 / 5.0) - t5 * t15 * t33 * (21.0 / 50.0) - t7 * t15 * t33 * 7.96e-2 - t13 * t15 * t28 * (2.0 / 5.0) - t6 * t7 * t8 * t20 * (2.0 / 5.0) - t4 * t10 * t13 * t15 * (21.0 / 50.0) - t5 * t7 * t8 * t23 * (21.0 / 50.0) + t6 * t7 * t8 * t28 * (2.0 / 5.0) + t7 * t13 * t15 * t20 * 7.96e-2 - t7 * t13 * t15 * t28 * 7.96e-2 + t4 * t6 * t7 * t8 * t10 * (21.0 / 50.0),
                        t4 * t6 * t15 * (-2.0 / 5.0) - t4 * t8 * t13 * 7.96e-2 + t11 * t13 * t15 * (21.0 / 50.0) - t4 * t5 * t6 * t15 * (21.0 / 50.0) - t5 * t6 * t8 * t11 * 7.96e-2 - t4 * t6 * t7 * t15 * 7.96e-2 - t4 * t7 * t8 * t13 * (2.0 / 5.0) - t6 * t7 * t8 * t11 * (21.0 / 50.0) + t5 * t11 * t13 * t15 * (2.0 / 5.0) - t4 * t5 * t7 * t8 * t13 * (21.0 / 50.0) - t5 * t6 * t7 * t8 * t11 * (2.0 / 5.0) + t5 * t7 * t11 * t13 * t15 * 7.96e-2,
                        t12 * (t6 * t8 * 1.99e+2 - t13 * t15 * 1.0e+3 + t6 * t7 * t8 * 1.0e+3 - t7 * t13 * t15 * 1.99e+2) * (-4.0e-4),
                        t6 * t15 * (2.0 / 5.0) + t8 * t13 * 7.96e-2 + t6 * t7 * t15 * 7.96e-2 + t7 * t8 * t13 * (2.0 / 5.0),
                        0.0, 0.0, 0.0,
                        t15 * t23 * 7.96e-2 - t8 * t33 * (2.0 / 5.0) + t6 * t15 * t20 * 7.96e-2 + t8 * t13 * t20 * (2.0 / 5.0) + t7 * t15 * t23 * (2.0 / 5.0) - t5 * t8 * t33 * (21.0 / 50.0) - t7 * t8 * t33 * 7.96e-2 - t6 * t15 * t28 * 7.96e-2 - t8 * t13 * t28 * (2.0 / 5.0) - t4 * t8 * t10 * t13 * (21.0 / 50.0) + t6 * t7 * t15 * t20 * (2.0 / 5.0) + t7 * t8 * t13 * t20 * 7.96e-2 + t5 * t7 * t15 * t23 * (21.0 / 50.0) - t6 * t7 * t15 * t28 * (2.0 / 5.0) - t7 * t8 * t13 * t28 * 7.96e-2 - t4 * t6 * t7 * t10 * t15 * (21.0 / 50.0),
                        t4 * t6 * t8 * (-2.0 / 5.0) + t4 * t13 * t15 * 7.96e-2 + t8 * t11 * t13 * (21.0 / 50.0) - t4 * t5 * t6 * t8 * (21.0 / 50.0) - t4 * t6 * t7 * t8 * 7.96e-2 + t5 * t6 * t11 * t15 * 7.96e-2 + t5 * t8 * t11 * t13 * (2.0 / 5.0) + t4 * t7 * t13 * t15 * (2.0 / 5.0) + t6 * t7 * t11 * t15 * (21.0 / 50.0) + t4 * t5 * t7 * t13 * t15 * (21.0 / 50.0) + t5 * t6 * t7 * t11 * t15 * (2.0 / 5.0) + t5 * t7 * t8 * t11 * t13 * 7.96e-2,
                        (t12 * (t6 * t15 * 1.99e+2 + t8 * t13 * 1.0e+3 + t6 * t7 * t15 * 1.0e+3 + t7 * t8 * t13 * 1.99e+2)) / 2.5e+3,
                        t6 * t8 * (2.0 / 5.0) - t13 * t15 * 7.96e-2 + t6 * t7 * t8 * 7.96e-2 - t7 * t13 * t15 * (2.0 / 5.0),
                        0.0, 0.0, 0.0,
                        t14 * (t23 * 2.0e+1 + t6 * t20 * 2.0e+1 + t5 * t23 * 2.1e+1 - t6 * t28 * 2.0e+1 - t4 * t6 * t10 * 2.1e+1) * (-1.0 / 5.0e+1),
                        t14 * (t4 * t13 * 2.0e+1 + t6 * t11 * 2.1e+1 + t4 * t5 * t13 * 2.1e+1 + t5 * t6 * t11 * 2.0e+1) * (-1.0 / 5.0e+1),
                        t6 * t12 * t14 * (-2.0 / 5.0),
                        t13 * t14 * (2.0 / 5.0),
                        0.0, 0.0, 0.0,
                        -t76 + t7 * t8 * (t33 - t58),
                        -t4 * t13 * t15 + t4 * t6 * t7 * t8 - t5 * t6 * t11 * t15 - t5 * t7 * t8 * t11 * t13,
                        -t3 * t115 + t2 * t10 * t129 - t9 * t10 * t127,
                        t41 * t129 + t43 * t127 + t10 * t11 * t115,
                        -t42 * t115 + t64 * t129 - t127 * (t31 - t49),
                        -t63 * (t99 + t15 * (t33 - t58)) - t129 * (t47 + t13 * (t30 - t52)) - t84 * t132 - t86 * t134 - t115 * (t33 - t58) - t127 * (t53 - t66),
                        -t127 * (t91 + t7 * (t31 - t49)) - t85 * t115 + t123 * t129 + t14 * t134 * (t47 + t13 * (t30 - t52)) + t14 * (t99 + t15 * (t33 - t58)) * (t33 - t58) + t14 * t132 * (t53 - t66),
                        -t73 + t82, -t4 * t8 * t13 - t5 * t6 * t8 * t11 - t4 * t6 * t7 * t15 + t5 * t7 * t11 * t13 * t15,
                        -t12 * (t6 * t8 - t7 * t13 * t15), t41 * t130 + t43 * t128 + t10 * t11 * t116,
                        -t42 * t116 + t64 * t130 - t128 * (t31 - t49),
                        -t130 * (t47 + t13 * (t30 - t52)) - t63 * (t8 * (t33 - t58) + t15 * (t51 - t72)) + t84 * t131 + t86 * t133 - t116 * (t33 - t58) - t128 * (t53 - t66),
                        -t128 * (t91 + t7 * (t31 - t49)) - t85 * t116 + t123 * t130 - t14 * t133 * (t47 + t13 * (t30 - t52)) + t14 * (t8 * (t33 - t58) + t15 * (t51 - t72)) * (t33 - t58) - t14 * t131 * (t53 - t66),
                        t14 * (t33 - t58), t14 * (t4 * t6 - t5 * t11 * t13), -t12 * t13 * t14,
                        -t14 * t41 * (t47 + t13 * (t30 - t52)) - t14 * t43 * (t53 - t66) - t10 * t11 * t14 * (t33 - t58),
                        -t71 * (t47 + t13 * (t30 - t52)) + t51 * (t33 - t58) + t14 * (t31 - t49) * (t53 - t66),
                        -t84 * (t91 + t7 * (t31 - t49)) - t63 * t85 + t86 * t123 + t14 * (t47 + t13 * (t30 - t52)) + t14 * (t33 - t58) + t14 * (t53 - t66),
                        t14 * t123 * (t47 + t13 * (t30 - t52)) * -2.0 + t14 * (t91 + t7 * (t31 - t49)) * (t53 - t66) * 2.0 + t14 * t85 * (t33 - t58) * 2.0;

    return dJ_T_dq5_mat.transpose( );
}

Eigen::MatrixXd dJB_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7) 
{

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = q5 + q7;
    double t17 = -q7;
    double t18 = t2 * t4;
    double t19 = t3 * t5;
    double t20 = t2 * t11;
    double t21 = t4 * t9;
    double t22 = t3 * t12;
    double t23 = t9 * t11;
    double t24 = q5 + t17;
    double t25 = t9 * t10 * t12;
    double t26 = t10 * t11 * t13;
    double t30 = t2 * t5 * t10;
    double t31 = t4 * t5 * t10;
    double t33 = t2 * t10 * t12;
    double t34 = t5 * t9 * t10;
    double t35 = t4 * t10 * t12;
    double t36 = t6 * t10 * t11;
    double t37 = t7 * t11 * t12;
    double t38 = t4 * t13 * t14;
    double t39 = t5 * t6 * t11 * t14;
    double t27 = t3 * t18;
    double t28 = t3 * t20;
    double t29 = t3 * t21;
    double t32 = t3 * t23;
    double t41 = -t31;
    double t46 = -t37;
    double t49 = t19 + t35;
    double t40 = -t27;
    double t42 = -t32;
    double t47 = t20 + t29;
    double t48 = t21 + t28;
    double t52 = t22 + t41;
    double t55 = t7 * t49;
    double t58 = t14 * t49;
    double t71 = t38 + t39 + t46;
    double t50 = t18 + t42;
    double t51 = t23 + t40;
    double t53 = t5 * t47;
    double t54 = t6 * t48;
    double t56 = t12 * t47;
    double t57 = t13 * t48;
    double t62 = t6 * t52;
    double t65 = t13 * t52;
    double t66 = -t58;
    double t59 = t5 * t51;
    double t60 = t6 * t50;
    double t63 = t12 * t51;
    double t64 = t13 * t50;
    double t68 = t25 + t53;
    double t70 = t26 + t62;
    double t81 = -t14 * (t34 - t56);
    double t72 = t30 + t63;
    double t73 = t6 * t68;
    double t74 = t13 * t68;
    double t80 = t7 * t70;
    double t82 = t14 * t70;
    double t83 = -t6 * (t33 - t59);
    double t77 = t7 * t72;
    double t79 = t14 * t72;
    double t85 = t64 + t73;
    double t86 = t55 + t82;
    double t87 = t57 + t83;
    double t89 = t66 + t80;
    double t90 = t86 * t86;
    double t91 = t7 * t85;
    double t92 = t14 * t85;
    double t94 = t7 * t87;
    double t95 = t14 * t87;
    double t97 = -t95;
    double t98 = t81 + t91;
    double t100 = t79 + t94;
    double t101 = (t92 + t7 * (t34 - t56)) * (t92 + t7 * (t34 - t56));
    double t102 = t77 + t97;
    double t103 = t102 * t102;

    Eigen::MatrixXd dJ_T_dq6_mat( 6, 7 );

    dJ_T_dq6_mat << t7 * t15 * t19 * 7.96e-2 + t8 * t10 * t37 * (21.0 / 50.0) + t14 * t15 * t26 * 7.96e-2 - t8 * t10 * t38 * (21.0 / 50.0) + t7 * t15 * t35 * 7.96e-2 - t8 * t14 * t36 * (2.0 / 5.0) + t6 * t14 * t15 * t22 * 7.96e-2 + t8 * t13 * t14 * t22 * (2.0 / 5.0) - t5 * t8 * t14 * t36 * (21.0 / 50.0) - t6 * t14 * t15 * t31 * 7.96e-2 - t8 * t13 * t14 * t31 * (2.0 / 5.0),
                        t15 * t37 * (-7.96e-2) + t15 * t38 * 7.96e-2 + t15 * t39 * 7.96e-2 + t4 * t7 * t8 * t12 * (21.0 / 50.0) - t4 * t6 * t8 * t14 * (2.0 / 5.0) + t8 * t11 * t13 * t14 * (21.0 / 50.0) - t4 * t5 * t6 * t8 * t14 * (21.0 / 50.0) + t5 * t8 * t11 * t13 * t14 * (2.0 / 5.0),
                        t5 * t7 * t15 * 7.96e-2 + t6 * t12 * t14 * t15 * 7.96e-2 + t8 * t12 * t13 * t14 * (2.0 / 5.0),
                        (t14 * (cos(t16) * 5.995e+2 + cos(t24) * (8.01e+2 / 2.0))) / 2.5e+3,
                        t7 * t15 * 7.96e-2, 0.0, 0.0,
                        t7 * t8 * t19 * 7.96e-2 + t8 * t14 * t26 * 7.96e-2 + t7 * t8 * t35 * 7.96e-2 - t10 * t15 * t37 * (21.0 / 50.0) + t10 * t15 * t38 * (21.0 / 50.0) + t14 * t15 * t36 * (2.0 / 5.0) + t6 * t8 * t14 * t22 * 7.96e-2 - t6 * t8 * t14 * t31 * 7.96e-2 - t13 * t14 * t15 * t22 * (2.0 / 5.0) + t5 * t14 * t15 * t36 * (21.0 / 50.0) + t13 * t14 * t15 * t31 * (2.0 / 5.0),
                        t8 * t37 * (-7.96e-2) + t8 * t38 * 7.96e-2 + t8 * t39 * 7.96e-2 - t4 * t7 * t12 * t15 * (21.0 / 50.0) + t4 * t6 * t14 * t15 * (2.0 / 5.0) - t11 * t13 * t14 * t15 * (21.0 / 50.0) + t4 * t5 * t6 * t14 * t15 * (21.0 / 50.0) - t5 * t11 * t13 * t14 * t15 * (2.0 / 5.0),
                        t5 * t7 * t8 * 7.96e-2 + t6 * t8 * t12 * t14 * 7.96e-2 - t12 * t13 * t14 * t15 * (2.0 / 5.0),
                        t14 * (sin(t16) * 5.995e+2 - sin(t24) * (8.01e+2 / 2.0)) * (-4.0e-4),
                        t7 * t8 * 7.96e-2, 0.0, 0.0,
                        t7 * t36 * (2.0 / 5.0) - t7 * t13 * t22 * (2.0 / 5.0) + t5 * t7 * t36 * (21.0 / 50.0) + t7 * t13 * t31 * (2.0 / 5.0) + t4 * t7 * t10 * t13 * (21.0 / 50.0) + t10 * t11 * t12 * t14 * (21.0 / 50.0),
                        t4 * t6 * t7 * (2.0 / 5.0) + t4 * t12 * t14 * (21.0 / 50.0) - t7 * t11 * t13 * (21.0 / 50.0) + t4 * t5 * t6 * t7 * (21.0 / 50.0) - t5 * t7 * t11 * t13 * (2.0 / 5.0),
                        t7 * t12 * t13 * (-2.0 / 5.0),
                        t6 * t7 * (-2.0 / 5.0),
                        0.0, 0.0, 0.0, -t8 * t86, -t8 * t71,
                        -t3 * t8 * t86 - t8 * t9 * t10 * (t92 + t7 * (t34 - t56)) - t2 * t8 * t10 * t102,
                        t8 * t50 * (t92 + t7 * (t34 - t56)) - t8 * t48 * t102 + t8 * t10 * t11 * t86,
                        -t8 * t49 * t86 - t8 * t72 * t102 - t8 * (t92 + t7 * (t34 - t56)) * (t34 - t56),
                        t8 * t102 * (t54 + t13 * (t33 - t59)) - t8 * (t92 + t7 * (t34 - t56)) * (t60 - t74) - t8 * t86 * (t36 - t65),
                        -t8 * t90 - t8 * t101 - t8 * t103 - (t15 * (t36 - t65) - t8 * (t58 - t80)) * (t58 - t80) + t100 * (t15 * (t54 + t13 * (t33 - t59)) + t8 * t100) + t98 * (t8 * t98 + t15 * (t60 - t74)),
                        t15 * t86, t15 * t71, t3 * t15 * t86 + t9 * t10 * t15 * (t92 + t7 * (t34 - t56)) + t2 * t10 * t15 * t102,
                        -t15 * t50 * (t92 + t7 * (t34 - t56)) + t15 * t48 * t102 - t10 * t11 * t15 * t86,
                        t15 * t49 * t86 + t15 * t72 * t102 + t15 * (t92 + t7 * (t34 - t56)) * (t34 - t56),
                        -t15 * t102 * (t54 + t13 * (t33 - t59)) + t15 * (t92 + t7 * (t34 - t56)) * (t60 - t74) + t15 * t86 * (t36 - t65),
                        t15 * t90 + t15 * t101 + t15 * t103 - (t8 * (t36 - t65) + t15 * (t58 - t80)) * (t58 - t80) + t100 * (t8 * (t54 + t13 * (t33 - t59)) - t15 * t100) - t98 * (t15 * t98 - t8 * (t60 - t74)),
                        t89, t4 * t7 * t13 + t11 * t12 * t14 + t5 * t6 * t7 * t11,
                        -t5 * t14 + t6 * t7 * t12,
                        -t48 * t100 - t50 * t98 + t10 * t11 * (t58 - t80),
                        -t72 * t100 - t49 * (t58 - t80) + t98 * (t34 - t56),
                        t100 * (t54 + t13 * (t33 - t59)) + t98 * (t60 - t74) - (t36 - t65) * (t58 - t80),
                        t98 * (t92 + t7 * (t34 - t56)) * 2.0 - t100 * t102 * 2.0 - t86 * (t58 - t80) * 2.0;

    return dJ_T_dq6_mat.transpose( );
}

Eigen::MatrixXd dJB_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
 
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = cos(q7);
    double t9 = sin(q1);
    double t10 = sin(q2);
    double t11 = sin(q3);
    double t12 = sin(q4);
    double t13 = sin(q5);
    double t14 = sin(q6);
    double t15 = sin(q7);
    double t16 = t2 * t4;
    double t17 = t3 * t5;
    double t18 = t2 * t11;
    double t19 = t4 * t9;
    double t20 = t3 * t12;
    double t21 = t9 * t11;
    double t22 = t9 * t10 * t12;
    double t23 = t10 * t11 * t13;
    double t27 = t2 * t5 * t10;
    double t28 = t4 * t5 * t10;
    double t30 = t2 * t10 * t12;
    double t31 = t5 * t9 * t10;
    double t32 = t4 * t10 * t12;
    double t33 = t6 * t10 * t11;
    double t34 = t3 * (9.0 / 25.0);
    double t35 = t5 * (39.0 / 50.0);
    double t36 = t7 * (59.0 / 50.0);
    double t43 = t2 * t10 * (9.0 / 25.0);
    double t44 = t9 * t10 * (9.0 / 25.0);
    double t24 = t3 * t16;
    double t25 = t3 * t18;
    double t26 = t3 * t19;
    double t29 = t3 * t21;
    double t38 = -t28;
    double t45 = t23 * (59.0 / 50.0);
    double t46 = t32 * (39.0 / 50.0);
    double t50 = t17 + t32;
    double t54 = t35 - 39.0 / 50.0;
    double t55 = t36 - 59.0 / 50.0;
    double t37 = -t24;
    double t39 = -t29;
    double t48 = t18 + t26;
    double t49 = t19 + t25;
    double t53 = t20 + t38;
    double t58 = t7 * t50;
    double t61 = t14 * t50;
    double t70 = t3 * t54;
    double t72 = t2 * t10 * t54;
    double t73 = t9 * t10 * t54;
    double t97 = t50 * t55;
    double t51 = t16 + t39;
    double t52 = t21 + t37;
    double t56 = t5 * t48;
    double t57 = t6 * t49;
    double t59 = t12 * t48;
    double t60 = t13 * t49;
    double t65 = t6 * t53;
    double t68 = t13 * t53;
    double t82 = t58 * 1.2596;
    double t112 = t34 + t46 + t70 - 9.0 / 25.0;
    double t62 = t5 * t52;
    double t63 = t6 * t51;
    double t66 = t12 * t52;
    double t67 = t13 * t51;
    double t74 = t59 * (39.0 / 50.0);
    double t75 = t60 * (59.0 / 50.0);
    double t77 = t65 * (59.0 / 50.0);
    double t80 = t22 + t56;
    double t84 = t23 + t65;
    double t85 = -t82;
    double t95 = -t14 * (t31 - t59);
    double t101 = -t15 * (t33 - t68);
    double t106 = t7 * (t31 - t59) * (-1.2596);
    double t111 = t55 * (t31 - t59);
    double t76 = -t74;
    double t78 = t66 * (39.0 / 50.0);
    double t79 = t67 * (59.0 / 50.0);
    double t86 = t27 + t66;
    double t87 = t6 * t80;
    double t88 = t13 * t80;
    double t94 = t7 * t84;
    double t96 = t14 * t84;
    double t98 = -t6 * (t30 - t62);
    double t103 = t6 * (t30 - t62) * (-59.0 / 50.0);
    double t128 = -t8 * (t57 + t13 * (t30 - t62));
    double t135 = t15 * (t57 + t13 * (t30 - t62));
    double t152 = -t112 * (t57 + t13 * (t30 - t62));
    double t91 = t7 * t86;
    double t93 = t14 * t86;
    double t102 = t87 * (59.0 / 50.0);
    double t104 = t96 * 7.96e-2;
    double t109 = t55 * t86;
    double t113 = t67 + t87;
    double t114 = t58 + t96;
    double t115 = t60 + t98;
    double t123 = -t8 * (t63 - t88);
    double t124 = -t8 * (t61 - t94);
    double t131 = t8 * (t61 - t94);
    double t132 = t15 * (t63 - t88);
    double t133 = t44 + t73 + t76;
    double t134 = t43 + t72 + t78;
    double t153 = t112 * (t63 - t88);
    double t105 = t91 * 1.2596;
    double t107 = -t104;
    double t118 = t7 * t113;
    double t119 = t14 * t113;
    double t121 = t7 * t115;
    double t122 = t14 * t115;
    double t142 = -t133 * (t33 - t68);
    double t146 = -t134 * (t33 - t68);
    double t150 = t101 + t131;
    double t108 = -t105;
    double t129 = -t122;
    double t136 = t119 * 7.96e-2;
    double t138 = t122 * 7.96e-2;
    double t139 = t95 + t118;
    double t141 = t93 + t121;
    double t159 = t85 + t97 + t107 + t112;
    double t163 = t79 + t102 + t146 + t152;
    double t164 = t75 + t103 + t142 + t153;
    double t137 = -t136;
    double t143 = t8 * t139;
    double t144 = t15 * t139;
    double t147 = t91 + t129;
    double t148 = t8 * t141;
    double t149 = t15 * t141;
    double t162 = t108 + t109 + t134 + t138;
    double t156 = t123 + t144;
    double t157 = t132 + t143;
    double t158 = t128 + t149;
    double t160 = t135 + t148;
    double t161 = t106 + t111 + t133 + t137;
    double t166 = t156 * t162;
    double t167 = t157 * t162;
    double t168 = t158 * t161;
    double t169 = t160 * t161;
    double t170 = t166 + t168;
    double t171 = t167 + t169;

    Eigen::MatrixXd dJ_T_dq7_mat( 6, 7 );
    dJ_T_dq7_mat << t170, t4 * t6 * t15 * (-7.96e-2) - t4 * t8 * t13 * (2.0 / 5.0) - t6 * t8 * t11 * (21.0 / 50.0) - t4 * t5 * t8 * t13 * (21.0 / 50.0) - t5 * t6 * t8 * t11 * (2.0 / 5.0) - t4 * t6 * t7 * t15 * (2.0 / 5.0) - t4 * t7 * t8 * t13 * 7.96e-2 + t5 * t11 * t13 * t15 * 7.96e-2 - t4 * t12 * t14 * t15 * (21.0 / 50.0) - t8 * t11 * t12 * t14 * 7.96e-2 + t7 * t11 * t13 * t15 * (21.0 / 50.0) - t4 * t5 * t6 * t7 * t15 * (21.0 / 50.0) - t5 * t6 * t7 * t8 * t11 * 7.96e-2 + t5 * t7 * t11 * t13 * t15 * (2.0 / 5.0),
                        t6 * t8 * t12 * (-2.0 / 5.0) + t5 * t8 * t14 * 7.96e-2 + t12 * t13 * t15 * 7.96e-2 - t6 * t7 * t8 * t12 * 7.96e-2 + t7 * t12 * t13 * t15 * (2.0 / 5.0),
                        t6 * t15 * 7.96e-2 + t8 * t13 * (2.0 / 5.0) + t6 * t7 * t15 * (2.0 / 5.0) + t7 * t8 * t13 * 7.96e-2,
                        t8 * t14 * 7.96e-2, t156 * t163 + t158 * t164 + t170 * (t33 - t68) + (t57 + t13 * (t30 - t62)) * (t161 * (t8 * (t33 - t68) + t15 * (t61 - t94)) + t156 * t159) - (t8 * (t33 - t68) + t15 * (t61 - t94)) * (t45 + t77 + t133 * (t57 + t13 * (t30 - t62)) + t134 * (t63 - t88)) + (t63 - t88) * (t162 * (t8 * (t33 - t68) + t15 * (t61 - t94)) - t158 * t159),
                        0.0, t171, t4 * t6 * t8 * (-7.96e-2) + t4 * t13 * t15 * (2.0 / 5.0) + t6 * t11 * t15 * (21.0 / 50.0) - t4 * t6 * t7 * t8 * (2.0 / 5.0) + t4 * t5 * t13 * t15 * (21.0 / 50.0) + t5 * t6 * t11 * t15 * (2.0 / 5.0) + t5 * t8 * t11 * t13 * 7.96e-2 - t4 * t8 * t12 * t14 * (21.0 / 50.0) + t4 * t7 * t13 * t15 * 7.96e-2 + t7 * t8 * t11 * t13 * (21.0 / 50.0) + t11 * t12 * t14 * t15 * 7.96e-2 - t4 * t5 * t6 * t7 * t8 * (21.0 / 50.0) + t5 * t6 * t7 * t11 * t15 * 7.96e-2 + t5 * t7 * t8 * t11 * t13 * (2.0 / 5.0),
                        t6 * t12 * t15 * (2.0 / 5.0) + t8 * t12 * t13 * 7.96e-2 - t5 * t14 * t15 * 7.96e-2 + t6 * t7 * t12 * t15 * 7.96e-2 + t7 * t8 * t12 * t13 * (2.0 / 5.0),
                        t6 * t8 * 7.96e-2 - t13 * t15 * (2.0 / 5.0) + t6 * t7 * t8 * (2.0 / 5.0) - t7 * t13 * t15 * 7.96e-2,
                        t14 * t15 * (-7.96e-2), t157 * t163 + t160 * t164 - (t57 + t13 * (t30 - t62)) * (t161 * (t124 + t15 * (t33 - t68)) - t157 * t159) + t171 * (t33 - t68) + (t124 + t15 * (t33 - t68)) * (t45 + t77 + t133 * (t57 + t13 * (t30 - t62)) + t134 * (t63 - t88)) - (t63 - t88) * (t162 * (t124 + t15 * (t33 - t68)) + t159 * t160),
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t8 * (t33 - t68) + t15 * (t61 - t94),
                        t4 * t6 * t8 - t5 * t8 * t11 * t13 - t4 * t7 * t13 * t15 - t11 * t12 * t14 * t15 - t5 * t6 * t7 * t11 * t15,
                        t3 * (t8 * (t33 - t68) + t15 * (t61 - t94)) + t2 * t10 * t158 - t9 * t10 * t156,
                        t49 * t158 + t51 * t156 - t10 * t11 * (t8 * (t33 - t68) + t15 * (t61 - t94)),
                        t50 * (t8 * (t33 - t68) + t15 * (t61 - t94)) + t86 * t158 - t156 * (t31 - t59),
                        -t158 * (t57 + t13 * (t30 - t62)) + (t8 * (t33 - t68) + t15 * (t61 - t94)) * (t33 - t68) - t156 * (t63 - t88),
                        -t156 * (t119 + t7 * (t31 - t59)) + t114 * (t8 * (t33 - t68) + t15 * (t61 - t94)) + t147 * t158,
                        t150, -t4 * t6 * t15 - t4 * t7 * t8 * t13 + t5 * t11 * t13 * t15 - t8 * t11 * t12 * t14 - t5 * t6 * t7 * t8 * t11,
                        -t3 * (t124 + t15 * (t33 - t68)) + t2 * t10 * t160 - t9 * t10 * t157,
                        t51 * t157 + t49 * t160 + t10 * t11 * (t124 + t15 * (t33 - t68)),
                        -t50 * (t124 + t15 * (t33 - t68)) + t86 * t160 - t157 * (t31 - t59),
                        -t160 * (t57 + t13 * (t30 - t62)) - (t124 + t15 * (t33 - t68)) * (t33 - t68) - t157 * (t63 - t88),
                        -t114 * (t124 + t15 * (t33 - t68)) - t157 * (t119 + t7 * (t31 - t59)) + t147 * t160,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return dJ_T_dq7_mat.transpose( );
}
