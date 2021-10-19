//
// Created by yishenjin on 8/3/21.
//

#include "MainVD.h"
#include <math.h>

MainVD::MainVD()
{
}

MainVD::~MainVD()
{
}




void MainVD::VD_filter(double sumo_X_nex,
                       double sumo_Y_nex,
                       double psi_ref,
                       double *psi_,
                       double *ue_X_,
                       double *ue_Y_,
                       double *ue_V_) {


    init_limits(V_pivot, max_dss);
    double mid_X = 0, mid_Y = 0;
    double x[4] = { 0,0,0,0 };
    double Ax[4] = { 0,0,0,0 };
    double R = MYINF, dirn = 0;
    double d_ss = 0, dss_lim = 0;
    double v_global_x = 0, v_global_y = 0;
    double A[16] = {}, B[4] = {};



    double psi = *psi_, ue_X = *ue_X_, ue_Y = *ue_Y_, ue_V=*ue_V_;

    // Simulation parameters

    int n_pivot = 36;


    transform(&mid_X, &mid_Y, psi, 5 * ue_V*ue_dT, 0, ue_X, ue_Y);
    CircRad(&R, ue_X, ue_Y, mid_X, mid_Y, sumo_X_nex, sumo_Y_nex);


    dirn = (mid_X - ue_X)*(sumo_Y_nex - ue_Y) - (mid_Y - ue_Y)*(sumo_X_nex - ue_X);


    if (ue_V > 2) {
        get_discrete_model(A, B, &d_ss, ue_V, ue_dT, R);

        if (d_ss != 0) {
            // std::cout << "dss = " << d_ss << std::endl;
        }
        if (interp1(&dss_lim, V_pivot, max_dss, n_pivot, ue_V) == -1) {
            dss_lim = 1;
        }

        if (d_ss > dss_lim) {
            d_ss = dss_lim;
        }

        if (dirn < 0) {
            d_ss = -d_ss;
        }

        // Propagate linear model by one step x_next = Ax+Bd_ss
        mat_prod(Ax, A, x, 4);
        for (int i = 0; i < 4; i++) {
            x[i] = Ax[i] + B[i] * d_ss;
        }

    }
    else {
        x[2] = atan2(sumo_Y_nex - ue_Y, sumo_X_nex - ue_X);
        d_ss = 0;
    }

    transform(&v_global_x, &v_global_y, x[2], ue_V, x[1], 0, 0);
    ue_X = ue_X + v_global_x * ue_dT;
    ue_Y = ue_Y + v_global_y * ue_dT;

    if (abs(ue_X - sumo_X_nex) < 0.01 && abs(psi_ref - psi) < 0.01) {
        ue_X = sumo_X_nex;
        x[0] = 0; x[1] = 0; x[2] = psi_ref; x[3] = 0;
    }

    if (abs(ue_Y - sumo_Y_nex) < 0.01 && abs(psi_ref - psi) < 0.01) {
        ue_Y = sumo_Y_nex;
        x[0] = 0; x[1] = 0; x[2] = psi_ref; x[3] = 0;
    }

    psi = x[2];

    //ue_V_[0] = ue_V;
    ue_X_[0] = ue_X;
    ue_Y_[0] = ue_Y;
    psi_[0] = psi;
}
void MainVD::init_limits(double *x, double *y) {
    int nx = 36;

    for (int i = 0; i < nx; i++) {
        x[i] = i;
        if (i < 15) {
            y[i] = 1.2;
        }
        else {
            y[i] = 1;
        }

    }
}
void MainVD::get_discrete_model(double *A, double *B, double *d_ss, double V_x, double dT, double R) {
    if (V_x == 0) {
        A[0] = 1; A[4] = 0; A[8] = 0; A[12] = 0;
        A[1] = 0; A[5] = 1; A[9] = 0; A[13] = 0;
        A[2] = 0; A[6] = 0; A[10] = 1; A[14] = 0;
        A[3] = 0; A[7] = 0; A[11] = 0; A[15] = 1;

        B[0] = 0;
        B[1] = 0;
        B[2] = 0;
        B[3] = 0;

        d_ss[0] = 0;
    }
    else {
        A[0] = 1; A[4] = 1 * dT;															A[8] = 0;  A[12] = 0;
        A[1] = 0; A[5] = 1 - dT * (2 * C_alpha_f + 2 * C_alpha_r) / (m*V_x);				A[9] = 0;  A[13] = -dT * V_x - dT * (2 * C_alpha_f*l_f - 2 * C_alpha_r*l_r) / (m*V_x);
        A[2] = 0; A[6] = 0;																A[10] = 1; A[14] = dT * 1;
        A[3] = 0; A[7] = -dT * (2 * C_alpha_f*l_f - 2 * C_alpha_r*l_r) / (I_z*V_x);		A[11] = 0; A[15] = 1 - dT * (2 * C_alpha_f* pow(l_f, 2) + 2 * C_alpha_r*pow(l_r, 2)) / (I_z*V_x);

        B[0] = 0;
        B[1] = dT * 2 * C_alpha_f / m;
        B[2] = 0;
        B[3] = dT * 2 * l_f*C_alpha_f / I_z;

        d_ss[0] = (l_f + l_r) / R + ((m*l_r*C_alpha_r - m * l_f*C_alpha_f) / (2 * C_alpha_f*C_alpha_r*(l_f + l_r)))*pow(V_x, 2) / R;
    }
    if (R == MYINF) {
        d_ss[0] = 0;
    }

}
// performs [x1;y1] = RotMat(psi)*[x_l; yl] + [x_0; y_0]
void MainVD::transform(double *x_1, double *y_1, double psi, double x_l, double y_l, double x_0, double y_0) {

    double output[2] = {};
    double RotMat[4] = {};
    double RotVec[2] = {};
    double RotRes_temp[2] = {};

    RotMat[0] = cos(psi);
    RotMat[1] = sin(psi);
    RotMat[2] = -sin(psi);
    RotMat[3] = cos(psi);
    RotVec[0] = x_l;
    RotVec[1] = y_l;

    mat_prod(RotRes_temp, RotMat, RotVec, 2);
    x_1[0] = RotRes_temp[0] + x_0;
    y_1[0] = RotRes_temp[1] + y_0;
}


// Multiply matrix M of dimension nXn with a vector V of dimention n. output = M*V (* is matrix mult)
void MainVD::mat_prod(double *output, double *M, double *V, int n) {
    double temp_sum = 0;

    for (int i = 0; i < n; i++) {
        temp_sum = 0;
        for (int j = 0; j < n; j++) {
            temp_sum += M[j*n + i] * V[j];
        }
        output[i] = temp_sum;
    }

}



void MainVD::CircRad(double *R, double x1, double y1, double x2, double y2, double x3, double y3) {
    double co = 0;
    if (cond(&co, x2 - x1, x3 - x1, y2 - y1, y3 - y1) == 0) {
        R[0] = MYINF;
    }
    if (co >= 1e12) {
        R[0] = MYINF;
    }
    else {
        computeR(R, x1, y1, x2, y2, x3, y3);
    }
    if (isnan(R[0])) {
        R[0] = MYINF;
        //std::cout << R[0] << std::endl;
    }
}

// Interpolation
// It is not assumed that x has equidistant steps
// Output is return
int MainVD::interp1(double * yp, double *x, double *y, int nx, double xp) {

    if ((xp < x[0]) || (xp > x[nx - 1])) {
        *yp = 1;
        return -1;
    }
    else {
        for (int ii = 1; ii < nx; ii++) {
            if (xp <= x[ii]) {
                *yp = y[ii - 1] + (y[ii] - y[ii - 1])*(xp - x[ii - 1]) / (x[ii] - x[ii - 1]);
                return 1;
            }
        }
    }
    return -1;
}

int MainVD::cond(double *co, double a, double b, double c, double d) {
    double ai = 0, bi = 0, ci = 0, di = 0;
    double s = 0, si = 0;

    if (a*d - b * c == 0) {
        return 0;
    }
    svd(&s, a, b, c, d);
    ai = d / (a*d - b * c);
    bi = -b / (a*d - b * c);
    ci = -c / (a*d - b * c);
    di = a / (a*d - b * c);

    svd(&si, ai, bi, ci, di);

    co[0] = s * si;
    return 1;
}

// Compute radius of curvature using 3-non linear points
void MainVD::computeR(double *R, double x1, double y1, double x2, double y2, double x3, double y3) {
    R[0] = sqrt((pow((x1*y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2), 2) * (pow(x1, 2) - 2 * x1*x2 + pow(x2, 2) + pow(y1, 2) - 2 * y1*y2 + pow(y2, 2))*(pow(x1, 2) - 2 * x1*x3 + pow(x3, 2) + pow(y1, 2) - 2 * y1*y3 + pow(y3, 2))*(pow(x2, 2) - 2 * x2*x3 + pow(x3, 2) + pow(y2, 2) - 2 * y2*y3 + pow(y3, 2)))) / (2 * pow(x1*y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2, 2));
}



void MainVD::svd(double *s, double a, double b, double c, double d) {
    double e1 = 0, e2 = 0;

    e1 = pow(a, 2) / 2 - sqrt((pow(a, 2) - 2 * a*d + pow(b, 2) + 2 * b*c + pow(c, 2) + pow(d, 2))*(pow(a, 2) + 2 * a*d + pow(b, 2) - 2 * b*c + pow(c, 2) + pow(d, 2))) / 2 + pow(b, 2) / 2 + pow(c, 2) / 2 + pow(d, 2) / 2;

    e2 = sqrt((pow(a, 2) - 2 * a*d + pow(b, 2) + 2 * b*c + pow(c, 2) + pow(d, 2))*(pow(a, 2) + 2 * a*d + pow(b, 2) - 2 * b*c + pow(c, 2) + pow(d, 2))) / 2 + pow(a, 2) / 2 + pow(b, 2) / 2 + pow(c, 2) / 2 + pow(d, 2) / 2;

    s[0] = e1;
    if (e1 < e2) {
        s[0] = e2;
    }

}