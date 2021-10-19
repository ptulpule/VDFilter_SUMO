//
// Created by yishenjin on 8/3/21.
//

#ifndef VDFILTER_MAINVD_H
#define VDFILTER_MAINVD_H


#define UE_dT		100				// 20 steps per 1 sumo step
#define SUMO_dT		0.1				// SUMO step size [s]
#define ue_dT		SUMO_dT/UE_dT	// UE Tick rate (may be variable when implemented)
#define C_alpha_f	100e3			// Front Tire cornering stiffness
#define C_alpha_r   110e3			// Rear Tire cornering stiffness
#define l_f			1.4				// Distance of CG from front axle [m]
#define l_r			1.6				// Distance of CG from rear axle [m]
#define I_z			2873			// vehicle moment of inertia [N/m^2]
#define m			1573			// vehicle mass [Kg]
#define MYINF		1e12			// Use this as replacement for inf

class MainVD {
public:
    MainVD();
    ~MainVD();

private:
    // double x[4] = { 0,0,0,0 };
    double max_dss[36] = {};
    double V_pivot[36] = {};
    void init_limits(double *x, double *y);
    void mat_prod(double *, double *, double *, int);
    void transform(double *, double *, double, double, double, double, double);  // rotation + translation operation: transform(x_1, y_1, psi,x_l, y_l,x_0,y_0)
    void computeR(double *, double, double, double, double, double, double);
    void CircRad(double *, double, double, double, double, double, double);
    void get_discrete_model(double *, double *, double *, double, double, double);
    void svd(double *, double, double, double, double);
    int cond(double *, double, double, double, double);
    int interp1(double *, double *, double *, int, double);

public:
    // x: state vector
    void VD_filter(double sumo_X_nex,
                   double sumo_Y_nex,
                   double psi_ref,
                   double *psi_,
                   double *ue_X_,
                   double *ue_Y_,
                   double *ue_V_);
};


#endif //VDFILTER_MAINVD_H
