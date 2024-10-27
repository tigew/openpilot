#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7995959195871035911) {
   out_7995959195871035911[0] = delta_x[0] + nom_x[0];
   out_7995959195871035911[1] = delta_x[1] + nom_x[1];
   out_7995959195871035911[2] = delta_x[2] + nom_x[2];
   out_7995959195871035911[3] = delta_x[3] + nom_x[3];
   out_7995959195871035911[4] = delta_x[4] + nom_x[4];
   out_7995959195871035911[5] = delta_x[5] + nom_x[5];
   out_7995959195871035911[6] = delta_x[6] + nom_x[6];
   out_7995959195871035911[7] = delta_x[7] + nom_x[7];
   out_7995959195871035911[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_687147536025638074) {
   out_687147536025638074[0] = -nom_x[0] + true_x[0];
   out_687147536025638074[1] = -nom_x[1] + true_x[1];
   out_687147536025638074[2] = -nom_x[2] + true_x[2];
   out_687147536025638074[3] = -nom_x[3] + true_x[3];
   out_687147536025638074[4] = -nom_x[4] + true_x[4];
   out_687147536025638074[5] = -nom_x[5] + true_x[5];
   out_687147536025638074[6] = -nom_x[6] + true_x[6];
   out_687147536025638074[7] = -nom_x[7] + true_x[7];
   out_687147536025638074[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5890016485206373197) {
   out_5890016485206373197[0] = 1.0;
   out_5890016485206373197[1] = 0;
   out_5890016485206373197[2] = 0;
   out_5890016485206373197[3] = 0;
   out_5890016485206373197[4] = 0;
   out_5890016485206373197[5] = 0;
   out_5890016485206373197[6] = 0;
   out_5890016485206373197[7] = 0;
   out_5890016485206373197[8] = 0;
   out_5890016485206373197[9] = 0;
   out_5890016485206373197[10] = 1.0;
   out_5890016485206373197[11] = 0;
   out_5890016485206373197[12] = 0;
   out_5890016485206373197[13] = 0;
   out_5890016485206373197[14] = 0;
   out_5890016485206373197[15] = 0;
   out_5890016485206373197[16] = 0;
   out_5890016485206373197[17] = 0;
   out_5890016485206373197[18] = 0;
   out_5890016485206373197[19] = 0;
   out_5890016485206373197[20] = 1.0;
   out_5890016485206373197[21] = 0;
   out_5890016485206373197[22] = 0;
   out_5890016485206373197[23] = 0;
   out_5890016485206373197[24] = 0;
   out_5890016485206373197[25] = 0;
   out_5890016485206373197[26] = 0;
   out_5890016485206373197[27] = 0;
   out_5890016485206373197[28] = 0;
   out_5890016485206373197[29] = 0;
   out_5890016485206373197[30] = 1.0;
   out_5890016485206373197[31] = 0;
   out_5890016485206373197[32] = 0;
   out_5890016485206373197[33] = 0;
   out_5890016485206373197[34] = 0;
   out_5890016485206373197[35] = 0;
   out_5890016485206373197[36] = 0;
   out_5890016485206373197[37] = 0;
   out_5890016485206373197[38] = 0;
   out_5890016485206373197[39] = 0;
   out_5890016485206373197[40] = 1.0;
   out_5890016485206373197[41] = 0;
   out_5890016485206373197[42] = 0;
   out_5890016485206373197[43] = 0;
   out_5890016485206373197[44] = 0;
   out_5890016485206373197[45] = 0;
   out_5890016485206373197[46] = 0;
   out_5890016485206373197[47] = 0;
   out_5890016485206373197[48] = 0;
   out_5890016485206373197[49] = 0;
   out_5890016485206373197[50] = 1.0;
   out_5890016485206373197[51] = 0;
   out_5890016485206373197[52] = 0;
   out_5890016485206373197[53] = 0;
   out_5890016485206373197[54] = 0;
   out_5890016485206373197[55] = 0;
   out_5890016485206373197[56] = 0;
   out_5890016485206373197[57] = 0;
   out_5890016485206373197[58] = 0;
   out_5890016485206373197[59] = 0;
   out_5890016485206373197[60] = 1.0;
   out_5890016485206373197[61] = 0;
   out_5890016485206373197[62] = 0;
   out_5890016485206373197[63] = 0;
   out_5890016485206373197[64] = 0;
   out_5890016485206373197[65] = 0;
   out_5890016485206373197[66] = 0;
   out_5890016485206373197[67] = 0;
   out_5890016485206373197[68] = 0;
   out_5890016485206373197[69] = 0;
   out_5890016485206373197[70] = 1.0;
   out_5890016485206373197[71] = 0;
   out_5890016485206373197[72] = 0;
   out_5890016485206373197[73] = 0;
   out_5890016485206373197[74] = 0;
   out_5890016485206373197[75] = 0;
   out_5890016485206373197[76] = 0;
   out_5890016485206373197[77] = 0;
   out_5890016485206373197[78] = 0;
   out_5890016485206373197[79] = 0;
   out_5890016485206373197[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3095575923665402692) {
   out_3095575923665402692[0] = state[0];
   out_3095575923665402692[1] = state[1];
   out_3095575923665402692[2] = state[2];
   out_3095575923665402692[3] = state[3];
   out_3095575923665402692[4] = state[4];
   out_3095575923665402692[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3095575923665402692[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3095575923665402692[7] = state[7];
   out_3095575923665402692[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6769955877553601173) {
   out_6769955877553601173[0] = 1;
   out_6769955877553601173[1] = 0;
   out_6769955877553601173[2] = 0;
   out_6769955877553601173[3] = 0;
   out_6769955877553601173[4] = 0;
   out_6769955877553601173[5] = 0;
   out_6769955877553601173[6] = 0;
   out_6769955877553601173[7] = 0;
   out_6769955877553601173[8] = 0;
   out_6769955877553601173[9] = 0;
   out_6769955877553601173[10] = 1;
   out_6769955877553601173[11] = 0;
   out_6769955877553601173[12] = 0;
   out_6769955877553601173[13] = 0;
   out_6769955877553601173[14] = 0;
   out_6769955877553601173[15] = 0;
   out_6769955877553601173[16] = 0;
   out_6769955877553601173[17] = 0;
   out_6769955877553601173[18] = 0;
   out_6769955877553601173[19] = 0;
   out_6769955877553601173[20] = 1;
   out_6769955877553601173[21] = 0;
   out_6769955877553601173[22] = 0;
   out_6769955877553601173[23] = 0;
   out_6769955877553601173[24] = 0;
   out_6769955877553601173[25] = 0;
   out_6769955877553601173[26] = 0;
   out_6769955877553601173[27] = 0;
   out_6769955877553601173[28] = 0;
   out_6769955877553601173[29] = 0;
   out_6769955877553601173[30] = 1;
   out_6769955877553601173[31] = 0;
   out_6769955877553601173[32] = 0;
   out_6769955877553601173[33] = 0;
   out_6769955877553601173[34] = 0;
   out_6769955877553601173[35] = 0;
   out_6769955877553601173[36] = 0;
   out_6769955877553601173[37] = 0;
   out_6769955877553601173[38] = 0;
   out_6769955877553601173[39] = 0;
   out_6769955877553601173[40] = 1;
   out_6769955877553601173[41] = 0;
   out_6769955877553601173[42] = 0;
   out_6769955877553601173[43] = 0;
   out_6769955877553601173[44] = 0;
   out_6769955877553601173[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6769955877553601173[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6769955877553601173[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6769955877553601173[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6769955877553601173[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6769955877553601173[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6769955877553601173[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6769955877553601173[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6769955877553601173[53] = -9.8000000000000007*dt;
   out_6769955877553601173[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6769955877553601173[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6769955877553601173[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6769955877553601173[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6769955877553601173[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6769955877553601173[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6769955877553601173[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6769955877553601173[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6769955877553601173[62] = 0;
   out_6769955877553601173[63] = 0;
   out_6769955877553601173[64] = 0;
   out_6769955877553601173[65] = 0;
   out_6769955877553601173[66] = 0;
   out_6769955877553601173[67] = 0;
   out_6769955877553601173[68] = 0;
   out_6769955877553601173[69] = 0;
   out_6769955877553601173[70] = 1;
   out_6769955877553601173[71] = 0;
   out_6769955877553601173[72] = 0;
   out_6769955877553601173[73] = 0;
   out_6769955877553601173[74] = 0;
   out_6769955877553601173[75] = 0;
   out_6769955877553601173[76] = 0;
   out_6769955877553601173[77] = 0;
   out_6769955877553601173[78] = 0;
   out_6769955877553601173[79] = 0;
   out_6769955877553601173[80] = 1;
}
void h_25(double *state, double *unused, double *out_6148156412027611728) {
   out_6148156412027611728[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7741577761423055414) {
   out_7741577761423055414[0] = 0;
   out_7741577761423055414[1] = 0;
   out_7741577761423055414[2] = 0;
   out_7741577761423055414[3] = 0;
   out_7741577761423055414[4] = 0;
   out_7741577761423055414[5] = 0;
   out_7741577761423055414[6] = 1;
   out_7741577761423055414[7] = 0;
   out_7741577761423055414[8] = 0;
}
void h_24(double *state, double *unused, double *out_4810852230657871105) {
   out_4810852230657871105[0] = state[4];
   out_4810852230657871105[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1614357259261373217) {
   out_1614357259261373217[0] = 0;
   out_1614357259261373217[1] = 0;
   out_1614357259261373217[2] = 0;
   out_1614357259261373217[3] = 0;
   out_1614357259261373217[4] = 1;
   out_1614357259261373217[5] = 0;
   out_1614357259261373217[6] = 0;
   out_1614357259261373217[7] = 0;
   out_1614357259261373217[8] = 0;
   out_1614357259261373217[9] = 0;
   out_1614357259261373217[10] = 0;
   out_1614357259261373217[11] = 0;
   out_1614357259261373217[12] = 0;
   out_1614357259261373217[13] = 0;
   out_1614357259261373217[14] = 1;
   out_1614357259261373217[15] = 0;
   out_1614357259261373217[16] = 0;
   out_1614357259261373217[17] = 0;
}
void h_30(double *state, double *unused, double *out_5527752435331588952) {
   out_5527752435331588952[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8186833353779247575) {
   out_8186833353779247575[0] = 0;
   out_8186833353779247575[1] = 0;
   out_8186833353779247575[2] = 0;
   out_8186833353779247575[3] = 0;
   out_8186833353779247575[4] = 1;
   out_8186833353779247575[5] = 0;
   out_8186833353779247575[6] = 0;
   out_8186833353779247575[7] = 0;
   out_8186833353779247575[8] = 0;
}
void h_26(double *state, double *unused, double *out_6936067799071820911) {
   out_6936067799071820911[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4000074442548999190) {
   out_4000074442548999190[0] = 0;
   out_4000074442548999190[1] = 0;
   out_4000074442548999190[2] = 0;
   out_4000074442548999190[3] = 0;
   out_4000074442548999190[4] = 0;
   out_4000074442548999190[5] = 0;
   out_4000074442548999190[6] = 0;
   out_4000074442548999190[7] = 1;
   out_4000074442548999190[8] = 0;
}
void h_27(double *state, double *unused, double *out_1146483144366766083) {
   out_1146483144366766083[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8085147408129879130) {
   out_8085147408129879130[0] = 0;
   out_8085147408129879130[1] = 0;
   out_8085147408129879130[2] = 0;
   out_8085147408129879130[3] = 1;
   out_8085147408129879130[4] = 0;
   out_8085147408129879130[5] = 0;
   out_8085147408129879130[6] = 0;
   out_8085147408129879130[7] = 0;
   out_8085147408129879130[8] = 0;
}
void h_29(double *state, double *unused, double *out_4168124904322219124) {
   out_4168124904322219124[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7676602009464855391) {
   out_7676602009464855391[0] = 0;
   out_7676602009464855391[1] = 1;
   out_7676602009464855391[2] = 0;
   out_7676602009464855391[3] = 0;
   out_7676602009464855391[4] = 0;
   out_7676602009464855391[5] = 0;
   out_7676602009464855391[6] = 0;
   out_7676602009464855391[7] = 0;
   out_7676602009464855391[8] = 0;
}
void h_28(double *state, double *unused, double *out_1816387137477598522) {
   out_1816387137477598522[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5687743047175165651) {
   out_5687743047175165651[0] = 1;
   out_5687743047175165651[1] = 0;
   out_5687743047175165651[2] = 0;
   out_5687743047175165651[3] = 0;
   out_5687743047175165651[4] = 0;
   out_5687743047175165651[5] = 0;
   out_5687743047175165651[6] = 0;
   out_5687743047175165651[7] = 0;
   out_5687743047175165651[8] = 0;
}
void h_31(double *state, double *unused, double *out_6143140465841279759) {
   out_6143140465841279759[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3373866340315647714) {
   out_3373866340315647714[0] = 0;
   out_3373866340315647714[1] = 0;
   out_3373866340315647714[2] = 0;
   out_3373866340315647714[3] = 0;
   out_3373866340315647714[4] = 0;
   out_3373866340315647714[5] = 0;
   out_3373866340315647714[6] = 0;
   out_3373866340315647714[7] = 0;
   out_3373866340315647714[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7995959195871035911) {
  err_fun(nom_x, delta_x, out_7995959195871035911);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_687147536025638074) {
  inv_err_fun(nom_x, true_x, out_687147536025638074);
}
void car_H_mod_fun(double *state, double *out_5890016485206373197) {
  H_mod_fun(state, out_5890016485206373197);
}
void car_f_fun(double *state, double dt, double *out_3095575923665402692) {
  f_fun(state,  dt, out_3095575923665402692);
}
void car_F_fun(double *state, double dt, double *out_6769955877553601173) {
  F_fun(state,  dt, out_6769955877553601173);
}
void car_h_25(double *state, double *unused, double *out_6148156412027611728) {
  h_25(state, unused, out_6148156412027611728);
}
void car_H_25(double *state, double *unused, double *out_7741577761423055414) {
  H_25(state, unused, out_7741577761423055414);
}
void car_h_24(double *state, double *unused, double *out_4810852230657871105) {
  h_24(state, unused, out_4810852230657871105);
}
void car_H_24(double *state, double *unused, double *out_1614357259261373217) {
  H_24(state, unused, out_1614357259261373217);
}
void car_h_30(double *state, double *unused, double *out_5527752435331588952) {
  h_30(state, unused, out_5527752435331588952);
}
void car_H_30(double *state, double *unused, double *out_8186833353779247575) {
  H_30(state, unused, out_8186833353779247575);
}
void car_h_26(double *state, double *unused, double *out_6936067799071820911) {
  h_26(state, unused, out_6936067799071820911);
}
void car_H_26(double *state, double *unused, double *out_4000074442548999190) {
  H_26(state, unused, out_4000074442548999190);
}
void car_h_27(double *state, double *unused, double *out_1146483144366766083) {
  h_27(state, unused, out_1146483144366766083);
}
void car_H_27(double *state, double *unused, double *out_8085147408129879130) {
  H_27(state, unused, out_8085147408129879130);
}
void car_h_29(double *state, double *unused, double *out_4168124904322219124) {
  h_29(state, unused, out_4168124904322219124);
}
void car_H_29(double *state, double *unused, double *out_7676602009464855391) {
  H_29(state, unused, out_7676602009464855391);
}
void car_h_28(double *state, double *unused, double *out_1816387137477598522) {
  h_28(state, unused, out_1816387137477598522);
}
void car_H_28(double *state, double *unused, double *out_5687743047175165651) {
  H_28(state, unused, out_5687743047175165651);
}
void car_h_31(double *state, double *unused, double *out_6143140465841279759) {
  h_31(state, unused, out_6143140465841279759);
}
void car_H_31(double *state, double *unused, double *out_3373866340315647714) {
  H_31(state, unused, out_3373866340315647714);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
