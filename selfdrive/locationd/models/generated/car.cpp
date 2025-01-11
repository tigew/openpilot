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
void err_fun(double *nom_x, double *delta_x, double *out_4696672223706615105) {
   out_4696672223706615105[0] = delta_x[0] + nom_x[0];
   out_4696672223706615105[1] = delta_x[1] + nom_x[1];
   out_4696672223706615105[2] = delta_x[2] + nom_x[2];
   out_4696672223706615105[3] = delta_x[3] + nom_x[3];
   out_4696672223706615105[4] = delta_x[4] + nom_x[4];
   out_4696672223706615105[5] = delta_x[5] + nom_x[5];
   out_4696672223706615105[6] = delta_x[6] + nom_x[6];
   out_4696672223706615105[7] = delta_x[7] + nom_x[7];
   out_4696672223706615105[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5014476520979880068) {
   out_5014476520979880068[0] = -nom_x[0] + true_x[0];
   out_5014476520979880068[1] = -nom_x[1] + true_x[1];
   out_5014476520979880068[2] = -nom_x[2] + true_x[2];
   out_5014476520979880068[3] = -nom_x[3] + true_x[3];
   out_5014476520979880068[4] = -nom_x[4] + true_x[4];
   out_5014476520979880068[5] = -nom_x[5] + true_x[5];
   out_5014476520979880068[6] = -nom_x[6] + true_x[6];
   out_5014476520979880068[7] = -nom_x[7] + true_x[7];
   out_5014476520979880068[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4194163149970831217) {
   out_4194163149970831217[0] = 1.0;
   out_4194163149970831217[1] = 0;
   out_4194163149970831217[2] = 0;
   out_4194163149970831217[3] = 0;
   out_4194163149970831217[4] = 0;
   out_4194163149970831217[5] = 0;
   out_4194163149970831217[6] = 0;
   out_4194163149970831217[7] = 0;
   out_4194163149970831217[8] = 0;
   out_4194163149970831217[9] = 0;
   out_4194163149970831217[10] = 1.0;
   out_4194163149970831217[11] = 0;
   out_4194163149970831217[12] = 0;
   out_4194163149970831217[13] = 0;
   out_4194163149970831217[14] = 0;
   out_4194163149970831217[15] = 0;
   out_4194163149970831217[16] = 0;
   out_4194163149970831217[17] = 0;
   out_4194163149970831217[18] = 0;
   out_4194163149970831217[19] = 0;
   out_4194163149970831217[20] = 1.0;
   out_4194163149970831217[21] = 0;
   out_4194163149970831217[22] = 0;
   out_4194163149970831217[23] = 0;
   out_4194163149970831217[24] = 0;
   out_4194163149970831217[25] = 0;
   out_4194163149970831217[26] = 0;
   out_4194163149970831217[27] = 0;
   out_4194163149970831217[28] = 0;
   out_4194163149970831217[29] = 0;
   out_4194163149970831217[30] = 1.0;
   out_4194163149970831217[31] = 0;
   out_4194163149970831217[32] = 0;
   out_4194163149970831217[33] = 0;
   out_4194163149970831217[34] = 0;
   out_4194163149970831217[35] = 0;
   out_4194163149970831217[36] = 0;
   out_4194163149970831217[37] = 0;
   out_4194163149970831217[38] = 0;
   out_4194163149970831217[39] = 0;
   out_4194163149970831217[40] = 1.0;
   out_4194163149970831217[41] = 0;
   out_4194163149970831217[42] = 0;
   out_4194163149970831217[43] = 0;
   out_4194163149970831217[44] = 0;
   out_4194163149970831217[45] = 0;
   out_4194163149970831217[46] = 0;
   out_4194163149970831217[47] = 0;
   out_4194163149970831217[48] = 0;
   out_4194163149970831217[49] = 0;
   out_4194163149970831217[50] = 1.0;
   out_4194163149970831217[51] = 0;
   out_4194163149970831217[52] = 0;
   out_4194163149970831217[53] = 0;
   out_4194163149970831217[54] = 0;
   out_4194163149970831217[55] = 0;
   out_4194163149970831217[56] = 0;
   out_4194163149970831217[57] = 0;
   out_4194163149970831217[58] = 0;
   out_4194163149970831217[59] = 0;
   out_4194163149970831217[60] = 1.0;
   out_4194163149970831217[61] = 0;
   out_4194163149970831217[62] = 0;
   out_4194163149970831217[63] = 0;
   out_4194163149970831217[64] = 0;
   out_4194163149970831217[65] = 0;
   out_4194163149970831217[66] = 0;
   out_4194163149970831217[67] = 0;
   out_4194163149970831217[68] = 0;
   out_4194163149970831217[69] = 0;
   out_4194163149970831217[70] = 1.0;
   out_4194163149970831217[71] = 0;
   out_4194163149970831217[72] = 0;
   out_4194163149970831217[73] = 0;
   out_4194163149970831217[74] = 0;
   out_4194163149970831217[75] = 0;
   out_4194163149970831217[76] = 0;
   out_4194163149970831217[77] = 0;
   out_4194163149970831217[78] = 0;
   out_4194163149970831217[79] = 0;
   out_4194163149970831217[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8207092215987119012) {
   out_8207092215987119012[0] = state[0];
   out_8207092215987119012[1] = state[1];
   out_8207092215987119012[2] = state[2];
   out_8207092215987119012[3] = state[3];
   out_8207092215987119012[4] = state[4];
   out_8207092215987119012[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8207092215987119012[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8207092215987119012[7] = state[7];
   out_8207092215987119012[8] = state[8];
}
void F_fun(double *state, double dt, double *out_210187902771147012) {
   out_210187902771147012[0] = 1;
   out_210187902771147012[1] = 0;
   out_210187902771147012[2] = 0;
   out_210187902771147012[3] = 0;
   out_210187902771147012[4] = 0;
   out_210187902771147012[5] = 0;
   out_210187902771147012[6] = 0;
   out_210187902771147012[7] = 0;
   out_210187902771147012[8] = 0;
   out_210187902771147012[9] = 0;
   out_210187902771147012[10] = 1;
   out_210187902771147012[11] = 0;
   out_210187902771147012[12] = 0;
   out_210187902771147012[13] = 0;
   out_210187902771147012[14] = 0;
   out_210187902771147012[15] = 0;
   out_210187902771147012[16] = 0;
   out_210187902771147012[17] = 0;
   out_210187902771147012[18] = 0;
   out_210187902771147012[19] = 0;
   out_210187902771147012[20] = 1;
   out_210187902771147012[21] = 0;
   out_210187902771147012[22] = 0;
   out_210187902771147012[23] = 0;
   out_210187902771147012[24] = 0;
   out_210187902771147012[25] = 0;
   out_210187902771147012[26] = 0;
   out_210187902771147012[27] = 0;
   out_210187902771147012[28] = 0;
   out_210187902771147012[29] = 0;
   out_210187902771147012[30] = 1;
   out_210187902771147012[31] = 0;
   out_210187902771147012[32] = 0;
   out_210187902771147012[33] = 0;
   out_210187902771147012[34] = 0;
   out_210187902771147012[35] = 0;
   out_210187902771147012[36] = 0;
   out_210187902771147012[37] = 0;
   out_210187902771147012[38] = 0;
   out_210187902771147012[39] = 0;
   out_210187902771147012[40] = 1;
   out_210187902771147012[41] = 0;
   out_210187902771147012[42] = 0;
   out_210187902771147012[43] = 0;
   out_210187902771147012[44] = 0;
   out_210187902771147012[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_210187902771147012[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_210187902771147012[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_210187902771147012[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_210187902771147012[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_210187902771147012[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_210187902771147012[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_210187902771147012[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_210187902771147012[53] = -9.8000000000000007*dt;
   out_210187902771147012[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_210187902771147012[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_210187902771147012[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_210187902771147012[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_210187902771147012[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_210187902771147012[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_210187902771147012[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_210187902771147012[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_210187902771147012[62] = 0;
   out_210187902771147012[63] = 0;
   out_210187902771147012[64] = 0;
   out_210187902771147012[65] = 0;
   out_210187902771147012[66] = 0;
   out_210187902771147012[67] = 0;
   out_210187902771147012[68] = 0;
   out_210187902771147012[69] = 0;
   out_210187902771147012[70] = 1;
   out_210187902771147012[71] = 0;
   out_210187902771147012[72] = 0;
   out_210187902771147012[73] = 0;
   out_210187902771147012[74] = 0;
   out_210187902771147012[75] = 0;
   out_210187902771147012[76] = 0;
   out_210187902771147012[77] = 0;
   out_210187902771147012[78] = 0;
   out_210187902771147012[79] = 0;
   out_210187902771147012[80] = 1;
}
void h_25(double *state, double *unused, double *out_3690881903938120256) {
   out_3690881903938120256[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2244001790848573428) {
   out_2244001790848573428[0] = 0;
   out_2244001790848573428[1] = 0;
   out_2244001790848573428[2] = 0;
   out_2244001790848573428[3] = 0;
   out_2244001790848573428[4] = 0;
   out_2244001790848573428[5] = 0;
   out_2244001790848573428[6] = 1;
   out_2244001790848573428[7] = 0;
   out_2244001790848573428[8] = 0;
}
void h_24(double *state, double *unused, double *out_1885523254122962400) {
   out_1885523254122962400[0] = state[4];
   out_1885523254122962400[1] = state[5];
}
void H_24(double *state, double *unused, double *out_66787367241423455) {
   out_66787367241423455[0] = 0;
   out_66787367241423455[1] = 0;
   out_66787367241423455[2] = 0;
   out_66787367241423455[3] = 0;
   out_66787367241423455[4] = 1;
   out_66787367241423455[5] = 0;
   out_66787367241423455[6] = 0;
   out_66787367241423455[7] = 0;
   out_66787367241423455[8] = 0;
   out_66787367241423455[9] = 0;
   out_66787367241423455[10] = 0;
   out_66787367241423455[11] = 0;
   out_66787367241423455[12] = 0;
   out_66787367241423455[13] = 0;
   out_66787367241423455[14] = 1;
   out_66787367241423455[15] = 0;
   out_66787367241423455[16] = 0;
   out_66787367241423455[17] = 0;
}
void h_30(double *state, double *unused, double *out_7985026943421080424) {
   out_7985026943421080424[0] = state[4];
}
void H_30(double *state, double *unused, double *out_274331167658675199) {
   out_274331167658675199[0] = 0;
   out_274331167658675199[1] = 0;
   out_274331167658675199[2] = 0;
   out_274331167658675199[3] = 0;
   out_274331167658675199[4] = 1;
   out_274331167658675199[5] = 0;
   out_274331167658675199[6] = 0;
   out_274331167658675199[7] = 0;
   out_274331167658675199[8] = 0;
}
void h_26(double *state, double *unused, double *out_2983353675760234779) {
   out_2983353675760234779[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5985505109722629652) {
   out_5985505109722629652[0] = 0;
   out_5985505109722629652[1] = 0;
   out_5985505109722629652[2] = 0;
   out_5985505109722629652[3] = 0;
   out_5985505109722629652[4] = 0;
   out_5985505109722629652[5] = 0;
   out_5985505109722629652[6] = 0;
   out_5985505109722629652[7] = 1;
   out_5985505109722629652[8] = 0;
}
void h_27(double *state, double *unused, double *out_9032749139824150819) {
   out_9032749139824150819[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8946461432776606537) {
   out_8946461432776606537[0] = 0;
   out_8946461432776606537[1] = 0;
   out_8946461432776606537[2] = 0;
   out_8946461432776606537[3] = 1;
   out_8946461432776606537[4] = 0;
   out_8946461432776606537[5] = 0;
   out_8946461432776606537[6] = 0;
   out_8946461432776606537[7] = 0;
   out_8946461432776606537[8] = 0;
}
void h_29(double *state, double *unused, double *out_549883508992379611) {
   out_549883508992379611[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6261466776661789442) {
   out_6261466776661789442[0] = 0;
   out_6261466776661789442[1] = 1;
   out_6261466776661789442[2] = 0;
   out_6261466776661789442[3] = 0;
   out_6261466776661789442[4] = 0;
   out_6261466776661789442[5] = 0;
   out_6261466776661789442[6] = 0;
   out_6261466776661789442[7] = 0;
   out_6261466776661789442[8] = 0;
}
void h_28(double *state, double *unused, double *out_4031075872163305174) {
   out_4031075872163305174[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4297836505096463191) {
   out_4297836505096463191[0] = 1;
   out_4297836505096463191[1] = 0;
   out_4297836505096463191[2] = 0;
   out_4297836505096463191[3] = 0;
   out_4297836505096463191[4] = 0;
   out_4297836505096463191[5] = 0;
   out_4297836505096463191[6] = 0;
   out_4297836505096463191[7] = 0;
   out_4297836505096463191[8] = 0;
}
void h_31(double *state, double *unused, double *out_1599193907889125899) {
   out_1599193907889125899[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6611713211955981128) {
   out_6611713211955981128[0] = 0;
   out_6611713211955981128[1] = 0;
   out_6611713211955981128[2] = 0;
   out_6611713211955981128[3] = 0;
   out_6611713211955981128[4] = 0;
   out_6611713211955981128[5] = 0;
   out_6611713211955981128[6] = 0;
   out_6611713211955981128[7] = 0;
   out_6611713211955981128[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4696672223706615105) {
  err_fun(nom_x, delta_x, out_4696672223706615105);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5014476520979880068) {
  inv_err_fun(nom_x, true_x, out_5014476520979880068);
}
void car_H_mod_fun(double *state, double *out_4194163149970831217) {
  H_mod_fun(state, out_4194163149970831217);
}
void car_f_fun(double *state, double dt, double *out_8207092215987119012) {
  f_fun(state,  dt, out_8207092215987119012);
}
void car_F_fun(double *state, double dt, double *out_210187902771147012) {
  F_fun(state,  dt, out_210187902771147012);
}
void car_h_25(double *state, double *unused, double *out_3690881903938120256) {
  h_25(state, unused, out_3690881903938120256);
}
void car_H_25(double *state, double *unused, double *out_2244001790848573428) {
  H_25(state, unused, out_2244001790848573428);
}
void car_h_24(double *state, double *unused, double *out_1885523254122962400) {
  h_24(state, unused, out_1885523254122962400);
}
void car_H_24(double *state, double *unused, double *out_66787367241423455) {
  H_24(state, unused, out_66787367241423455);
}
void car_h_30(double *state, double *unused, double *out_7985026943421080424) {
  h_30(state, unused, out_7985026943421080424);
}
void car_H_30(double *state, double *unused, double *out_274331167658675199) {
  H_30(state, unused, out_274331167658675199);
}
void car_h_26(double *state, double *unused, double *out_2983353675760234779) {
  h_26(state, unused, out_2983353675760234779);
}
void car_H_26(double *state, double *unused, double *out_5985505109722629652) {
  H_26(state, unused, out_5985505109722629652);
}
void car_h_27(double *state, double *unused, double *out_9032749139824150819) {
  h_27(state, unused, out_9032749139824150819);
}
void car_H_27(double *state, double *unused, double *out_8946461432776606537) {
  H_27(state, unused, out_8946461432776606537);
}
void car_h_29(double *state, double *unused, double *out_549883508992379611) {
  h_29(state, unused, out_549883508992379611);
}
void car_H_29(double *state, double *unused, double *out_6261466776661789442) {
  H_29(state, unused, out_6261466776661789442);
}
void car_h_28(double *state, double *unused, double *out_4031075872163305174) {
  h_28(state, unused, out_4031075872163305174);
}
void car_H_28(double *state, double *unused, double *out_4297836505096463191) {
  H_28(state, unused, out_4297836505096463191);
}
void car_h_31(double *state, double *unused, double *out_1599193907889125899) {
  h_31(state, unused, out_1599193907889125899);
}
void car_H_31(double *state, double *unused, double *out_6611713211955981128) {
  H_31(state, unused, out_6611713211955981128);
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
