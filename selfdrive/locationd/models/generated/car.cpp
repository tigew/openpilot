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
void err_fun(double *nom_x, double *delta_x, double *out_6542580248912300035) {
   out_6542580248912300035[0] = delta_x[0] + nom_x[0];
   out_6542580248912300035[1] = delta_x[1] + nom_x[1];
   out_6542580248912300035[2] = delta_x[2] + nom_x[2];
   out_6542580248912300035[3] = delta_x[3] + nom_x[3];
   out_6542580248912300035[4] = delta_x[4] + nom_x[4];
   out_6542580248912300035[5] = delta_x[5] + nom_x[5];
   out_6542580248912300035[6] = delta_x[6] + nom_x[6];
   out_6542580248912300035[7] = delta_x[7] + nom_x[7];
   out_6542580248912300035[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1014258657320824476) {
   out_1014258657320824476[0] = -nom_x[0] + true_x[0];
   out_1014258657320824476[1] = -nom_x[1] + true_x[1];
   out_1014258657320824476[2] = -nom_x[2] + true_x[2];
   out_1014258657320824476[3] = -nom_x[3] + true_x[3];
   out_1014258657320824476[4] = -nom_x[4] + true_x[4];
   out_1014258657320824476[5] = -nom_x[5] + true_x[5];
   out_1014258657320824476[6] = -nom_x[6] + true_x[6];
   out_1014258657320824476[7] = -nom_x[7] + true_x[7];
   out_1014258657320824476[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3607293837968811587) {
   out_3607293837968811587[0] = 1.0;
   out_3607293837968811587[1] = 0;
   out_3607293837968811587[2] = 0;
   out_3607293837968811587[3] = 0;
   out_3607293837968811587[4] = 0;
   out_3607293837968811587[5] = 0;
   out_3607293837968811587[6] = 0;
   out_3607293837968811587[7] = 0;
   out_3607293837968811587[8] = 0;
   out_3607293837968811587[9] = 0;
   out_3607293837968811587[10] = 1.0;
   out_3607293837968811587[11] = 0;
   out_3607293837968811587[12] = 0;
   out_3607293837968811587[13] = 0;
   out_3607293837968811587[14] = 0;
   out_3607293837968811587[15] = 0;
   out_3607293837968811587[16] = 0;
   out_3607293837968811587[17] = 0;
   out_3607293837968811587[18] = 0;
   out_3607293837968811587[19] = 0;
   out_3607293837968811587[20] = 1.0;
   out_3607293837968811587[21] = 0;
   out_3607293837968811587[22] = 0;
   out_3607293837968811587[23] = 0;
   out_3607293837968811587[24] = 0;
   out_3607293837968811587[25] = 0;
   out_3607293837968811587[26] = 0;
   out_3607293837968811587[27] = 0;
   out_3607293837968811587[28] = 0;
   out_3607293837968811587[29] = 0;
   out_3607293837968811587[30] = 1.0;
   out_3607293837968811587[31] = 0;
   out_3607293837968811587[32] = 0;
   out_3607293837968811587[33] = 0;
   out_3607293837968811587[34] = 0;
   out_3607293837968811587[35] = 0;
   out_3607293837968811587[36] = 0;
   out_3607293837968811587[37] = 0;
   out_3607293837968811587[38] = 0;
   out_3607293837968811587[39] = 0;
   out_3607293837968811587[40] = 1.0;
   out_3607293837968811587[41] = 0;
   out_3607293837968811587[42] = 0;
   out_3607293837968811587[43] = 0;
   out_3607293837968811587[44] = 0;
   out_3607293837968811587[45] = 0;
   out_3607293837968811587[46] = 0;
   out_3607293837968811587[47] = 0;
   out_3607293837968811587[48] = 0;
   out_3607293837968811587[49] = 0;
   out_3607293837968811587[50] = 1.0;
   out_3607293837968811587[51] = 0;
   out_3607293837968811587[52] = 0;
   out_3607293837968811587[53] = 0;
   out_3607293837968811587[54] = 0;
   out_3607293837968811587[55] = 0;
   out_3607293837968811587[56] = 0;
   out_3607293837968811587[57] = 0;
   out_3607293837968811587[58] = 0;
   out_3607293837968811587[59] = 0;
   out_3607293837968811587[60] = 1.0;
   out_3607293837968811587[61] = 0;
   out_3607293837968811587[62] = 0;
   out_3607293837968811587[63] = 0;
   out_3607293837968811587[64] = 0;
   out_3607293837968811587[65] = 0;
   out_3607293837968811587[66] = 0;
   out_3607293837968811587[67] = 0;
   out_3607293837968811587[68] = 0;
   out_3607293837968811587[69] = 0;
   out_3607293837968811587[70] = 1.0;
   out_3607293837968811587[71] = 0;
   out_3607293837968811587[72] = 0;
   out_3607293837968811587[73] = 0;
   out_3607293837968811587[74] = 0;
   out_3607293837968811587[75] = 0;
   out_3607293837968811587[76] = 0;
   out_3607293837968811587[77] = 0;
   out_3607293837968811587[78] = 0;
   out_3607293837968811587[79] = 0;
   out_3607293837968811587[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8315547614334729188) {
   out_8315547614334729188[0] = state[0];
   out_8315547614334729188[1] = state[1];
   out_8315547614334729188[2] = state[2];
   out_8315547614334729188[3] = state[3];
   out_8315547614334729188[4] = state[4];
   out_8315547614334729188[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8315547614334729188[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8315547614334729188[7] = state[7];
   out_8315547614334729188[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9009147579009871055) {
   out_9009147579009871055[0] = 1;
   out_9009147579009871055[1] = 0;
   out_9009147579009871055[2] = 0;
   out_9009147579009871055[3] = 0;
   out_9009147579009871055[4] = 0;
   out_9009147579009871055[5] = 0;
   out_9009147579009871055[6] = 0;
   out_9009147579009871055[7] = 0;
   out_9009147579009871055[8] = 0;
   out_9009147579009871055[9] = 0;
   out_9009147579009871055[10] = 1;
   out_9009147579009871055[11] = 0;
   out_9009147579009871055[12] = 0;
   out_9009147579009871055[13] = 0;
   out_9009147579009871055[14] = 0;
   out_9009147579009871055[15] = 0;
   out_9009147579009871055[16] = 0;
   out_9009147579009871055[17] = 0;
   out_9009147579009871055[18] = 0;
   out_9009147579009871055[19] = 0;
   out_9009147579009871055[20] = 1;
   out_9009147579009871055[21] = 0;
   out_9009147579009871055[22] = 0;
   out_9009147579009871055[23] = 0;
   out_9009147579009871055[24] = 0;
   out_9009147579009871055[25] = 0;
   out_9009147579009871055[26] = 0;
   out_9009147579009871055[27] = 0;
   out_9009147579009871055[28] = 0;
   out_9009147579009871055[29] = 0;
   out_9009147579009871055[30] = 1;
   out_9009147579009871055[31] = 0;
   out_9009147579009871055[32] = 0;
   out_9009147579009871055[33] = 0;
   out_9009147579009871055[34] = 0;
   out_9009147579009871055[35] = 0;
   out_9009147579009871055[36] = 0;
   out_9009147579009871055[37] = 0;
   out_9009147579009871055[38] = 0;
   out_9009147579009871055[39] = 0;
   out_9009147579009871055[40] = 1;
   out_9009147579009871055[41] = 0;
   out_9009147579009871055[42] = 0;
   out_9009147579009871055[43] = 0;
   out_9009147579009871055[44] = 0;
   out_9009147579009871055[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9009147579009871055[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9009147579009871055[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9009147579009871055[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9009147579009871055[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9009147579009871055[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9009147579009871055[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9009147579009871055[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9009147579009871055[53] = -9.8000000000000007*dt;
   out_9009147579009871055[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9009147579009871055[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9009147579009871055[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9009147579009871055[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9009147579009871055[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9009147579009871055[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9009147579009871055[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9009147579009871055[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9009147579009871055[62] = 0;
   out_9009147579009871055[63] = 0;
   out_9009147579009871055[64] = 0;
   out_9009147579009871055[65] = 0;
   out_9009147579009871055[66] = 0;
   out_9009147579009871055[67] = 0;
   out_9009147579009871055[68] = 0;
   out_9009147579009871055[69] = 0;
   out_9009147579009871055[70] = 1;
   out_9009147579009871055[71] = 0;
   out_9009147579009871055[72] = 0;
   out_9009147579009871055[73] = 0;
   out_9009147579009871055[74] = 0;
   out_9009147579009871055[75] = 0;
   out_9009147579009871055[76] = 0;
   out_9009147579009871055[77] = 0;
   out_9009147579009871055[78] = 0;
   out_9009147579009871055[79] = 0;
   out_9009147579009871055[80] = 1;
}
void h_25(double *state, double *unused, double *out_513819750196926609) {
   out_513819750196926609[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2713869850867312764) {
   out_2713869850867312764[0] = 0;
   out_2713869850867312764[1] = 0;
   out_2713869850867312764[2] = 0;
   out_2713869850867312764[3] = 0;
   out_2713869850867312764[4] = 0;
   out_2713869850867312764[5] = 0;
   out_2713869850867312764[6] = 1;
   out_2713869850867312764[7] = 0;
   out_2713869850867312764[8] = 0;
}
void h_24(double *state, double *unused, double *out_7508705919294981954) {
   out_7508705919294981954[0] = state[4];
   out_7508705919294981954[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1868873242083617365) {
   out_1868873242083617365[0] = 0;
   out_1868873242083617365[1] = 0;
   out_1868873242083617365[2] = 0;
   out_1868873242083617365[3] = 0;
   out_1868873242083617365[4] = 1;
   out_1868873242083617365[5] = 0;
   out_1868873242083617365[6] = 0;
   out_1868873242083617365[7] = 0;
   out_1868873242083617365[8] = 0;
   out_1868873242083617365[9] = 0;
   out_1868873242083617365[10] = 0;
   out_1868873242083617365[11] = 0;
   out_1868873242083617365[12] = 0;
   out_1868873242083617365[13] = 0;
   out_1868873242083617365[14] = 1;
   out_1868873242083617365[15] = 0;
   out_1868873242083617365[16] = 0;
   out_1868873242083617365[17] = 0;
}
void h_30(double *state, double *unused, double *out_936811642431956558) {
   out_936811642431956558[0] = state[4];
}
void H_30(double *state, double *unused, double *out_195536892360064137) {
   out_195536892360064137[0] = 0;
   out_195536892360064137[1] = 0;
   out_195536892360064137[2] = 0;
   out_195536892360064137[3] = 0;
   out_195536892360064137[4] = 1;
   out_195536892360064137[5] = 0;
   out_195536892360064137[6] = 0;
   out_195536892360064137[7] = 0;
   out_195536892360064137[8] = 0;
}
void h_26(double *state, double *unused, double *out_4821173271561781398) {
   out_4821173271561781398[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6455373169741368988) {
   out_6455373169741368988[0] = 0;
   out_6455373169741368988[1] = 0;
   out_6455373169741368988[2] = 0;
   out_6455373169741368988[3] = 0;
   out_6455373169741368988[4] = 0;
   out_6455373169741368988[5] = 0;
   out_6455373169741368988[6] = 0;
   out_6455373169741368988[7] = 1;
   out_6455373169741368988[8] = 0;
}
void h_27(double *state, double *unused, double *out_8733816862100309637) {
   out_8733816862100309637[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2028057178823879080) {
   out_2028057178823879080[0] = 0;
   out_2028057178823879080[1] = 0;
   out_2028057178823879080[2] = 0;
   out_2028057178823879080[3] = 1;
   out_2028057178823879080[4] = 0;
   out_2028057178823879080[5] = 0;
   out_2028057178823879080[6] = 0;
   out_2028057178823879080[7] = 0;
   out_2028057178823879080[8] = 0;
}
void h_29(double *state, double *unused, double *out_9069567579423277710) {
   out_9069567579423277710[0] = state[1];
}
void H_29(double *state, double *unused, double *out_314694451954328047) {
   out_314694451954328047[0] = 0;
   out_314694451954328047[1] = 1;
   out_314694451954328047[2] = 0;
   out_314694451954328047[3] = 0;
   out_314694451954328047[4] = 0;
   out_314694451954328047[5] = 0;
   out_314694451954328047[6] = 0;
   out_314694451954328047[7] = 0;
   out_314694451954328047[8] = 0;
}
void h_28(double *state, double *unused, double *out_7864833537133436639) {
   out_7864833537133436639[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4767704565115202527) {
   out_4767704565115202527[0] = 1;
   out_4767704565115202527[1] = 0;
   out_4767704565115202527[2] = 0;
   out_4767704565115202527[3] = 0;
   out_4767704565115202527[4] = 0;
   out_4767704565115202527[5] = 0;
   out_4767704565115202527[6] = 0;
   out_4767704565115202527[7] = 0;
   out_4767704565115202527[8] = 0;
}
void h_31(double *state, double *unused, double *out_6951335222449415183) {
   out_6951335222449415183[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2683223888990352336) {
   out_2683223888990352336[0] = 0;
   out_2683223888990352336[1] = 0;
   out_2683223888990352336[2] = 0;
   out_2683223888990352336[3] = 0;
   out_2683223888990352336[4] = 0;
   out_2683223888990352336[5] = 0;
   out_2683223888990352336[6] = 0;
   out_2683223888990352336[7] = 0;
   out_2683223888990352336[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6542580248912300035) {
  err_fun(nom_x, delta_x, out_6542580248912300035);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1014258657320824476) {
  inv_err_fun(nom_x, true_x, out_1014258657320824476);
}
void car_H_mod_fun(double *state, double *out_3607293837968811587) {
  H_mod_fun(state, out_3607293837968811587);
}
void car_f_fun(double *state, double dt, double *out_8315547614334729188) {
  f_fun(state,  dt, out_8315547614334729188);
}
void car_F_fun(double *state, double dt, double *out_9009147579009871055) {
  F_fun(state,  dt, out_9009147579009871055);
}
void car_h_25(double *state, double *unused, double *out_513819750196926609) {
  h_25(state, unused, out_513819750196926609);
}
void car_H_25(double *state, double *unused, double *out_2713869850867312764) {
  H_25(state, unused, out_2713869850867312764);
}
void car_h_24(double *state, double *unused, double *out_7508705919294981954) {
  h_24(state, unused, out_7508705919294981954);
}
void car_H_24(double *state, double *unused, double *out_1868873242083617365) {
  H_24(state, unused, out_1868873242083617365);
}
void car_h_30(double *state, double *unused, double *out_936811642431956558) {
  h_30(state, unused, out_936811642431956558);
}
void car_H_30(double *state, double *unused, double *out_195536892360064137) {
  H_30(state, unused, out_195536892360064137);
}
void car_h_26(double *state, double *unused, double *out_4821173271561781398) {
  h_26(state, unused, out_4821173271561781398);
}
void car_H_26(double *state, double *unused, double *out_6455373169741368988) {
  H_26(state, unused, out_6455373169741368988);
}
void car_h_27(double *state, double *unused, double *out_8733816862100309637) {
  h_27(state, unused, out_8733816862100309637);
}
void car_H_27(double *state, double *unused, double *out_2028057178823879080) {
  H_27(state, unused, out_2028057178823879080);
}
void car_h_29(double *state, double *unused, double *out_9069567579423277710) {
  h_29(state, unused, out_9069567579423277710);
}
void car_H_29(double *state, double *unused, double *out_314694451954328047) {
  H_29(state, unused, out_314694451954328047);
}
void car_h_28(double *state, double *unused, double *out_7864833537133436639) {
  h_28(state, unused, out_7864833537133436639);
}
void car_H_28(double *state, double *unused, double *out_4767704565115202527) {
  H_28(state, unused, out_4767704565115202527);
}
void car_h_31(double *state, double *unused, double *out_6951335222449415183) {
  h_31(state, unused, out_6951335222449415183);
}
void car_H_31(double *state, double *unused, double *out_2683223888990352336) {
  H_31(state, unused, out_2683223888990352336);
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
