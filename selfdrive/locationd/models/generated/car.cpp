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
void err_fun(double *nom_x, double *delta_x, double *out_1331898109693414845) {
   out_1331898109693414845[0] = delta_x[0] + nom_x[0];
   out_1331898109693414845[1] = delta_x[1] + nom_x[1];
   out_1331898109693414845[2] = delta_x[2] + nom_x[2];
   out_1331898109693414845[3] = delta_x[3] + nom_x[3];
   out_1331898109693414845[4] = delta_x[4] + nom_x[4];
   out_1331898109693414845[5] = delta_x[5] + nom_x[5];
   out_1331898109693414845[6] = delta_x[6] + nom_x[6];
   out_1331898109693414845[7] = delta_x[7] + nom_x[7];
   out_1331898109693414845[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3228453431263078712) {
   out_3228453431263078712[0] = -nom_x[0] + true_x[0];
   out_3228453431263078712[1] = -nom_x[1] + true_x[1];
   out_3228453431263078712[2] = -nom_x[2] + true_x[2];
   out_3228453431263078712[3] = -nom_x[3] + true_x[3];
   out_3228453431263078712[4] = -nom_x[4] + true_x[4];
   out_3228453431263078712[5] = -nom_x[5] + true_x[5];
   out_3228453431263078712[6] = -nom_x[6] + true_x[6];
   out_3228453431263078712[7] = -nom_x[7] + true_x[7];
   out_3228453431263078712[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2553420295817803628) {
   out_2553420295817803628[0] = 1.0;
   out_2553420295817803628[1] = 0;
   out_2553420295817803628[2] = 0;
   out_2553420295817803628[3] = 0;
   out_2553420295817803628[4] = 0;
   out_2553420295817803628[5] = 0;
   out_2553420295817803628[6] = 0;
   out_2553420295817803628[7] = 0;
   out_2553420295817803628[8] = 0;
   out_2553420295817803628[9] = 0;
   out_2553420295817803628[10] = 1.0;
   out_2553420295817803628[11] = 0;
   out_2553420295817803628[12] = 0;
   out_2553420295817803628[13] = 0;
   out_2553420295817803628[14] = 0;
   out_2553420295817803628[15] = 0;
   out_2553420295817803628[16] = 0;
   out_2553420295817803628[17] = 0;
   out_2553420295817803628[18] = 0;
   out_2553420295817803628[19] = 0;
   out_2553420295817803628[20] = 1.0;
   out_2553420295817803628[21] = 0;
   out_2553420295817803628[22] = 0;
   out_2553420295817803628[23] = 0;
   out_2553420295817803628[24] = 0;
   out_2553420295817803628[25] = 0;
   out_2553420295817803628[26] = 0;
   out_2553420295817803628[27] = 0;
   out_2553420295817803628[28] = 0;
   out_2553420295817803628[29] = 0;
   out_2553420295817803628[30] = 1.0;
   out_2553420295817803628[31] = 0;
   out_2553420295817803628[32] = 0;
   out_2553420295817803628[33] = 0;
   out_2553420295817803628[34] = 0;
   out_2553420295817803628[35] = 0;
   out_2553420295817803628[36] = 0;
   out_2553420295817803628[37] = 0;
   out_2553420295817803628[38] = 0;
   out_2553420295817803628[39] = 0;
   out_2553420295817803628[40] = 1.0;
   out_2553420295817803628[41] = 0;
   out_2553420295817803628[42] = 0;
   out_2553420295817803628[43] = 0;
   out_2553420295817803628[44] = 0;
   out_2553420295817803628[45] = 0;
   out_2553420295817803628[46] = 0;
   out_2553420295817803628[47] = 0;
   out_2553420295817803628[48] = 0;
   out_2553420295817803628[49] = 0;
   out_2553420295817803628[50] = 1.0;
   out_2553420295817803628[51] = 0;
   out_2553420295817803628[52] = 0;
   out_2553420295817803628[53] = 0;
   out_2553420295817803628[54] = 0;
   out_2553420295817803628[55] = 0;
   out_2553420295817803628[56] = 0;
   out_2553420295817803628[57] = 0;
   out_2553420295817803628[58] = 0;
   out_2553420295817803628[59] = 0;
   out_2553420295817803628[60] = 1.0;
   out_2553420295817803628[61] = 0;
   out_2553420295817803628[62] = 0;
   out_2553420295817803628[63] = 0;
   out_2553420295817803628[64] = 0;
   out_2553420295817803628[65] = 0;
   out_2553420295817803628[66] = 0;
   out_2553420295817803628[67] = 0;
   out_2553420295817803628[68] = 0;
   out_2553420295817803628[69] = 0;
   out_2553420295817803628[70] = 1.0;
   out_2553420295817803628[71] = 0;
   out_2553420295817803628[72] = 0;
   out_2553420295817803628[73] = 0;
   out_2553420295817803628[74] = 0;
   out_2553420295817803628[75] = 0;
   out_2553420295817803628[76] = 0;
   out_2553420295817803628[77] = 0;
   out_2553420295817803628[78] = 0;
   out_2553420295817803628[79] = 0;
   out_2553420295817803628[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7018245215613934816) {
   out_7018245215613934816[0] = state[0];
   out_7018245215613934816[1] = state[1];
   out_7018245215613934816[2] = state[2];
   out_7018245215613934816[3] = state[3];
   out_7018245215613934816[4] = state[4];
   out_7018245215613934816[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7018245215613934816[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7018245215613934816[7] = state[7];
   out_7018245215613934816[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7460937261702248251) {
   out_7460937261702248251[0] = 1;
   out_7460937261702248251[1] = 0;
   out_7460937261702248251[2] = 0;
   out_7460937261702248251[3] = 0;
   out_7460937261702248251[4] = 0;
   out_7460937261702248251[5] = 0;
   out_7460937261702248251[6] = 0;
   out_7460937261702248251[7] = 0;
   out_7460937261702248251[8] = 0;
   out_7460937261702248251[9] = 0;
   out_7460937261702248251[10] = 1;
   out_7460937261702248251[11] = 0;
   out_7460937261702248251[12] = 0;
   out_7460937261702248251[13] = 0;
   out_7460937261702248251[14] = 0;
   out_7460937261702248251[15] = 0;
   out_7460937261702248251[16] = 0;
   out_7460937261702248251[17] = 0;
   out_7460937261702248251[18] = 0;
   out_7460937261702248251[19] = 0;
   out_7460937261702248251[20] = 1;
   out_7460937261702248251[21] = 0;
   out_7460937261702248251[22] = 0;
   out_7460937261702248251[23] = 0;
   out_7460937261702248251[24] = 0;
   out_7460937261702248251[25] = 0;
   out_7460937261702248251[26] = 0;
   out_7460937261702248251[27] = 0;
   out_7460937261702248251[28] = 0;
   out_7460937261702248251[29] = 0;
   out_7460937261702248251[30] = 1;
   out_7460937261702248251[31] = 0;
   out_7460937261702248251[32] = 0;
   out_7460937261702248251[33] = 0;
   out_7460937261702248251[34] = 0;
   out_7460937261702248251[35] = 0;
   out_7460937261702248251[36] = 0;
   out_7460937261702248251[37] = 0;
   out_7460937261702248251[38] = 0;
   out_7460937261702248251[39] = 0;
   out_7460937261702248251[40] = 1;
   out_7460937261702248251[41] = 0;
   out_7460937261702248251[42] = 0;
   out_7460937261702248251[43] = 0;
   out_7460937261702248251[44] = 0;
   out_7460937261702248251[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7460937261702248251[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7460937261702248251[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7460937261702248251[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7460937261702248251[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7460937261702248251[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7460937261702248251[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7460937261702248251[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7460937261702248251[53] = -9.8000000000000007*dt;
   out_7460937261702248251[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7460937261702248251[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7460937261702248251[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7460937261702248251[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7460937261702248251[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7460937261702248251[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7460937261702248251[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7460937261702248251[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7460937261702248251[62] = 0;
   out_7460937261702248251[63] = 0;
   out_7460937261702248251[64] = 0;
   out_7460937261702248251[65] = 0;
   out_7460937261702248251[66] = 0;
   out_7460937261702248251[67] = 0;
   out_7460937261702248251[68] = 0;
   out_7460937261702248251[69] = 0;
   out_7460937261702248251[70] = 1;
   out_7460937261702248251[71] = 0;
   out_7460937261702248251[72] = 0;
   out_7460937261702248251[73] = 0;
   out_7460937261702248251[74] = 0;
   out_7460937261702248251[75] = 0;
   out_7460937261702248251[76] = 0;
   out_7460937261702248251[77] = 0;
   out_7460937261702248251[78] = 0;
   out_7460937261702248251[79] = 0;
   out_7460937261702248251[80] = 1;
}
void h_25(double *state, double *unused, double *out_3041199912639663321) {
   out_3041199912639663321[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8445651218056560338) {
   out_8445651218056560338[0] = 0;
   out_8445651218056560338[1] = 0;
   out_8445651218056560338[2] = 0;
   out_8445651218056560338[3] = 0;
   out_8445651218056560338[4] = 0;
   out_8445651218056560338[5] = 0;
   out_8445651218056560338[6] = 1;
   out_8445651218056560338[7] = 0;
   out_8445651218056560338[8] = 0;
}
void h_24(double *state, double *unused, double *out_5687290987886979710) {
   out_5687290987886979710[0] = state[4];
   out_5687290987886979710[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7975193736013221614) {
   out_7975193736013221614[0] = 0;
   out_7975193736013221614[1] = 0;
   out_7975193736013221614[2] = 0;
   out_7975193736013221614[3] = 0;
   out_7975193736013221614[4] = 1;
   out_7975193736013221614[5] = 0;
   out_7975193736013221614[6] = 0;
   out_7975193736013221614[7] = 0;
   out_7975193736013221614[8] = 0;
   out_7975193736013221614[9] = 0;
   out_7975193736013221614[10] = 0;
   out_7975193736013221614[11] = 0;
   out_7975193736013221614[12] = 0;
   out_7975193736013221614[13] = 0;
   out_7975193736013221614[14] = 1;
   out_7975193736013221614[15] = 0;
   out_7975193736013221614[16] = 0;
   out_7975193736013221614[17] = 0;
}
void h_30(double *state, double *unused, double *out_5822911512253357865) {
   out_5822911512253357865[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8316312270913320268) {
   out_8316312270913320268[0] = 0;
   out_8316312270913320268[1] = 0;
   out_8316312270913320268[2] = 0;
   out_8316312270913320268[3] = 0;
   out_8316312270913320268[4] = 1;
   out_8316312270913320268[5] = 0;
   out_8316312270913320268[6] = 0;
   out_8316312270913320268[7] = 0;
   out_8316312270913320268[8] = 0;
}
void h_26(double *state, double *unused, double *out_6981102690375236916) {
   out_6981102690375236916[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4704147899182504114) {
   out_4704147899182504114[0] = 0;
   out_4704147899182504114[1] = 0;
   out_4704147899182504114[2] = 0;
   out_4704147899182504114[3] = 0;
   out_4704147899182504114[4] = 0;
   out_4704147899182504114[5] = 0;
   out_4704147899182504114[6] = 0;
   out_4704147899182504114[7] = 1;
   out_4704147899182504114[8] = 0;
}
void h_27(double *state, double *unused, double *out_1240363928603997187) {
   out_1240363928603997187[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6141548959112895357) {
   out_6141548959112895357[0] = 0;
   out_6141548959112895357[1] = 0;
   out_6141548959112895357[2] = 0;
   out_6141548959112895357[3] = 1;
   out_6141548959112895357[4] = 0;
   out_6141548959112895357[5] = 0;
   out_6141548959112895357[6] = 0;
   out_6141548959112895357[7] = 0;
   out_6141548959112895357[8] = 0;
}
void h_29(double *state, double *unused, double *out_6766421365155020300) {
   out_6766421365155020300[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4428186232243344324) {
   out_4428186232243344324[0] = 0;
   out_4428186232243344324[1] = 1;
   out_4428186232243344324[2] = 0;
   out_4428186232243344324[3] = 0;
   out_4428186232243344324[4] = 0;
   out_4428186232243344324[5] = 0;
   out_4428186232243344324[6] = 0;
   out_4428186232243344324[7] = 0;
   out_4428186232243344324[8] = 0;
}
void h_28(double *state, double *unused, double *out_5006087128856176281) {
   out_5006087128856176281[0] = state[0];
}
void H_28(double *state, double *unused, double *out_654212784826186250) {
   out_654212784826186250[0] = 1;
   out_654212784826186250[1] = 0;
   out_654212784826186250[2] = 0;
   out_654212784826186250[3] = 0;
   out_654212784826186250[4] = 0;
   out_654212784826186250[5] = 0;
   out_654212784826186250[6] = 0;
   out_654212784826186250[7] = 0;
   out_654212784826186250[8] = 0;
}
void h_31(double *state, double *unused, double *out_1521228060555829609) {
   out_1521228060555829609[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8476297179933520766) {
   out_8476297179933520766[0] = 0;
   out_8476297179933520766[1] = 0;
   out_8476297179933520766[2] = 0;
   out_8476297179933520766[3] = 0;
   out_8476297179933520766[4] = 0;
   out_8476297179933520766[5] = 0;
   out_8476297179933520766[6] = 0;
   out_8476297179933520766[7] = 0;
   out_8476297179933520766[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1331898109693414845) {
  err_fun(nom_x, delta_x, out_1331898109693414845);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3228453431263078712) {
  inv_err_fun(nom_x, true_x, out_3228453431263078712);
}
void car_H_mod_fun(double *state, double *out_2553420295817803628) {
  H_mod_fun(state, out_2553420295817803628);
}
void car_f_fun(double *state, double dt, double *out_7018245215613934816) {
  f_fun(state,  dt, out_7018245215613934816);
}
void car_F_fun(double *state, double dt, double *out_7460937261702248251) {
  F_fun(state,  dt, out_7460937261702248251);
}
void car_h_25(double *state, double *unused, double *out_3041199912639663321) {
  h_25(state, unused, out_3041199912639663321);
}
void car_H_25(double *state, double *unused, double *out_8445651218056560338) {
  H_25(state, unused, out_8445651218056560338);
}
void car_h_24(double *state, double *unused, double *out_5687290987886979710) {
  h_24(state, unused, out_5687290987886979710);
}
void car_H_24(double *state, double *unused, double *out_7975193736013221614) {
  H_24(state, unused, out_7975193736013221614);
}
void car_h_30(double *state, double *unused, double *out_5822911512253357865) {
  h_30(state, unused, out_5822911512253357865);
}
void car_H_30(double *state, double *unused, double *out_8316312270913320268) {
  H_30(state, unused, out_8316312270913320268);
}
void car_h_26(double *state, double *unused, double *out_6981102690375236916) {
  h_26(state, unused, out_6981102690375236916);
}
void car_H_26(double *state, double *unused, double *out_4704147899182504114) {
  H_26(state, unused, out_4704147899182504114);
}
void car_h_27(double *state, double *unused, double *out_1240363928603997187) {
  h_27(state, unused, out_1240363928603997187);
}
void car_H_27(double *state, double *unused, double *out_6141548959112895357) {
  H_27(state, unused, out_6141548959112895357);
}
void car_h_29(double *state, double *unused, double *out_6766421365155020300) {
  h_29(state, unused, out_6766421365155020300);
}
void car_H_29(double *state, double *unused, double *out_4428186232243344324) {
  H_29(state, unused, out_4428186232243344324);
}
void car_h_28(double *state, double *unused, double *out_5006087128856176281) {
  h_28(state, unused, out_5006087128856176281);
}
void car_H_28(double *state, double *unused, double *out_654212784826186250) {
  H_28(state, unused, out_654212784826186250);
}
void car_h_31(double *state, double *unused, double *out_1521228060555829609) {
  h_31(state, unused, out_1521228060555829609);
}
void car_H_31(double *state, double *unused, double *out_8476297179933520766) {
  H_31(state, unused, out_8476297179933520766);
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
