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
void err_fun(double *nom_x, double *delta_x, double *out_4341311303706823754) {
   out_4341311303706823754[0] = delta_x[0] + nom_x[0];
   out_4341311303706823754[1] = delta_x[1] + nom_x[1];
   out_4341311303706823754[2] = delta_x[2] + nom_x[2];
   out_4341311303706823754[3] = delta_x[3] + nom_x[3];
   out_4341311303706823754[4] = delta_x[4] + nom_x[4];
   out_4341311303706823754[5] = delta_x[5] + nom_x[5];
   out_4341311303706823754[6] = delta_x[6] + nom_x[6];
   out_4341311303706823754[7] = delta_x[7] + nom_x[7];
   out_4341311303706823754[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2772556154175351549) {
   out_2772556154175351549[0] = -nom_x[0] + true_x[0];
   out_2772556154175351549[1] = -nom_x[1] + true_x[1];
   out_2772556154175351549[2] = -nom_x[2] + true_x[2];
   out_2772556154175351549[3] = -nom_x[3] + true_x[3];
   out_2772556154175351549[4] = -nom_x[4] + true_x[4];
   out_2772556154175351549[5] = -nom_x[5] + true_x[5];
   out_2772556154175351549[6] = -nom_x[6] + true_x[6];
   out_2772556154175351549[7] = -nom_x[7] + true_x[7];
   out_2772556154175351549[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7428145175925163782) {
   out_7428145175925163782[0] = 1.0;
   out_7428145175925163782[1] = 0;
   out_7428145175925163782[2] = 0;
   out_7428145175925163782[3] = 0;
   out_7428145175925163782[4] = 0;
   out_7428145175925163782[5] = 0;
   out_7428145175925163782[6] = 0;
   out_7428145175925163782[7] = 0;
   out_7428145175925163782[8] = 0;
   out_7428145175925163782[9] = 0;
   out_7428145175925163782[10] = 1.0;
   out_7428145175925163782[11] = 0;
   out_7428145175925163782[12] = 0;
   out_7428145175925163782[13] = 0;
   out_7428145175925163782[14] = 0;
   out_7428145175925163782[15] = 0;
   out_7428145175925163782[16] = 0;
   out_7428145175925163782[17] = 0;
   out_7428145175925163782[18] = 0;
   out_7428145175925163782[19] = 0;
   out_7428145175925163782[20] = 1.0;
   out_7428145175925163782[21] = 0;
   out_7428145175925163782[22] = 0;
   out_7428145175925163782[23] = 0;
   out_7428145175925163782[24] = 0;
   out_7428145175925163782[25] = 0;
   out_7428145175925163782[26] = 0;
   out_7428145175925163782[27] = 0;
   out_7428145175925163782[28] = 0;
   out_7428145175925163782[29] = 0;
   out_7428145175925163782[30] = 1.0;
   out_7428145175925163782[31] = 0;
   out_7428145175925163782[32] = 0;
   out_7428145175925163782[33] = 0;
   out_7428145175925163782[34] = 0;
   out_7428145175925163782[35] = 0;
   out_7428145175925163782[36] = 0;
   out_7428145175925163782[37] = 0;
   out_7428145175925163782[38] = 0;
   out_7428145175925163782[39] = 0;
   out_7428145175925163782[40] = 1.0;
   out_7428145175925163782[41] = 0;
   out_7428145175925163782[42] = 0;
   out_7428145175925163782[43] = 0;
   out_7428145175925163782[44] = 0;
   out_7428145175925163782[45] = 0;
   out_7428145175925163782[46] = 0;
   out_7428145175925163782[47] = 0;
   out_7428145175925163782[48] = 0;
   out_7428145175925163782[49] = 0;
   out_7428145175925163782[50] = 1.0;
   out_7428145175925163782[51] = 0;
   out_7428145175925163782[52] = 0;
   out_7428145175925163782[53] = 0;
   out_7428145175925163782[54] = 0;
   out_7428145175925163782[55] = 0;
   out_7428145175925163782[56] = 0;
   out_7428145175925163782[57] = 0;
   out_7428145175925163782[58] = 0;
   out_7428145175925163782[59] = 0;
   out_7428145175925163782[60] = 1.0;
   out_7428145175925163782[61] = 0;
   out_7428145175925163782[62] = 0;
   out_7428145175925163782[63] = 0;
   out_7428145175925163782[64] = 0;
   out_7428145175925163782[65] = 0;
   out_7428145175925163782[66] = 0;
   out_7428145175925163782[67] = 0;
   out_7428145175925163782[68] = 0;
   out_7428145175925163782[69] = 0;
   out_7428145175925163782[70] = 1.0;
   out_7428145175925163782[71] = 0;
   out_7428145175925163782[72] = 0;
   out_7428145175925163782[73] = 0;
   out_7428145175925163782[74] = 0;
   out_7428145175925163782[75] = 0;
   out_7428145175925163782[76] = 0;
   out_7428145175925163782[77] = 0;
   out_7428145175925163782[78] = 0;
   out_7428145175925163782[79] = 0;
   out_7428145175925163782[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3699809735407870276) {
   out_3699809735407870276[0] = state[0];
   out_3699809735407870276[1] = state[1];
   out_3699809735407870276[2] = state[2];
   out_3699809735407870276[3] = state[3];
   out_3699809735407870276[4] = state[4];
   out_3699809735407870276[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3699809735407870276[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3699809735407870276[7] = state[7];
   out_3699809735407870276[8] = state[8];
}
void F_fun(double *state, double dt, double *out_538816335070506230) {
   out_538816335070506230[0] = 1;
   out_538816335070506230[1] = 0;
   out_538816335070506230[2] = 0;
   out_538816335070506230[3] = 0;
   out_538816335070506230[4] = 0;
   out_538816335070506230[5] = 0;
   out_538816335070506230[6] = 0;
   out_538816335070506230[7] = 0;
   out_538816335070506230[8] = 0;
   out_538816335070506230[9] = 0;
   out_538816335070506230[10] = 1;
   out_538816335070506230[11] = 0;
   out_538816335070506230[12] = 0;
   out_538816335070506230[13] = 0;
   out_538816335070506230[14] = 0;
   out_538816335070506230[15] = 0;
   out_538816335070506230[16] = 0;
   out_538816335070506230[17] = 0;
   out_538816335070506230[18] = 0;
   out_538816335070506230[19] = 0;
   out_538816335070506230[20] = 1;
   out_538816335070506230[21] = 0;
   out_538816335070506230[22] = 0;
   out_538816335070506230[23] = 0;
   out_538816335070506230[24] = 0;
   out_538816335070506230[25] = 0;
   out_538816335070506230[26] = 0;
   out_538816335070506230[27] = 0;
   out_538816335070506230[28] = 0;
   out_538816335070506230[29] = 0;
   out_538816335070506230[30] = 1;
   out_538816335070506230[31] = 0;
   out_538816335070506230[32] = 0;
   out_538816335070506230[33] = 0;
   out_538816335070506230[34] = 0;
   out_538816335070506230[35] = 0;
   out_538816335070506230[36] = 0;
   out_538816335070506230[37] = 0;
   out_538816335070506230[38] = 0;
   out_538816335070506230[39] = 0;
   out_538816335070506230[40] = 1;
   out_538816335070506230[41] = 0;
   out_538816335070506230[42] = 0;
   out_538816335070506230[43] = 0;
   out_538816335070506230[44] = 0;
   out_538816335070506230[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_538816335070506230[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_538816335070506230[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_538816335070506230[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_538816335070506230[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_538816335070506230[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_538816335070506230[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_538816335070506230[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_538816335070506230[53] = -9.8000000000000007*dt;
   out_538816335070506230[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_538816335070506230[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_538816335070506230[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_538816335070506230[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_538816335070506230[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_538816335070506230[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_538816335070506230[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_538816335070506230[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_538816335070506230[62] = 0;
   out_538816335070506230[63] = 0;
   out_538816335070506230[64] = 0;
   out_538816335070506230[65] = 0;
   out_538816335070506230[66] = 0;
   out_538816335070506230[67] = 0;
   out_538816335070506230[68] = 0;
   out_538816335070506230[69] = 0;
   out_538816335070506230[70] = 1;
   out_538816335070506230[71] = 0;
   out_538816335070506230[72] = 0;
   out_538816335070506230[73] = 0;
   out_538816335070506230[74] = 0;
   out_538816335070506230[75] = 0;
   out_538816335070506230[76] = 0;
   out_538816335070506230[77] = 0;
   out_538816335070506230[78] = 0;
   out_538816335070506230[79] = 0;
   out_538816335070506230[80] = 1;
}
void h_25(double *state, double *unused, double *out_8603780821444072072) {
   out_8603780821444072072[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3712116104417966457) {
   out_3712116104417966457[0] = 0;
   out_3712116104417966457[1] = 0;
   out_3712116104417966457[2] = 0;
   out_3712116104417966457[3] = 0;
   out_3712116104417966457[4] = 0;
   out_3712116104417966457[5] = 0;
   out_3712116104417966457[6] = 1;
   out_3712116104417966457[7] = 0;
   out_3712116104417966457[8] = 0;
}
void h_24(double *state, double *unused, double *out_2115321085035505881) {
   out_2115321085035505881[0] = state[4];
   out_2115321085035505881[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4108761408771825952) {
   out_4108761408771825952[0] = 0;
   out_4108761408771825952[1] = 0;
   out_4108761408771825952[2] = 0;
   out_4108761408771825952[3] = 0;
   out_4108761408771825952[4] = 1;
   out_4108761408771825952[5] = 0;
   out_4108761408771825952[6] = 0;
   out_4108761408771825952[7] = 0;
   out_4108761408771825952[8] = 0;
   out_4108761408771825952[9] = 0;
   out_4108761408771825952[10] = 0;
   out_4108761408771825952[11] = 0;
   out_4108761408771825952[12] = 0;
   out_4108761408771825952[13] = 0;
   out_4108761408771825952[14] = 1;
   out_4108761408771825952[15] = 0;
   out_4108761408771825952[16] = 0;
   out_4108761408771825952[17] = 0;
}
void h_30(double *state, double *unused, double *out_533936463239779110) {
   out_533936463239779110[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3204574237073650298) {
   out_3204574237073650298[0] = 0;
   out_3204574237073650298[1] = 0;
   out_3204574237073650298[2] = 0;
   out_3204574237073650298[3] = 0;
   out_3204574237073650298[4] = 1;
   out_3204574237073650298[5] = 0;
   out_3204574237073650298[6] = 0;
   out_3204574237073650298[7] = 0;
   out_3204574237073650298[8] = 0;
}
void h_26(double *state, double *unused, double *out_5535609730900624755) {
   out_5535609730900624755[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7453619423292022681) {
   out_7453619423292022681[0] = 0;
   out_7453619423292022681[1] = 0;
   out_7453619423292022681[2] = 0;
   out_7453619423292022681[3] = 0;
   out_7453619423292022681[4] = 0;
   out_7453619423292022681[5] = 0;
   out_7453619423292022681[6] = 0;
   out_7453619423292022681[7] = 1;
   out_7453619423292022681[8] = 0;
}
void h_27(double *state, double *unused, double *out_1019244953982879883) {
   out_1019244953982879883[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1029810925273225387) {
   out_1029810925273225387[0] = 0;
   out_1029810925273225387[1] = 0;
   out_1029810925273225387[2] = 0;
   out_1029810925273225387[3] = 1;
   out_1029810925273225387[4] = 0;
   out_1029810925273225387[5] = 0;
   out_1029810925273225387[6] = 0;
   out_1029810925273225387[7] = 0;
   out_1029810925273225387[8] = 0;
}
void h_29(double *state, double *unused, double *out_5842188832284245276) {
   out_5842188832284245276[0] = state[1];
}
void H_29(double *state, double *unused, double *out_683551801596325646) {
   out_683551801596325646[0] = 0;
   out_683551801596325646[1] = 1;
   out_683551801596325646[2] = 0;
   out_683551801596325646[3] = 0;
   out_683551801596325646[4] = 0;
   out_683551801596325646[5] = 0;
   out_683551801596325646[6] = 0;
   out_683551801596325646[7] = 0;
   out_683551801596325646[8] = 0;
}
void h_28(double *state, double *unused, double *out_3378707127298749129) {
   out_3378707127298749129[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5765950818665856220) {
   out_5765950818665856220[0] = 1;
   out_5765950818665856220[1] = 0;
   out_5765950818665856220[2] = 0;
   out_5765950818665856220[3] = 0;
   out_5765950818665856220[4] = 0;
   out_5765950818665856220[5] = 0;
   out_5765950818665856220[6] = 0;
   out_5765950818665856220[7] = 0;
   out_5765950818665856220[8] = 0;
}
void h_31(double *state, double *unused, double *out_81451567269911697) {
   out_81451567269911697[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3681470142541006029) {
   out_3681470142541006029[0] = 0;
   out_3681470142541006029[1] = 0;
   out_3681470142541006029[2] = 0;
   out_3681470142541006029[3] = 0;
   out_3681470142541006029[4] = 0;
   out_3681470142541006029[5] = 0;
   out_3681470142541006029[6] = 0;
   out_3681470142541006029[7] = 0;
   out_3681470142541006029[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4341311303706823754) {
  err_fun(nom_x, delta_x, out_4341311303706823754);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2772556154175351549) {
  inv_err_fun(nom_x, true_x, out_2772556154175351549);
}
void car_H_mod_fun(double *state, double *out_7428145175925163782) {
  H_mod_fun(state, out_7428145175925163782);
}
void car_f_fun(double *state, double dt, double *out_3699809735407870276) {
  f_fun(state,  dt, out_3699809735407870276);
}
void car_F_fun(double *state, double dt, double *out_538816335070506230) {
  F_fun(state,  dt, out_538816335070506230);
}
void car_h_25(double *state, double *unused, double *out_8603780821444072072) {
  h_25(state, unused, out_8603780821444072072);
}
void car_H_25(double *state, double *unused, double *out_3712116104417966457) {
  H_25(state, unused, out_3712116104417966457);
}
void car_h_24(double *state, double *unused, double *out_2115321085035505881) {
  h_24(state, unused, out_2115321085035505881);
}
void car_H_24(double *state, double *unused, double *out_4108761408771825952) {
  H_24(state, unused, out_4108761408771825952);
}
void car_h_30(double *state, double *unused, double *out_533936463239779110) {
  h_30(state, unused, out_533936463239779110);
}
void car_H_30(double *state, double *unused, double *out_3204574237073650298) {
  H_30(state, unused, out_3204574237073650298);
}
void car_h_26(double *state, double *unused, double *out_5535609730900624755) {
  h_26(state, unused, out_5535609730900624755);
}
void car_H_26(double *state, double *unused, double *out_7453619423292022681) {
  H_26(state, unused, out_7453619423292022681);
}
void car_h_27(double *state, double *unused, double *out_1019244953982879883) {
  h_27(state, unused, out_1019244953982879883);
}
void car_H_27(double *state, double *unused, double *out_1029810925273225387) {
  H_27(state, unused, out_1029810925273225387);
}
void car_h_29(double *state, double *unused, double *out_5842188832284245276) {
  h_29(state, unused, out_5842188832284245276);
}
void car_H_29(double *state, double *unused, double *out_683551801596325646) {
  H_29(state, unused, out_683551801596325646);
}
void car_h_28(double *state, double *unused, double *out_3378707127298749129) {
  h_28(state, unused, out_3378707127298749129);
}
void car_H_28(double *state, double *unused, double *out_5765950818665856220) {
  H_28(state, unused, out_5765950818665856220);
}
void car_h_31(double *state, double *unused, double *out_81451567269911697) {
  h_31(state, unused, out_81451567269911697);
}
void car_H_31(double *state, double *unused, double *out_3681470142541006029) {
  H_31(state, unused, out_3681470142541006029);
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
