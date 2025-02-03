#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3710923511766162661);
void live_err_fun(double *nom_x, double *delta_x, double *out_8235738597437757261);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1427613531133799841);
void live_H_mod_fun(double *state, double *out_7973615964457922447);
void live_f_fun(double *state, double dt, double *out_7275339822647435942);
void live_F_fun(double *state, double dt, double *out_2029680424947406875);
void live_h_4(double *state, double *unused, double *out_642530476629486358);
void live_H_4(double *state, double *unused, double *out_1039847706362814556);
void live_h_9(double *state, double *unused, double *out_1767915360412975078);
void live_H_9(double *state, double *unused, double *out_798658059733223911);
void live_h_10(double *state, double *unused, double *out_713097810682123197);
void live_H_10(double *state, double *unused, double *out_5881857032909129039);
void live_h_12(double *state, double *unused, double *out_812844079194924243);
void live_H_12(double *state, double *unused, double *out_3979608701669147239);
void live_h_35(double *state, double *unused, double *out_8061967074050234911);
void live_H_35(double *state, double *unused, double *out_2326814351009792820);
void live_h_32(double *state, double *unused, double *out_9004229427108273613);
void live_H_32(double *state, double *unused, double *out_4588050152684323536);
void live_h_13(double *state, double *unused, double *out_2195951255504061060);
void live_H_13(double *state, double *unused, double *out_6169172646337010757);
void live_h_14(double *state, double *unused, double *out_1767915360412975078);
void live_H_14(double *state, double *unused, double *out_798658059733223911);
void live_h_33(double *state, double *unused, double *out_909979849639614577);
void live_H_33(double *state, double *unused, double *out_5477371355648650424);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}