#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_344773025285735819);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8115144850596300460);
void car_H_mod_fun(double *state, double *out_2122213824906596317);
void car_f_fun(double *state, double dt, double *out_3194357031179583653);
void car_F_fun(double *state, double dt, double *out_8656604033789363826);
void car_h_25(double *state, double *unused, double *out_721844597343897651);
void car_H_25(double *state, double *unused, double *out_3342677436150207327);
void car_h_24(double *state, double *unused, double *out_584620759994682920);
void car_H_24(double *state, double *unused, double *out_215248162652115809);
void car_h_30(double *state, double *unused, double *out_6738714166904990674);
void car_H_30(double *state, double *unused, double *out_3472016383293447397);
void car_h_26(double *state, double *unused, double *out_6706356639143715297);
void car_H_26(double *state, double *unused, double *out_2685823372039895423);
void car_h_27(double *state, double *unused, double *out_221665255557764784);
void car_H_27(double *state, double *unused, double *out_5646779695093872308);
void car_h_29(double *state, double *unused, double *out_8671243855360228257);
void car_H_29(double *state, double *unused, double *out_2961785038979055213);
void car_h_28(double *state, double *unused, double *out_4780008012103080861);
void car_H_28(double *state, double *unused, double *out_998154767413728962);
void car_h_31(double *state, double *unused, double *out_879031265695026796);
void car_H_31(double *state, double *unused, double *out_3312031474273246899);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}