#pragma once

#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "types.h"
#include "utils.h"

typedef struct BodyClass {
    scalar qi, dqi, ddqi, qi_act, dqi_act, ddqi_act, gear;
    vec3 sijp, sij, ri, Hi, sep, se, re, rpy, u_vec;
    mat3 Cij, Aijpp, Ai, Ce, Ae;
    vec3 rhoip, rhoi, ric;
    mat3 Jip, Jic, Cii;
    scalar mi;
    scalar tau;
    vec3 wi, dri, dre, dric;
    mat3 wit, rit, drit, rict, drict, dwit;
    vec3 dHi, fic, tic;
    vec6 Yih, dYih, Bi, Di, Qih, Li, dYib;
    mat6 Mih, Ki, Ti, dTi;
    mat3 Ai_Cii;
    scalar Ti_tau;
    vec3 ddri, dwi, ddric, ddre;
} Body;

class ControlMain {
public:
    ControlMain();
    ~ControlMain();

    void run();
    void run_ik();
    void run_vsd();

private:
    Body base{};
    Body body[4]{};
    double h = 0;
    double t_c = 0;
    double t_e = 0;
    double g = 0;
    int index = 0;

    vec8 Y{};
    vec4 ddq{};
    std::ofstream fp;
    /** ``rec_data_path.csv``를 읽은 뒤 첫 열을 제외한 부분 (Python ``rec_data_raw[:, 1:]``). */
    Eigen::MatrixXd rec_data;
    std::vector< std::array<double, 3> > path_build(double* wp_t, double* wp_x, int wp_n, double ta, double h, bool fuul_quintic=false);
    std::vector< std::array<double, 3> > path_generation(double x0, double xf, double tf, double ta, double h, bool full_quintic=false);

    void read_data();
    void Y2qdq();
    vec8 dqddq2Yp();

    void position_calculation();
    void velocity_calculation();
    void mass_force_calculation();
    /** U = Σ m_i (-g) z_ric (`g`는 멤버 중력 가속도). */
    scalar gravity_potential_energy() const;
    /** ∂U/∂q (관절 일반화 중력토크), 중앙차분. */
    vec4 joint_gravity_torque();
    vec4 EQM();
    void acceleration_calculation();
    Eigen::Matrix<scalar, 5, 4> jacobian_calculation();
    mat3 A01pp_q1, A12pp_q2, A23pp_q3, A34pp_q4;
    mat3 A1_q1, A2_q1, A3_q1, A4_q1;
    mat3 A2_q2, A3_q2, A4_q2;
    mat3 A3_q3, A4_q3, A4_q4;
    mat3 Ae_q1, Ae_q2, Ae_q3, Ae_q4;
    vec3 jac_q1, jac_q2, jac_q3, jac_q4;
    Eigen::Matrix<scalar, 3, 4> jac_pos;
    std::array<Eigen::Matrix3d, 4> dAe_dq;
    Eigen::Matrix<scalar, 2, 4> jac_rp;
    Eigen::Matrix<scalar, 5, 4> J;
    Eigen::Matrix<scalar, 5, 5> JJT_reg;

    /** ``Y``가 주어지면 해당 상태로 ``self.Y``를 두고 ``[dq, ddq]``를 반환 (RK4 스테이지). */
    vec8 analysis(const vec8& Y_in);
    /** 현재 멤버 ``Y``로 동일 계산. */
    vec8 analysis();

    void data_save();
    void open_log(const std::string& path);
    void close_log();
};
