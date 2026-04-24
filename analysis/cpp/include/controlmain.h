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

    /** ``Y``가 주어지면 해당 상태로 ``self.Y``를 두고 ``[dq, ddq]``를 반환 (RK4 스테이지). */
    vec8 analysis(const vec8& Y_in);
    /** 현재 멤버 ``Y``로 동일 계산. */
    vec8 analysis();

    void data_save();
    void open_log(const std::string& path);
    void close_log();

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

    void read_data();
    void Y2qdq();
    vec8 dqddq2Yp();

    void position_calculation();
    void velocity_calculation();
    void mass_force_calculation();
    vec4 EQM();
    void acceleration_calculation();
};
