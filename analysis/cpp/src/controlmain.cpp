#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include <Eigen/LU>

#include "controlmain.h"

namespace {

constexpr int NB = 4;

double env_or_default(const char* name, double default_value)
{
    const char* v = std::getenv(name);
    if (!v) return default_value;
    char* end = nullptr;
    const double parsed = std::strtod(v, &end);
    if (end == v) return default_value;
    return parsed;
}

void fill_inertia(mat3& J, scalar Ixx, scalar Ixy, scalar Iyy, scalar Iyz, scalar Izz, scalar Izx)
{
    J << Ixx, Ixy, Izx, Ixy, Iyy, Iyz, Izx, Iyz, Izz;
}

/** CSV 전체를 읽은 뒤 0열을 버려 ``[:, 1:]`` 와 동일한 ``Eigen::MatrixXd``로 만든다. */
bool load_recurdyn_csv(const std::filesystem::path& csv_path, Eigen::MatrixXd& out)
{
    std::ifstream in(csv_path);
    if (!in) {
        return false;
    }
    std::vector<std::vector<scalar>> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<scalar> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            row.push_back(static_cast<scalar>(std::stod(cell)));
        }
        if (!row.empty()) {
            rows.push_back(std::move(row));
        }
    }
    if (rows.empty() || rows[0].size() < 2) {
        return false;
    }
    const int ncols = static_cast<int>(rows[0].size()) - 1;
    out.resize(static_cast<Eigen::Index>(rows.size()), ncols);
    for (Eigen::Index r = 0; r < out.rows(); ++r) {
        if (static_cast<size_t>(r) >= rows.size() || rows[static_cast<size_t>(r)].size() != rows[0].size()) {
            return false;
        }
        for (int c = 0; c < ncols; ++c) {
            out(r, c) = rows[static_cast<size_t>(r)][static_cast<size_t>(c + 1)];
        }
    }
    return true;
}

}  // namespace

ControlMain::ControlMain()
{
    g = -9.80665;
}

ControlMain::~ControlMain()
{
    close_log();
}

void ControlMain::open_log(const std::string& path)
{
    close_log();
    fp.open(path, std::ios::out | std::ios::trunc);
    if (!fp) {
        std::cerr << "open_log failed: " << path << '\n';
        return;
    }

    fp << "t,"
       << "ee_x,ee_y,ee_z,ee_roll,ee_pitch,ee_yaw,"
       << "ee_vx,ee_vy,ee_vz,ee_wx,ee_wy,ee_wz,"
       << "ee_ax,ee_ay,ee_az,ee_awx,ee_awy,ee_awz,";
    for (int i = 0; i < NB; ++i) fp << "q_act_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "dq_act_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "ddq_act_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "q_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "dq_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "ddq_" << (i + 1) << ",";
    fp << "des_x,des_y,des_z,"
       << "err_x,err_y,err_z,following_error,";
    for (int i = 0; i < NB; ++i) fp << "des_q_" << (i + 1) << ",";
    for (int i = 0; i < NB; ++i) fp << "err_q_" << (i + 1) << ",";
    fp << "following_error_q,"
       << "des_roll,des_pitch,des_yaw,"
       << "err_roll,err_pitch,err_yaw,following_error_ori\n";
}

void ControlMain::close_log()
{
    if (fp.is_open()) {
        fp.close();
    }
}

void ControlMain::run()
{
    h = 0.0001;
    t_c = 0;
    t_e = 2;

    read_data();

    // Python: Path(main.py).parent / "../recurdyn/rec_data_path.csv" → analysis/recurdyn/...
    // __FILE__ = analysis/cpp/src/controlmain.cpp → parent×3 = analysis/
    const std::filesystem::path csv_path =
        std::filesystem::weakly_canonical(
            std::filesystem::path(__FILE__).parent_path().parent_path().parent_path())
        / "recurdyn"
        / "rec_data_torque.csv";
    if (!load_recurdyn_csv(csv_path, rec_data)) {
        std::cerr << "load rec_data.csv failed: " << csv_path << '\n';
    }

    // std::cout << rec_data << std::endl;

    for(int i = 0; i < 4; i++){
        body[i].qi = rec_data(0, 31 + i);
        body[i].dqi = rec_data(0, 35 + i);
    }

    for (int i = 0; i < NB; ++i) {
        Y(i) = body[i].qi;
        Y(4 + i) = body[i].dqi;
    }

    open_log("cpp_data_torque.csv");

    vec8 Y0, k1, k2, k3, k4;

    while(t_e > t_c){
        log_des_pos = vec3::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_err_pos = vec3::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_following_error = std::numeric_limits<scalar>::quiet_NaN();
        log_des_q = vec4::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_err_q = vec4::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_following_error_q = std::numeric_limits<scalar>::quiet_NaN();
        log_des_rpy = vec3::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_err_rpy = vec3::Constant(std::numeric_limits<scalar>::quiet_NaN());
        log_following_error_ori = std::numeric_limits<scalar>::quiet_NaN();

        Y0 = Y;
        k1 = analysis(Y0);
        k2 = analysis(Y0 + (h/2.0)*k1);
        k3 = analysis(Y0 + (h/2.0)*k2);
        k4 = analysis(Y0 + h*k3);
        Y = Y0 + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);

        analysis();

        data_save();

        t_c += h;
        index++;
    }

    close_log();
}

void ControlMain::run_vsd(){
    h = 0.001;
    t_c = 0;
    t_e = 1.5;

    read_data();

        // Python: Path(main.py).parent / "../recurdyn/rec_data_path.csv" → analysis/recurdyn/...
    // __FILE__ = analysis/cpp/src/controlmain.cpp → parent×3 = analysis/
    const std::filesystem::path csv_path =
        std::filesystem::weakly_canonical(
            std::filesystem::path(__FILE__).parent_path().parent_path().parent_path())
        / "recurdyn"
        / "rec_data_path2.csv";
    if (!load_recurdyn_csv(csv_path, rec_data)) {
        std::cerr << "load rec_data.csv failed: " << csv_path << '\n';
    }

    // double wp_t[7] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    // double wp_x[7] = {-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35};
    // double wp_y[7] = {0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15};
    // double wp_z[7] = {-0.2, -0.2, -0.2, -0.2, 0.13, 0.13, -0.2};

    // double wp_t[8] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    // double wp_x[8] = {0, -0.25, -0.35, -0.18, 0.18, 0.35, 0.25, 0};
    // double wp_y[8] = {0.33716302, -0.28, 0.15, 0.37, 0.37, 0.15, -0.28, 0.33716302};
    // double wp_z[8] = {0.2987143, 0, 0, 0.13, 0.13, 0, 0, 0.2987143};

    double wp_t[4] = {0.0, 0.5, 1.0, 1.5};
    double wp_x[4] = {-0.35, -0.25, 0.25, 0.35};
    double wp_y[4] = {0.15, -0.28, -0.28, 0.15};
    double wp_z[4] = {0, 0, 0, 0};

    std::vector< std::array<double, 3> > path_x, path_y, path_z;
    path_x = path_build(wp_t, wp_x, 4, 0.0, h, true);
    path_y = path_build(wp_t, wp_y, 4, 0.0, h, true);
    path_z = path_build(wp_t, wp_z, 4, 0.0, h, true);

    std::cout << "path len x/y/z : " << path_x.size() << ", " << path_y.size() << ", " << path_z.size() << std::endl;

    double q_init[4] = {-2.7367009, 1.0880061, 1.1749032, -0.87868275};
    for(int i = 0; i < 4; i++){
        body[i].qi = q_init[i];
    }

    open_log("cpp_data_vsd.csv");

    vec3 des_pos, err_pos;
    scalar des_roll, des_pitch, err_roll, err_pitch;
    Eigen::Matrix<scalar, 5, 1> err, Ke, Kv;
    Eigen::Matrix<scalar, 5, 1> des_vel, ev;
    // Further tuned defaults for current path/initial posture setup.
    double Ks[5] = {218279.026146, 230800.488552, 159168.050363, 2433.229196, 16124.352342};
    double Kd[5] = {1109.348611, 1707.380149, 5502.23545, 16.731615, 7.095061};
    const double ks_pos_scale = env_or_default("VSD_KS_POS_SCALE", 1.0);
    const double kd_pos_scale = env_or_default("VSD_KD_POS_SCALE", 1.0);
    const double ks_ori_scale = env_or_default("VSD_KS_ORI_SCALE", 1.0);
    const double kd_ori_scale = env_or_default("VSD_KD_ORI_SCALE", 1.0);
    for (int i = 0; i < 3; ++i) {
        Ks[i] *= ks_pos_scale;
        Kd[i] *= kd_pos_scale;
    }
    for (int i = 3; i < 5; ++i) {
        Ks[i] *= ks_ori_scale;
        Kd[i] *= kd_ori_scale;
    }
    // Per-axis absolute override (priority over scale env): x/y/z/roll/pitch
    Ks[0] = env_or_default("VSD_KS_X", Ks[0]);
    Ks[1] = env_or_default("VSD_KS_Y", Ks[1]);
    Ks[2] = env_or_default("VSD_KS_Z", Ks[2]);
    Ks[3] = env_or_default("VSD_KS_ROLL", Ks[3]);
    Ks[4] = env_or_default("VSD_KS_PITCH", Ks[4]);
    Kd[0] = env_or_default("VSD_KD_X", Kd[0]);
    Kd[1] = env_or_default("VSD_KD_Y", Kd[1]);
    Kd[2] = env_or_default("VSD_KD_Z", Kd[2]);
    Kd[3] = env_or_default("VSD_KD_ROLL", Kd[3]);
    Kd[4] = env_or_default("VSD_KD_PITCH", Kd[4]);
    vec4 tau;
    double fe_pos_sum = 0.0, fe_q_sum = 0.0, fe_ori_sum = 0.0;
    int fe_pos_count = 0, fe_q_count = 0, fe_ori_count = 0;
    double fe_pos_peak = 0.0, fe_q_peak = 0.0, fe_ori_peak = 0.0;
    while(t_c < t_e){
        des_pos = vec3(path_x[index][0], path_y[index][0], path_z[index][0]);
        des_roll = -M_PI_2;
        des_pitch = 0.0;
        des_vel << path_x[index][1], path_y[index][1], path_z[index][1], scalar(0), scalar(0);

        position_calculation();
        velocity_calculation();

        err_pos = des_pos - body[3].re;
        err_roll = wrap_to_pi(des_roll - body[3].rpy(0));
        err_pitch = wrap_to_pi(des_pitch - body[3].rpy(1));
        err << err_pos, err_roll, err_pitch;

        ev[0] = des_vel[0] - body[3].dre[0];
        ev[1] = des_vel[1] - body[3].dre[1];
        ev[2] = des_vel[2] - body[3].dre[2];
        ev[3] = des_vel[3] - body[3].wi[0];
        ev[4] = des_vel[4] - body[3].wi[1];
        J = jacobian_calculation();
        for(int i = 0; i < 5; i++){
            Ke[i] = Ks[i]*err[i];
            Kv[i] = Kd[i]*ev[i];
        }

        // J: 5×4 (작업공간 ← 관절), Ke+Kd: 5×1 → 관절 공간 토크 τ_joint = J^T * (K e + …)
        tau = J.transpose() * (Ke + Kv);
        const vec4 tau_g = joint_gravity_torque();
        // mass_force_calculation()이 Ti_tau = tau_body / gear 로 덮어쓰므로,
        // EQM에 들어갈 관절 일반화력이 (τ_vsd + τ_g)가 되려면 tau_body = (τ_vsd + τ_g) * gear.
        for(int i = 0; i < 4; i++){
            body[i].tau = (tau[i] + tau_g(i)) * body[i].gear;
        }

        for (int i = 0; i < NB; ++i) {
            Y(i) = body[i].qi;
            Y(4 + i) = body[i].dqi;
        }

        vec8 Y0, k1, k2, k3, k4;

        Y0 = Y;
        k1 = analysis(Y0);
        k2 = analysis(Y0 + (h/2.0)*k1);
        k3 = analysis(Y0 + (h/2.0)*k2);
        k4 = analysis(Y0 + h*k3);
        Y = Y0 + (h/6.0)*(k1 + 2*k2 + 2*k3 + k4);

        analysis();

        // Match logged error with the same EE state that is being saved.
        err_pos = des_pos - body[3].re;
        err_roll = wrap_to_pi(des_roll - body[3].rpy(0));
        err_pitch = wrap_to_pi(des_pitch - body[3].rpy(1));
        log_des_pos = des_pos;
        log_err_pos = err_pos;
        log_following_error = err_pos.norm();
        if (index < rec_data.rows() && rec_data.cols() >= 35) {
            for (int i = 0; i < NB; ++i) {
                log_des_q(i) = rec_data(index, 31 + i);
                log_err_q(i) = log_des_q(i) - body[i].qi;
            }
            log_following_error_q = log_err_q.norm();
        } else {
            log_des_q = vec4::Constant(std::numeric_limits<scalar>::quiet_NaN());
            log_err_q = vec4::Constant(std::numeric_limits<scalar>::quiet_NaN());
            log_following_error_q = std::numeric_limits<scalar>::quiet_NaN();
        }
        log_des_rpy << des_roll, des_pitch, std::numeric_limits<scalar>::quiet_NaN();
        log_err_rpy << err_roll, err_pitch, std::numeric_limits<scalar>::quiet_NaN();
        log_following_error_ori = std::hypot(err_roll, err_pitch);

        const bool invalid_state =
            !std::isfinite(log_following_error) ||
            !std::isfinite(log_following_error_ori) ||
            !std::isfinite(body[0].qi) || !std::isfinite(body[1].qi) ||
            !std::isfinite(body[2].qi) || !std::isfinite(body[3].qi);
        if (invalid_state) {
            std::cerr << "[WARN] Invalid numeric state detected at t=" << t_c
                      << ". Stopping simulation early.\n";
            break;
        }

        if (std::isfinite(log_following_error)) {
            fe_pos_sum += log_following_error;
            fe_pos_count++;
            if (log_following_error > fe_pos_peak) fe_pos_peak = log_following_error;
        }
        if (std::isfinite(log_following_error_q)) {
            fe_q_sum += log_following_error_q;
            fe_q_count++;
            if (log_following_error_q > fe_q_peak) fe_q_peak = log_following_error_q;
        }
        if (std::isfinite(log_following_error_ori)) {
            fe_ori_sum += log_following_error_ori;
            fe_ori_count++;
            if (log_following_error_ori > fe_ori_peak) fe_ori_peak = log_following_error_ori;
        }

        data_save();

        t_c += h;
        index++;
    }

    std::cout << "[SUMMARY] "
              << "fe_pos(avg=" << (fe_pos_count > 0 ? fe_pos_sum / fe_pos_count : 0.0)
              << ", peak=" << fe_pos_peak << ")";
    if (fe_q_count > 0) {
        std::cout << ", fe_q(avg=" << (fe_q_sum / fe_q_count)
                  << ", peak=" << fe_q_peak << ")";
    }
    if (fe_ori_count > 0) {
        std::cout << ", fe_ori(avg=" << (fe_ori_sum / fe_ori_count)
                  << ", peak=" << fe_ori_peak << ")";
    }
    std::cout << std::endl;

    close_log();
}

void ControlMain::run_ik(){
    h = 0.001;
    t_c = 0;
    t_e = 3.5;

    read_data();

    // Python: Path(main.py).parent / "../recurdyn/rec_data_path.csv" → analysis/recurdyn/...
    // __FILE__ = analysis/cpp/src/controlmain.cpp → parent×3 = analysis/
    // const std::filesystem::path csv_path =
    //     std::filesystem::weakly_canonical(
    //         std::filesystem::path(__FILE__).parent_path().parent_path().parent_path())
    //     / "recurdyn"
    //     / "rec_data_path.csv";
    // if (!load_recurdyn_csv(csv_path, rec_data)) {
    //     std::cerr << "load rec_data.csv failed: " << csv_path << '\n';
    // }

    double wp_t[8] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    double wp_x[8] = {0, -0.25, -0.35, -0.18, 0.18, 0.35, 0.25, 0};
    double wp_y[8] = {0.33716302, -0.28, 0.15, 0.37, 0.37, 0.15, -0.28, 0.33716302};
    double wp_z[8] = {0.2987143, 0, 0, 0.13, 0.13, 0, 0, 0.2987143};

    std::vector< std::array<double, 3> > path_x, path_y, path_z;
    path_x = path_build(wp_t, wp_x, 8, 0.0, h, true);
    path_y = path_build(wp_t, wp_y, 8, 0.0, h, true);
    path_z = path_build(wp_t, wp_z, 8, 0.0, h, true);

    std::cout << "path len x/y/z : " << path_x.size() << ", " << path_y.size() << ", " << path_z.size() << std::endl;

    double q_init[NB] = {-90, 30, 45, -105};
    for(int i = 0; i < NB; i++){
        body[i].qi = q_init[i]*M_PI/180.0;
    }

    open_log("cpp_data_path.csv");

    std::cout << "initial q : ";
    for(int i = 0; i < NB; i++){
        std::cout << body[i].qi;
        std::cout << (i == (NB-1) ? "\n" : ", ");
    }
    std::cout << "initial pos_d : " << vec3(path_x[index][0], path_y[index][0], path_z[index][0]).transpose() << std::endl;

    vec3 des_pos, des_acc, err_pos;
    scalar des_roll, des_pitch, err_roll, err_pitch;
    Eigen::Matrix<scalar, 5, 1> err;
    Eigen::Matrix<scalar, 5, 1> des_vel;
    double err_tol = 1e-3;
    double damping = 1e-7;
    double alpha = 0.6;
    using mat5 = Eigen::Matrix<scalar, 5, 5>;
    vec4 delta_q, q_dot;
    double fe_pos_sum = 0.0, fe_q_sum = 0.0, fe_ori_sum = 0.0;
    int fe_pos_count = 0, fe_q_count = 0, fe_ori_count = 0;
    double fe_pos_peak = 0.0, fe_q_peak = 0.0, fe_ori_peak = 0.0;
    while(t_e > t_c){
        des_pos = vec3(path_x[index][0], path_y[index][0], path_z[index][0]);
        des_roll = -M_PI_2;
        des_pitch = 0.0;
        des_vel << path_x[index][1], path_y[index][1], path_z[index][1], scalar(0), scalar(0);

        position_calculation();

        err_pos = des_pos - body[3].re;
        err_roll = wrap_to_pi(des_roll - body[3].rpy(0));
        err_pitch = wrap_to_pi(des_pitch - body[3].rpy(1));
        err << err_pos, err_roll, err_pitch;

        int iter_count = 0;
        while(true){
            J = jacobian_calculation();

            JJT_reg = J * J.transpose() + (damping * damping) * mat5::Identity();
            Eigen::PartialPivLU<mat5> lu(JJT_reg);
            delta_q = alpha * J.transpose() * lu.solve(err);
            q_dot = J.transpose() * lu.solve(des_vel);

            for(int i = 0; i < NB; i++){
                body[i].qi += delta_q(i);
                body[i].dqi = q_dot(i);
            }

            position_calculation();
            velocity_calculation();

            err_pos = des_pos - body[3].re;
            err_roll = wrap_to_pi(des_roll - body[3].rpy(0));
            err_pitch = wrap_to_pi(des_pitch - body[3].rpy(1));
            err << err_pos, err_roll, err_pitch;

            iter_count++;
            if(iter_count > 100){
                std::cout << "IK failed to converge" << std::endl;
                break;
            }
            if(err.norm() < err_tol) break;
        }

        // Save the post-IK achieved position error for this step.
        log_des_pos = des_pos;
        log_err_pos = err_pos;
        log_following_error = err_pos.norm();
        for (int i = 0; i < NB; ++i) {
            // In IK mode, the converged joint state is the desired joint target.
            log_des_q(i) = body[i].qi;
            log_err_q(i) = scalar(0);
        }
        log_following_error_q = scalar(0);
        log_des_rpy << des_roll, des_pitch, std::numeric_limits<scalar>::quiet_NaN();
        log_err_rpy << err_roll, err_pitch, std::numeric_limits<scalar>::quiet_NaN();
        log_following_error_ori = std::hypot(err_roll, err_pitch);

        if (std::isfinite(log_following_error)) {
            fe_pos_sum += log_following_error;
            fe_pos_count++;
            if (log_following_error > fe_pos_peak) fe_pos_peak = log_following_error;
        }
        if (std::isfinite(log_following_error_q)) {
            fe_q_sum += log_following_error_q;
            fe_q_count++;
            if (log_following_error_q > fe_q_peak) fe_q_peak = log_following_error_q;
        }
        if (std::isfinite(log_following_error_ori)) {
            fe_ori_sum += log_following_error_ori;
            fe_ori_count++;
            if (log_following_error_ori > fe_ori_peak) fe_ori_peak = log_following_error_ori;
        }

        data_save();

        t_c += h;
        index++;
    }

    std::cout << "[SUMMARY] "
              << "fe_pos(avg=" << (fe_pos_count > 0 ? fe_pos_sum / fe_pos_count : 0.0)
              << ", peak=" << fe_pos_peak << ")";
    if (fe_q_count > 0) {
        std::cout << ", fe_q(avg=" << (fe_q_sum / fe_q_count)
                  << ", peak=" << fe_q_peak << ")";
    }
    if (fe_ori_count > 0) {
        std::cout << ", fe_ori(avg=" << (fe_ori_sum / fe_ori_count)
                  << ", peak=" << fe_ori_peak << ")";
    }
    std::cout << std::endl;

    close_log();
}

void ControlMain::read_data()
{
    base.Ai.setIdentity();
    base.ri.setZero();
    base.wi.setZero();
    base.dri.setZero();
    base.wit = skew(base.wi);
    base.Yih.setZero();
    base.dYih.setZero();

    // body1
    body[0].qi = 0;
    body[0].qi_act = 0;
    body[0].gear = 32.0 / 60.0;
    body[0].dqi = 0;
    body[0].dqi_act = 0;
    body[0].ddqi = 0;
    body[0].ddqi_act = 0;
    body[0].sijp.setZero();
    body[0].Cij = euler_zxz(0, M_PI, 0);
    body[0].u_vec = vec3(0, 0, 1);
    body[0].rhoip = vec3(0.0026336, -1.68446e-05, -0.111117);
    body[0].Cii = euler_zxz(0, 0, 0);
    body[0].mi = 10.0123496865811;
    fill_inertia(body[0].Jip, 
        8.29601614566715e-002, -3.28089653994623e-004, 
        3.91199810914461e-002, 2.09012210735718e-006, 
        6.49458588226152e-002, -1.19428008908532e-004);
    body[0].tau = 0;

    // body2
    body[1].qi = 0;
    body[1].qi_act = 0;
    body[1].gear = 360.0 / 54.0;
    body[1].dqi = 0;
    body[1].dqi_act = 0;
    body[1].ddqi = 0;
    body[1].ddqi_act = 0;
    body[1].sijp = vec3(0, 0, -0.22);
    body[1].Cij = euler_zxz(0, M_PI_2, 0);
    body[1].u_vec = vec3(0, 0, 1);
    body[1].rhoip = vec3(6.90559e-05, -0.0851548, -0.00686211);
    body[1].Cii = euler_zxz(M_PI, M_PI_2, M_PI);
    body[1].mi = 10.4391437674567;
    fill_inertia(body[1].Jip, 
        7.80970193464117e-002, 3.53131298101708e-005, 
        2.55871855807303e-002, 5.47282289489732e-003, 
        7.45746466344453e-002, -8.81620777833472e-006);
    body[1].tau = 375;

    // body3
    body[2].qi = 0;
    body[2].qi_act = 0;
    body[2].gear = 360.0 / 108.0;
    body[2].dqi = 0;
    body[2].dqi_act = 0;
    body[2].ddqi = 0;
    body[2].ddqi_act = 0;
    body[2].sijp = vec3(0, -0.23, 0);
    body[2].Cij = euler_zxz(-M_PI_2, 0, 0);
    body[2].u_vec = vec3(0, 0, 1);
    body[2].rhoip = vec3(0.0969069, -3.20036e-05, -0.00548574);
    body[2].Cii = euler_zxz(-M_PI_2, M_PI_2, M_PI);
    body[2].mi = 10.3406497234359;
    fill_inertia(body[2].Jip, 
        7.92872354716476e-002, 8.26042768438978e-006, 
        2.27404242493157e-002, 4.31986606376804e-003, 
        7.81409250910383e-002, 5.43833216931251e-006);
    body[2].tau = 60;

    // body4
    body[3].qi = 0;
    body[3].qi_act = 0;
    body[3].gear = 360.0 / 108.0;
    body[3].dqi = 0;
    body[3].dqi_act = 0;
    body[3].ddqi = 0;
    body[3].ddqi_act = 0;
    body[3].sijp = vec3(0.23, 0, 0);
    body[3].Cij = euler_zxz(0, M_PI, 0);
    body[3].u_vec = vec3(0, 0, 1);
    body[3].rhoip = vec3(0.0675884, 0.00443192, 0.000679202);
    body[3].Cii = euler_zxz(-M_PI_2, M_PI_2, 0);
    body[3].mi = 7.01416597186014;
    fill_inertia(body[3].Jip, 
        3.93493190184971e-002, -9.26169916890393e-004, 
        1.11764686166838e-002, 2.04330936352529e-004, 
        4.10218257620852e-002, -8.43406506016272e-006);
    body[3].tau = -3;

    body[3].sep = vec3(0.18, 0, 0);
    body[3].Ce = euler_zxz(-M_PI_2, 0, 0);
}

void ControlMain::Y2qdq()
{
    for (int i = 0; i < NB; ++i) {
        body[i].qi = Y(i);
        body[i].dqi = Y(4 + i);
    }
}

vec8 ControlMain::dqddq2Yp()
{
    vec8 yp;
    for (int i = 0; i < NB; ++i) {
        yp(i) = body[i].dqi;
        yp(4 + i) = body[i].ddqi;
    }
    return yp;
}

void ControlMain::position_calculation()
{
    for (int i = 0; i < NB; ++i) {
        Body& b = body[i];
        const scalar c = std::cos(b.qi);
        const scalar s = std::sin(b.qi);
        b.Aijpp << c, -s, 0, s, c, 0, 0, 0, 1;
    }

    const Body* prev = &base;
    for (Body& b : body) {
        b.Ai = prev->Ai * b.Cij * b.Aijpp;
        b.sij = prev->Ai * b.sijp;
        b.ri = prev->ri + b.sij;
        b.Hi = prev->Ai * b.Cij * b.u_vec;
        prev = &b;
    }

    Body& ee = body[3];
    ee.se = ee.Ai * ee.sep;
    ee.re = ee.ri + ee.se;
    ee.Ae = ee.Ai * ee.Ce;
    ee.rpy = mat2rpy(ee.Ae);

    for (Body& b : body) {
        b.rhoi = b.Ai * b.rhoip;
        b.ric = b.ri + b.rhoi;
    }
}

void ControlMain::velocity_calculation()
{
    const Body* prev = &base;
    for (Body& b : body) {
        b.wi = prev->wi + b.Hi * b.dqi;
        b.wit = skew(b.wi);
        b.dri = prev->dri + prev->wit * b.sij;
        prev = &b;
    }

    body[3].dre = body[3].dri + body[3].wit * body[3].se;

    prev = &base;
    for (Body& b : body) {
        b.rit = skew(b.ri);
        b.Bi.head<3>() = b.rit * b.Hi;
        b.Bi.tail<3>() = b.Hi;
        b.drit = skew(b.dri);
        b.dric = b.dri + b.wit * b.rhoi;
        b.dHi = prev->wit * b.Hi;
        const vec3 di_top = b.drit * b.Hi + b.rit * b.dHi;
        b.Di.head<3>() = di_top * b.dqi;
        b.Di.tail<3>() = b.dHi * b.dqi;
        b.Yih = prev->Yih + b.Bi * b.dqi;
        prev = &b;
    }
}

void ControlMain::mass_force_calculation()
{
    for (Body& b : body) {
        b.Ai_Cii = b.Ai * b.Cii;
        b.Jic = b.Ai_Cii * b.Jip * b.Ai_Cii.transpose();
        b.rict = skew(b.ric);
        b.drict = skew(b.dric);
        b.fic = vec3(0, 0, b.mi * g);
        b.tic = vec3::Zero();
        const mat3& rict = b.rict;
        const scalar mi = b.mi;
        const mat3& Jic = b.Jic;
        const mat3 I3 = mat3::Identity();

        b.Mih.topLeftCorner<3, 3>() = mi * I3;
        b.Mih.topRightCorner<3, 3>() = -mi * rict;
        b.Mih.bottomLeftCorner<3, 3>() = mi * rict;
        b.Mih.bottomRightCorner<3, 3>() = Jic - mi * (rict * rict);

        const vec3 q_top = b.fic + mi * (b.drict * b.wi);
        const vec3 q_bot = b.tic + b.rict * b.fic
            + mi * (b.rict * b.drict * b.wi) - b.wit * b.Jic * b.wi;
        b.Qih.head<3>() = q_top;
        b.Qih.tail<3>() = q_bot;
        b.Ti_tau = b.tau / b.gear;
    }
}

vec4 ControlMain::EQM()
{
    body[3].Ki = body[3].Mih;
    body[2].Ki = body[2].Mih + body[3].Ki;
    body[1].Ki = body[1].Mih + body[2].Ki;
    body[0].Ki = body[0].Mih + body[1].Ki;

    body[3].Li = body[3].Qih;
    body[2].Li = body[2].Qih + body[3].Li - body[3].Ki * body[3].Di;
    body[1].Li = body[1].Qih + body[2].Li - body[2].Ki * body[2].Di;
    body[0].Li = body[0].Qih + body[1].Li - body[1].Ki * body[1].Di;

    scalar M11 = body[0].Bi.transpose() * body[0].Ki * body[0].Bi;
    scalar M12 = body[0].Bi.transpose() * body[1].Ki * body[1].Bi;
    scalar M13 = body[0].Bi.transpose() * body[2].Ki * body[2].Bi;
    scalar M14 = body[0].Bi.transpose() * body[3].Ki * body[3].Bi;

    scalar M21 = body[1].Bi.transpose() * body[1].Ki * body[0].Bi;
    scalar M22 = body[1].Bi.transpose() * body[1].Ki * body[1].Bi;
    scalar M23 = body[1].Bi.transpose() * body[2].Ki * body[2].Bi;
    scalar M24 = body[1].Bi.transpose() * body[3].Ki * body[3].Bi;

    scalar M31 = body[2].Bi.transpose() * body[2].Ki * body[0].Bi;
    scalar M32 = body[2].Bi.transpose() * body[2].Ki * body[1].Bi;
    scalar M33 = body[2].Bi.transpose() * body[2].Ki * body[2].Bi;
    scalar M34 = body[2].Bi.transpose() * body[3].Ki * body[3].Bi;

    scalar M41 = body[3].Bi.transpose() * body[3].Ki * body[0].Bi;
    scalar M42 = body[3].Bi.transpose() * body[3].Ki * body[1].Bi;
    scalar M43 = body[3].Bi.transpose() * body[3].Ki * body[2].Bi;
    scalar M44 = body[3].Bi.transpose() * body[3].Ki * body[3].Bi;

    mat4 M;
    M << M11, M12, M13, M14, 
        M21, M22, M23, M24, 
        M31, M32, M33, M34, 
        M41, M42, M43, M44;

    // mat4 M;
    for (int i = 0; i < NB; i++) {
        for (int j = 0; j < NB; j++) {
            if(i > j)
                M(i, j) = body[i].Bi.transpose() * body[i].Ki * body[j].Bi;
            else
                M(i, j) = body[i].Bi.transpose() * body[j].Ki * body[j].Bi;
        }       
    }

    const vec6& D0 = body[0].Di;
    const vec6 D01 = D0 + body[1].Di;
    const vec6 D012 = D01 + body[2].Di;
    const vec6 D0123 = D012 + body[3].Di;

    vec4 Q;
    Q(0) = body[0].Bi.dot(body[0].Li - body[0].Ki * D0);
    Q(1) = body[1].Bi.dot(body[1].Li - body[1].Ki * D01);
    Q(2) = body[2].Bi.dot(body[2].Li - body[2].Ki * D012);
    Q(3) = body[3].Bi.dot(body[3].Li - body[3].Ki * D0123);

    Q(0) += body[0].Ti_tau;
    Q(1) += body[1].Ti_tau;
    Q(2) += body[2].Ti_tau;
    Q(3) += body[3].Ti_tau;

    Eigen::PartialPivLU<mat4> lu(M);
    return lu.solve(Q);
}

void ControlMain::acceleration_calculation()
{
    const Body* prev = &base;
    for (Body& b : body) {
        b.dYih = prev->dYih + b.Bi * b.ddqi + b.Di;

        const mat3& rit = b.rit;
        b.Ti.setZero();
        b.Ti.topLeftCorner<3, 3>().setIdentity();
        b.Ti.topRightCorner<3, 3>() = -rit;
        b.Ti.bottomRightCorner<3, 3>().setIdentity();

        const mat3& drit = b.drit;
        b.dTi.setZero();
        b.dTi.topRightCorner<3, 3>() = -drit;

        b.dYib = b.Ti * b.dYih + b.dTi * b.Yih;

        b.ddri = b.dYib.head<3>();
        b.dwi = b.dYib.tail<3>();

        b.dwit = skew(b.dwi);
        b.ddric = b.ddri + b.dwit * b.rhoi + b.wit * b.wit * b.rhoi;
        prev = &b;
    }

    body[3].ddre = body[3].ddri + body[3].dwit * body[3].se + body[3].wit * body[3].wit * body[3].se;

    for (Body& b : body) {
        b.ddqi_act = b.ddqi / b.gear;
    }
}

Eigen::Matrix<scalar, 5, 4> ControlMain::jacobian_calculation()
{
    A01pp_q1 << -std::sin(body[0].qi), -std::cos(body[0].qi), scalar(0), std::cos(body[0].qi),
        -std::sin(body[0].qi), scalar(0), scalar(0), scalar(0), scalar(0);
    A12pp_q2 << -std::sin(body[1].qi), -std::cos(body[1].qi), scalar(0), std::cos(body[1].qi),
        -std::sin(body[1].qi), scalar(0), scalar(0), scalar(0), scalar(0);
    A23pp_q3 << -std::sin(body[2].qi), -std::cos(body[2].qi), scalar(0), std::cos(body[2].qi),
        -std::sin(body[2].qi), scalar(0), scalar(0), scalar(0), scalar(0);
    A34pp_q4 << -std::sin(body[3].qi), -std::cos(body[3].qi), scalar(0), std::cos(body[3].qi),
        -std::sin(body[3].qi), scalar(0), scalar(0), scalar(0), scalar(0);

    A1_q1 = base.Ai*body[0].Cij*A01pp_q1;
    A2_q1 = A1_q1*body[1].Cij*body[1].Aijpp;
    A3_q1 = A2_q1*body[2].Cij*body[2].Aijpp;
    A4_q1 = A3_q1*body[3].Cij*body[3].Aijpp;

    A2_q2 = body[0].Ai*body[1].Cij*A12pp_q2;
    A3_q2 = A2_q2*body[2].Cij*body[2].Aijpp;
    A4_q2 = A3_q2*body[3].Cij*body[3].Aijpp;

    A3_q3 = body[1].Ai*body[2].Cij*A23pp_q3;
    A4_q3 = A3_q3*body[3].Cij*body[3].Aijpp;

    A4_q4 = body[2].Ai*body[3].Cij*A34pp_q4;

    Ae_q1 = A4_q1*body[3].Ce;
    Ae_q2 = A4_q2*body[3].Ce;
    Ae_q3 = A4_q3*body[3].Ce;
    Ae_q4 = A4_q4*body[3].Ce;

    jac_pos.col(0) = body[0].Hi.cross(body[3].re - body[0].ri);
    jac_pos.col(1) = body[1].Hi.cross(body[3].re - body[1].ri);
    jac_pos.col(2) = body[2].Hi.cross(body[3].re - body[2].ri);
    jac_pos.col(3) = body[3].Hi.cross(body[3].re - body[3].ri);

    dAe_dq[0] = Ae_q1;
    dAe_dq[1] = Ae_q2;
    dAe_dq[2] = Ae_q3;
    dAe_dq[3] = Ae_q4;

    jac_rp = roll_pitch_jacobian_wrt_q(body[3].Ae, dAe_dq);

    Eigen::Matrix<scalar, 5, 4> jac;
    jac.topRows<3>() = jac_pos;
    jac.bottomRows<2>() = jac_rp;

    return jac;
}

vec8 ControlMain::analysis(const vec8& Y_in)
{
    Y = Y_in;
    Y2qdq();
    position_calculation();
    velocity_calculation();
    mass_force_calculation();
    ddq = EQM();
    for (int i = 0; i < NB; ++i) {
        body[i].ddqi = ddq(i);
    }
    acceleration_calculation();
    return dqddq2Yp();
}

vec8 ControlMain::analysis()
{
    Y2qdq();
    position_calculation();
    velocity_calculation();
    mass_force_calculation();
    ddq = EQM();
    for (int i = 0; i < NB; ++i) {
        body[i].ddqi = ddq(i);
    }
    acceleration_calculation();
    return dqddq2Yp();
}

scalar ControlMain::gravity_potential_energy() const
{
    scalar U = 0;
    for (int i = 0; i < 4; ++i) {
        U += body[i].mi * (-g) * body[i].ric(2);
    }
    return U;
}

vec4 ControlMain::joint_gravity_torque()
{
    constexpr scalar eps = 1e-5;
    scalar qsave[4];
    for (int i = 0; i < 4; ++i) {
        qsave[i] = body[i].qi;
    }

    vec4 tau_g;
    for (int j = 0; j < 4; ++j) {
        body[j].qi = qsave[j] + eps;
        position_calculation();
        const scalar up = gravity_potential_energy();
        body[j].qi = qsave[j] - eps;
        position_calculation();
        const scalar um = gravity_potential_energy();
        tau_g(j) = (up - um) / (2 * eps);
        body[j].qi = qsave[j];
    }
    for (int i = 0; i < 4; ++i) {
        body[i].qi = qsave[i];
    }
    position_calculation();
    return tau_g;
}

void ControlMain::data_save()
{
    if (!fp.is_open()) {
        return;
    }
    for (Body& b : body) {
        b.qi_act = b.qi / b.gear;
        b.dqi_act = b.dqi / b.gear;
        b.ddqi_act = b.ddqi / b.gear;
    }
    const Body& ee = body[3];
    fp << t_c << ",";
    fp << ee.re(0) << "," << ee.re(1) << "," << ee.re(2) << "," << ee.rpy(0) << "," << ee.rpy(1) << "," << ee.rpy(2) << ",";
    fp << ee.dre(0) << "," << ee.dre(1) << "," << ee.dre(2) << "," << ee.wi(0) << "," << ee.wi(1) << "," << ee.wi(2) << ",";
    fp << ee.ddre(0) << "," << ee.ddre(1) << "," << ee.ddre(2) << "," << ee.dwi(0) << "," << ee.dwi(1) << "," << ee.dwi(2) << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].qi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].dqi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].ddqi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].qi << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].dqi << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].ddqi << ",";
    fp << log_des_pos(0) << "," << log_des_pos(1) << "," << log_des_pos(2) << ",";
    fp << log_err_pos(0) << "," << log_err_pos(1) << "," << log_err_pos(2) << ",";
    fp << log_following_error << ",";
    for (int i = 0; i < NB; ++i) fp << log_des_q(i) << ",";
    for (int i = 0; i < NB; ++i) fp << log_err_q(i) << ",";
    fp << log_following_error_q << ",";
    fp << log_des_rpy(0) << "," << log_des_rpy(1) << "," << log_des_rpy(2) << ",";
    fp << log_err_rpy(0) << "," << log_err_rpy(1) << "," << log_err_rpy(2) << ",";
    fp << log_following_error_ori << '\n';
}

std::vector< std::array<double, 3> > ControlMain::path_build(double* wp_t, double* wp_x, int wp_n, double ta, double h, bool full_quintic){
    std::vector< std::array<double, 3> > path_wp, path_full;

    for(int i = 1; i < wp_n; i++){
        path_wp = path_generation(wp_x[i - 1], wp_x[i], wp_t[i] - wp_t[i - 1], ta, h, full_quintic);
        if(i < wp_n - 1 && !path_wp.empty()) path_wp.pop_back();
        path_full.insert(path_full.end(), path_wp.begin(), path_wp.end());
    }

    return path_full;
}
