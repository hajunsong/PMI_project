#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/LU>

#include "controlmain.h"

namespace {

constexpr int NB = 4;

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
    }
}

void ControlMain::close_log()
{
    if (fp.is_open()) {
        fp.close();
    }
}

void ControlMain::run()
{
    h = 0.001;
    t_c = 0;
    t_e = 3;

    read_data();

    for (int i = 0; i < NB; ++i) {
        Y(i) = body[i].qi;
        Y(4 + i) = body[i].dqi;
    }

    // Python: Path(main.py).parent / "../recurdyn/rec_data_path.csv" → analysis/recurdyn/...
    // __FILE__ = analysis/cpp/src/controlmain.cpp → parent×3 = analysis/
    const std::filesystem::path csv_path =
        std::filesystem::weakly_canonical(
            std::filesystem::path(__FILE__).parent_path().parent_path().parent_path())
        / "recurdyn"
        / "rec_data_path.csv";
    if (!load_recurdyn_csv(csv_path, rec_data)) {
        std::cerr << "load rec_data_path.csv failed: " << csv_path << '\n';
    }

    // std::cout << rec_data << std::endl;

    open_log("data/cpp_data.csv");

    while(t_e > t_c){
        for(int i = 0; i < NB; i++){
            body[i].qi = rec_data(index, 31 + i);
            body[i].dqi = rec_data(index, 35 + i);
        }

        position_calculation();
        velocity_calculation();

        data_save();

        std::cout << "t_c : " << t_c << std::endl;
        t_c += h;
        index++;
    }

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
    fill_inertia(body[0].Jip, 8.29601614566715e-002, -3.28089653994623e-004, 3.91199810914461e-002,
        2.09012210735718e-006, 6.49458588226152e-002, -1.19428008908532e-004);
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
    fill_inertia(body[1].Jip, 7.80970193464117e-002, 3.53131298101708e-005, 2.55871855807303e-002,
        5.47282289489732e-003, 7.45746466344453e-002, -8.81620777833472e-006);
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
    fill_inertia(body[2].Jip, 7.92872354716476e-002, 8.26042768438978e-006, 2.27404242493157e-002,
        4.31986606376804e-003, 7.81409250910383e-002, 5.43833216931251e-006);
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
    fill_inertia(body[3].Jip, 3.93493190184971e-002, -9.26169916890393e-004, 1.11764686166838e-002,
        2.04330936352529e-004, 4.10218257620852e-002, -8.43406506016272e-006);
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

    mat4 M;
    for (int i = 0; i < NB; ++i) {
        for (int j = 0; j < NB; ++j) {
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
    fp << ee.dre(0) << "," << ee.dre(1) << "," << ee.dre(2) << "," << ee.wi(0) << "," << ee.wi(1) << "," << ee.wi(2) << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].qi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].dqi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].ddqi_act << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].qi << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].dqi << ",";
    for (int i = 0; i < NB; ++i) fp << body[i].ddqi << ",";
    fp << '\n';
}
