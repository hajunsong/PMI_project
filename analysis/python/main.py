#!/usr/bin/env python3

from pathlib import Path

import numpy as np

from read_data import read_data as apply_model_read_data
from utils import mat2rpy, roll_pitch_jacobian_wrt_q, skew, wrap_to_pi
from path_generation import path_generation


def _path_build_full_quintic(wp_t, wp_vals, h):
    """
    C++ ``ControlMain::path_build`` 와 동일: 웨이포인트 사이를 각각 단일 5차(휴지–휴지)로 잇고,
    내부 경계는 끝 샘플을 제거해 이어 붙인다. ``(N, 3)`` = [pos, vel, acc].
    """
    wp_t = np.asarray(wp_t, dtype=float)
    wp_vals = np.asarray(wp_vals, dtype=float)
    wp_n = len(wp_t)
    parts = []
    for i in range(1, wp_n):
        seg = path_generation(
            float(wp_vals[i - 1]),
            float(wp_vals[i]),
            float(wp_t[i] - wp_t[i - 1]),
            0.0,
            h,
            full_quintic=True,
        )
        if i < wp_n - 1 and len(seg) > 0:
            seg = seg[:-1]
        parts.append(seg)
    return np.vstack(parts)


class Body:
    def __init__(self):
        self.qi = 0
        self.qi_act = 0
        self.gear = 0
        self.dqi = 0
        self.dqi_act = 0
        self.ddqi = 0
        self.ddqi_act = 0
        self.sijp = np.array([0, 0, 0])
        self.Cij = np.eye(3)
        self.u_vec = np.array([0, 0, 1])
        self.rhoip = np.array([0, 0, 0])
        self.Cii = np.eye(3)
        self.mi = 0
        self.Jip = np.eye(3)
        self.tau = 0
        self.sep = np.array([0, 0, 0])
        self.Ce = np.eye(3)
        self.Ai = np.eye(3)
        self.ri = np.array([0, 0, 0])
        self.wi = np.array([0, 0, 0])
        self.dri = np.array([0, 0, 0])
        self.Yih = np.array([0, 0, 0, 0, 0, 0])
        self.dYih = np.array([0, 0, 0, 0, 0, 0])

class ControlMain:
    def __init__(self):
        self.t_c = 0
        self.h = 0
        self.g = 0
        self.t_e = 0
        self.index = 0
        self.fp = None
        self.base = Body()
        self.body = [Body() for _ in range(4)]
        self.rec_data = np.empty((0, 0))
        self.read_data()

    def read_data(self) -> None:
        apply_model_read_data(self)

    def define_Y_vector(self):
        return np.array(
            [
                self.body[0].qi,
                self.body[1].qi,
                self.body[2].qi,
                self.body[3].qi,
                self.body[0].dqi,
                self.body[1].dqi,
                self.body[2].dqi,
                self.body[3].dqi,
            ],
            dtype=float,
        )

    def Y2qdq(self):
        for i in range(4):
            self.body[i].qi = float(self.Y[i])
            self.body[i].dqi = float(self.Y[4 + i])

    def dqddq2Yp(self):
        return np.array(
            [
                self.body[0].dqi,
                self.body[1].dqi,
                self.body[2].dqi,
                self.body[3].dqi,
                self.body[0].ddqi,
                self.body[1].ddqi,
                self.body[2].ddqi,
                self.body[3].ddqi,
            ],
            dtype=float,
        )

    def data_save(self):
        for i in range(4):
            self.body[i].qi_act = self.body[i].qi/self.body[i].gear
            self.body[i].dqi_act = self.body[i].dqi/self.body[i].gear
            self.body[i].ddqi_act = self.body[i].ddqi/self.body[i].gear

        self.fp.write(f'{self.t_c},')
        self.fp.write(f'{self.body[3].re[0]}, {self.body[3].re[1]}, {self.body[3].re[2]}, {self.body[3].rpy[0]}, {self.body[3].rpy[1]}, {self.body[3].rpy[2]},')
        self.fp.write(f'{self.body[3].dre[0]}, {self.body[3].dre[1]}, {self.body[3].dre[2]}, {self.body[3].wi[0]}, {self.body[3].wi[1]}, {self.body[3].wi[2]},')
        self.fp.write(f'{self.body[3].dre[0]}, {self.body[3].dre[1]}, {self.body[3].dre[2]}, {self.body[3].wi[0]}, {self.body[3].wi[1]}, {self.body[3].wi[2]},')
        self.fp.write(f'{self.body[0].qi_act}, {self.body[1].qi_act}, {self.body[2].qi_act}, {self.body[3].qi_act},')
        self.fp.write(f'{self.body[0].dqi_act}, {self.body[1].dqi_act}, {self.body[2].dqi_act}, {self.body[3].dqi_act},')
        self.fp.write(f'{self.body[0].ddqi_act}, {self.body[1].ddqi_act}, {self.body[2].ddqi_act}, {self.body[3].ddqi_act},')
        self.fp.write(f'{self.body[0].qi}, {self.body[1].qi}, {self.body[2].qi}, {self.body[3].qi},')
        self.fp.write(f'{self.body[0].dqi}, {self.body[1].dqi}, {self.body[2].dqi}, {self.body[3].dqi},')
        self.fp.write(f'{self.body[0].ddqi}, {self.body[1].ddqi}, {self.body[2].ddqi}, {self.body[3].ddqi},')
        self.fp.write('\n')

    def position_calculation(self):
        for i in range(4):
            self.body[i].Aijpp = np.array([[np.cos(self.body[i].qi), -np.sin(self.body[i].qi), 0], [np.sin(self.body[i].qi), np.cos(self.body[i].qi), 0], [0, 0, 1]])
        
        prev = self.base
        for i in range(4):
            self.body[i].Ai = prev.Ai @ self.body[i].Cij @ self.body[i].Aijpp
            self.body[i].sij = prev.Ai @ self.body[i].sijp
            self.body[i].ri = prev.ri + self.body[i].sij
            self.body[i].Hi = prev.Ai @ self.body[i].Cij @ self.body[i].u_vec
            prev = self.body[i]
        
        self.body[3].se = self.body[3].Ai @ self.body[3].sep
        self.body[3].re = self.body[3].ri + self.body[3].se
        self.body[3].Ae = self.body[3].Ai @ self.body[3].Ce
        self.body[3].rpy = mat2rpy(self.body[3].Ae)
        
        for i in range(4):
            self.body[i].rhoi = self.body[i].Ai @ self.body[i].rhoip
            self.body[i].ric = self.body[i].ri + self.body[i].rhoi

    def velocity_calculation(self):
        prev = self.base
        for i in range(4):
            self.body[i].wi = prev.wi + self.body[i].Hi * self.body[i].dqi
            self.body[i].wit = skew(self.body[i].wi)
            self.body[i].dri = prev.dri + prev.wit @ self.body[i].sij
            prev = self.body[i]
        
        self.body[3].dre = self.body[3].dri + self.body[3].wit @ self.body[3].se
        
        prev = self.base
        for i in range(4):
            self.body[i].rit = skew(self.body[i].ri)
            hi_block = self.body[i].rit @ self.body[i].Hi
            self.body[i].Bi = np.concatenate([hi_block, self.body[i].Hi])
            self.body[i].drit = skew(self.body[i].dri)
            self.body[i].dric = self.body[i].dri + self.body[i].wit @ self.body[i].rhoi
            self.body[i].dHi = prev.wit @ self.body[i].Hi
            di_top = (
                self.body[i].drit @ self.body[i].Hi
                + self.body[i].rit @ self.body[i].dHi
            )
            self.body[i].Di = (
                np.concatenate([di_top, self.body[i].dHi]) * self.body[i].dqi
            )
            self.body[i].Yih = prev.Yih + self.body[i].Bi * self.body[i].dqi
            prev = self.body[i]

    def mass_force_calculation(self):
        for i in range(4):
            self.body[i].Ai_Cii = self.body[i].Ai @ self.body[i].Cii
            self.body[i].Jic = (
                self.body[i].Ai_Cii
                @ self.body[i].Jip
                @ self.body[i].Ai_Cii.T
            )
            self.body[i].rict = skew(self.body[i].ric)
            self.body[i].drict = skew(self.body[i].dric)
            self.body[i].fic = np.array([0, 0, self.body[i].mi * self.g])
            self.body[i].tic = np.array([0, 0, 0])
            rict = self.body[i].rict
            mi = self.body[i].mi
            Jic = self.body[i].Jic
            self.body[i].Mih = np.block(
                [
                    [mi * np.eye(3), -mi * rict],
                    [mi * rict, Jic - mi * (rict @ rict)],
                ]
            )
            q_top = self.body[i].fic + self.body[i].mi * (
                self.body[i].drict @ self.body[i].wi
            )
            q_bot = (
                self.body[i].tic
                + self.body[i].rict @ self.body[i].fic
                + self.body[i].mi * (self.body[i].rict @ self.body[i].drict @ self.body[i].wi)
                - self.body[i].wit @ self.body[i].Jic @ self.body[i].wi
            )
            self.body[i].Qih = np.concatenate([q_top, q_bot])
            self.body[i].Ti_tau = self.body[i].tau / self.body[i].gear

    def EQM(self):
        self.body[3].Ki = self.body[3].Mih
        self.body[2].Ki = self.body[2].Mih + self.body[3].Ki
        self.body[1].Ki = self.body[1].Mih + self.body[2].Ki
        self.body[0].Ki = self.body[0].Mih + self.body[1].Ki

        self.body[3].Li = self.body[3].Qih
        self.body[2].Li = self.body[2].Qih + self.body[3].Li - self.body[3].Ki @ self.body[3].Di
        self.body[1].Li = self.body[1].Qih + self.body[2].Li - self.body[2].Ki @ self.body[2].Di
        self.body[0].Li = self.body[0].Qih + self.body[1].Li - self.body[1].Ki @ self.body[1].Di

        M11 = self.body[0].Bi.T @ self.body[0].Ki @ self.body[0].Bi
        M12 = self.body[0].Bi.T @ self.body[1].Ki @ self.body[1].Bi
        M13 = self.body[0].Bi.T @ self.body[2].Ki @ self.body[2].Bi
        M14 = self.body[0].Bi.T @ self.body[3].Ki @ self.body[3].Bi

        M21 = self.body[1].Bi.T @ self.body[1].Ki @ self.body[0].Bi
        M22 = self.body[1].Bi.T @ self.body[1].Ki @ self.body[1].Bi
        M23 = self.body[1].Bi.T @ self.body[2].Ki @ self.body[2].Bi
        M24 = self.body[1].Bi.T @ self.body[3].Ki @ self.body[3].Bi

        M31 = self.body[2].Bi.T @ self.body[2].Ki @ self.body[0].Bi
        M32 = self.body[2].Bi.T @ self.body[2].Ki @ self.body[1].Bi
        M33 = self.body[2].Bi.T @ self.body[2].Ki @ self.body[2].Bi
        M34 = self.body[2].Bi.T @ self.body[3].Ki @ self.body[3].Bi

        M41 = self.body[3].Bi.T @ self.body[3].Ki @ self.body[0].Bi
        M42 = self.body[3].Bi.T @ self.body[3].Ki @ self.body[1].Bi
        M43 = self.body[3].Bi.T @ self.body[3].Ki @ self.body[2].Bi
        M44 = self.body[3].Bi.T @ self.body[3].Ki @ self.body[3].Bi
        M = np.block([[M11, M12, M13, M14], [M21, M22, M23, M24], [M31, M32, M33, M34], [M41, M42, M43, M44]])

        D0 = self.body[0].Di
        D01 = D0 + self.body[1].Di
        D012 = D01 + self.body[2].Di
        D0123 = D012 + self.body[3].Di

        Q1 = self.body[0].Bi @ (self.body[0].Li - self.body[0].Ki @ D0)
        Q2 = self.body[1].Bi @ (self.body[1].Li - self.body[1].Ki @ D01)
        Q3 = self.body[2].Bi @ (self.body[2].Li - self.body[2].Ki @ D012)
        Q4 = self.body[3].Bi @ (self.body[3].Li - self.body[3].Ki @ D0123)
        Q = np.array([Q1, Q2, Q3, Q4], dtype=float)

        Q += np.array([self.body[0].Ti_tau, self.body[1].Ti_tau, self.body[2].Ti_tau, self.body[3].Ti_tau], dtype=float)

        return np.linalg.solve(M, Q)

    def acceleration_calculation(self):
        prev = self.base
        for i in range(4):
            self.body[i].dYih = (
                prev.dYih
                + self.body[i].Bi * self.body[i].ddqi
                + self.body[i].Di
            )
            rit = self.body[i].rit
            self.body[i].Ti = np.block(
                [[np.eye(3), -rit], [np.zeros((3, 3)), np.eye(3)]]
            )
            drit = self.body[i].drit
            self.body[i].dTi = np.block(
                [[np.zeros((3, 3)), -drit], [np.zeros((3, 3)), np.zeros((3, 3))]]
            )
            self.body[i].dYib = (
                self.body[i].Ti @ self.body[i].dYih
                + self.body[i].dTi @ self.body[i].Yih
            )

            self.body[i].ddri = self.body[i].dYib[0:3]
            self.body[i].dwi = self.body[i].dYib[3:6]

            self.body[i].dwit = skew(self.body[i].dwi)
            self.body[i].ddric = (
                self.body[i].ddri
                + self.body[i].dwit @ self.body[i].rhoi
                + self.body[i].wit @ self.body[i].wit @ self.body[i].rhoi
            )
            prev = self.body[i]

        self.body[3].ddre = (
            self.body[3].ddri
            + self.body[3].dwit @ self.body[3].se
            + self.body[3].wit @ self.body[3].wit @ self.body[3].se
        )
        
        for i in range(4):
            self.body[i].ddqi_act = self.body[i].ddqi/self.body[i].gear

    def jacobian_calculation(self):
        A01pp_q1 = np.array([[-np.sin(self.body[0].qi), -np.cos(self.body[0].qi), 0], [np.cos(self.body[0].qi), -np.sin(self.body[0].qi), 0], [0, 0, 0]])
        A12pp_q2 = np.array([[-np.sin(self.body[1].qi), -np.cos(self.body[1].qi), 0], [np.cos(self.body[1].qi), -np.sin(self.body[1].qi), 0], [0, 0, 0]])
        A23pp_q3 = np.array([[-np.sin(self.body[2].qi), -np.cos(self.body[2].qi), 0], [np.cos(self.body[2].qi), -np.sin(self.body[2].qi), 0], [0, 0, 0]])
        A34pp_q4 = np.array([[-np.sin(self.body[3].qi), -np.cos(self.body[3].qi), 0], [np.cos(self.body[3].qi), -np.sin(self.body[3].qi), 0], [0, 0, 0]])

        A1_q1 = self.base.Ai@self.body[0].Cij@A01pp_q1
        A2_q1 = A1_q1@self.body[1].Cij@self.body[1].Aijpp
        A3_q1 = A2_q1@self.body[2].Cij@self.body[2].Aijpp
        A4_q1 = A3_q1@self.body[3].Cij@self.body[3].Aijpp

        # ∂(body[3].Ai)/∂(body[j].qi): 회전은 ``body[j].Aijpp`` 만 ``body[j].qi`` 에 의존한다.
        A2_q2 = self.body[0].Ai @ self.body[1].Cij @ A12pp_q2
        A3_q2 = A2_q2 @ self.body[2].Cij @ self.body[2].Aijpp
        A4_q2 = A3_q2 @ self.body[3].Cij @ self.body[3].Aijpp

        A3_q3 = self.body[1].Ai @ self.body[2].Cij @ A23pp_q3
        A4_q3 = A3_q3 @ self.body[3].Cij @ self.body[3].Aijpp

        A4_q4 = self.body[2].Ai @ self.body[3].Cij @ A34pp_q4

        Ae_q1 = A4_q1@self.body[3].Ce
        Ae_q2 = A4_q2@self.body[3].Ce
        Ae_q3 = A4_q3@self.body[3].Ce
        Ae_q4 = A4_q4@self.body[3].Ce

        jac_q1 = np.cross(self.body[0].Hi, (self.body[3].re - self.body[0].ri))
        jac_q2 = np.cross(self.body[1].Hi, (self.body[3].re - self.body[1].ri))
        jac_q3 = np.cross(self.body[2].Hi, (self.body[3].re - self.body[2].ri))
        jac_q4 = np.cross(self.body[3].Hi, (self.body[3].re - self.body[3].ri))
        jac_pos = np.column_stack(
            (
                np.asarray(jac_q1, dtype=float).ravel(),
                np.asarray(jac_q2, dtype=float).ravel(),
                np.asarray(jac_q3, dtype=float).ravel(),
                np.asarray(jac_q4, dtype=float).ravel(),
            )
        )
        # mat2rpy(Ae) 의 roll·pitch 오차와 일치: ∂(roll,pitch)/∂q = ⟨∂(roll,pitch)/∂Ae, ∂Ae/∂q⟩
        dAe_dq = np.stack((Ae_q1, Ae_q2, Ae_q3, Ae_q4), axis=2)
        jac_roll_pitch = roll_pitch_jacobian_wrt_q(self.body[3].Ae, dAe_dq)
        return np.vstack((jac_pos, jac_roll_pitch))

    def analysis(self, Y=None):
        """
        상태 Y = [q1..q4, dq1..dq4]에서 기구학·역학을 계산하고 Y' = [dq, ddq]를 반환.

        - ``Y``가 주어지면(예: RK4 중간 스테이지) ``self.Y``를 그 값으로 두고 계산한다.
        - ``Y``가 ``None``이면 현재 ``self.Y``를 그대로 사용한다(스텝 종료 후 동기화 등).
        """
        if Y is not None:
            self.Y = np.asarray(Y, dtype=float).copy()
        self.Y2qdq()
        self.position_calculation()
        self.velocity_calculation()
        self.mass_force_calculation()
        self.ddq = self.EQM()
        for i in range(4):
            self.body[i].ddqi = float(self.ddq[i])
        self.acceleration_calculation()
        return self.dqddq2Yp()
    
    def simple_run(self):
        self.read_data()

        csv_path = (
            Path(__file__).resolve().parent / "../recurdyn/rec_data_path.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.001
        self.g = -9.80665
        # rec_data 첫 열: 원 CSV의 시간. 마지막 행 시간으로 종료 시각 설정.
        self.t_e = float(self.rec_data[-1, 0])
        self.t_c = 0
        self.index = 0
        self.fp = open('python_data.csv', 'w+')

        while self.t_e > self.t_c:

            for i in range(4):
                self.body[i].qi = self.rec_data[self.index, 31 + i]
                self.body[i].dqi = self.rec_data[self.index, 35 + i]
                self.body[i].ddqi = self.rec_data[self.index, 39 + i]

            self.position_calculation()
            self.velocity_calculation()

            self.acceleration_calculation()

            self.data_save()

            print(self.t_c)

            self.t_c += self.h
            self.index += 1

        self.fp.close()

    def run(self):
        self.read_data()

        csv_path = (
            Path(__file__).resolve().parent / "../recurdyn/rec_data_torque.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.0001
        self.g = -9.80665
        # rec_data 첫 열: 원 CSV의 시간. 마지막 행 시간으로 종료 시각 설정.
        self.t_e = float(self.rec_data[-1, 0])
        self.t_c = 0
        self.index = 0
        self.fp = open('python_data_torque.csv', 'w+')

        for i in range(4):
            self.body[i].qi = self.rec_data[self.index, 31 + i]
            self.body[i].dqi = self.rec_data[self.index, 35 + i]

        self.Y = self.define_Y_vector()

        while self.t_e > self.t_c:
            # RK4: 각 스테이지는 반드시 해당 중간 상태 Y + a*k 에서 f를 계산해야 함.
            # (이전 코드는 y2,y3,y4를 self.Y에 넣지 않아 k2=k3=k4가 동일 상태에서 계산됨.)
            Y0 = self.Y.copy()
            k1 = self.analysis(Y0)
            k2 = self.analysis(Y0 + (self.h / 2) * k1)
            k3 = self.analysis(Y0 + (self.h / 2) * k2)
            k4 = self.analysis(Y0 + self.h * k3)
            self.Y = Y0 + (self.h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

            self.analysis()

            self.data_save()

            print(self.t_c)

            self.t_c += self.h
            self.index += 1

        self.fp.close()

    def run_vsd(
        self,
        integrate_with_mujoco=False,
        mujoco_model_path=None,
        *,
        mujoco_pd_scale=1.2,
        mujoco_tau_limit=1200.0,
        mujoco_gravity_comp=True,
    ):
        """
        작업 공간 PD 및 상태 적분.

        - ``integrate_with_mujoco=False`` (기본): 해석 역학 ``analysis`` + RK4 (기존).
        - ``integrate_with_mujoco=True``: PD 토크 ``J.T@(Ks*e + Kd*ev)`` 에 (옵션) MuJoCo 기준
          중력 보상을 더한 뒤 모터에 넣고 ``mj_step`` 으로 적분. 중력 보상은 현재 관절 상태에서
          ``qvel`` 을 0으로 둔 채 ``mj_forward`` 후 ``qfrc_bias`` 를 사용한다 (속도 0이면
          코리올리스 없이 일반화 중력에 해당).

        해석 모델과 MuJoCo primitive 의 질량·관성이 달라 동일 Ks/Kd 는 관절 토크가 과도해지기
        쉬우므로 ``mujoco_pd_scale`` 로 스케일하고 ``mujoco_tau_limit`` [Nm] 로 제한한다.

        ``mujoco_model_path`` 미지정 시 ``mujoco_pmi_viz/models/pmi_arm_primitive_actuated.xml``.
        """
        def gravity_potential_energy():
            U = 0.0
            for i in range(4):
                U += self.body[i].mi * (-self.g) * self.body[i].ric[2]
            return U

        def joint_gravity_torque():
            eps = 1e-5
            qsave = [float(self.body[i].qi) for i in range(4)]
            tau_g = np.zeros(4, dtype=float)
            for j in range(4):
                self.body[j].qi = qsave[j] + eps
                self.position_calculation()
                up = gravity_potential_energy()
                self.body[j].qi = qsave[j] - eps
                self.position_calculation()
                um = gravity_potential_energy()
                tau_g[j] = (up - um) / (2 * eps)
                self.body[j].qi = qsave[j]
            for i in range(4):
                self.body[i].qi = qsave[i]
            self.position_calculation()
            return tau_g

        self.read_data()

        csv_path = (
            Path(__file__).resolve().parent / "../recurdyn/rec_data_path.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.001
        self.g = -9.80665
        self.t_e = 3.0
        self.t_c = 0.0
        self.index = 0
        self.fp = open("python_data_vsd.csv", "w+")

        wp_t = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
        wp_x = np.array([-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35], dtype=float)
        wp_y = np.array([0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15], dtype=float)
        wp_z = np.array([-0.2, -0.2, -0.2, -0.2, 0.13, 0.13, -0.2], dtype=float)

        stack_x = _path_build_full_quintic(wp_t, wp_x, self.h)
        stack_y = _path_build_full_quintic(wp_t, wp_y, self.h)
        stack_z = _path_build_full_quintic(wp_t, wp_z, self.h)
        path_x = stack_x[:, 0]
        path_vx = stack_x[:, 1]
        path_y = stack_y[:, 0]
        path_vy = stack_y[:, 1]
        path_z = stack_z[:, 0]
        path_vz = stack_z[:, 1]

        print(
            f"path len x/y/z : {len(path_x)}, {len(path_y)}, {len(path_z)}"
        )

        for i in range(4):
            self.body[i].qi = float(self.rec_data[0, 31 + i])

        Ks = np.array([15000.0, 15000.0, 15000.0, 1500.0, 1500.0], dtype=float)
        Kd = np.array([1000.0, 1000.0, 1000.0, 10.0, 10.0], dtype=float)

        mj_model = None
        mj_data = None
        if integrate_with_mujoco:
            try:
                import mujoco
            except ImportError as exc:
                raise ImportError(
                    "integrate_with_mujoco=True 인 경우 `pip install mujoco` 가 필요합니다."
                ) from exc
            if mujoco_model_path is not None:
                mj_xml = Path(mujoco_model_path).resolve()
            else:
                # main.py 위치: <프로젝트루트>/analysis/python/main.py → 루트는 parents[2]
                mj_xml = (
                    Path(__file__).resolve().parents[2]
                    / "mujoco_pmi_viz/models/pmi_arm_primitive_actuated.xml"
                ).resolve()
            if not mj_xml.is_file():
                raise FileNotFoundError(f"MuJoCo 모델 파일 없음: {mj_xml}")
            mj_model = mujoco.MjModel.from_xml_path(str(mj_xml))
            mj_model.opt.timestep = float(self.h)
            mj_data = mujoco.MjData(mj_model)
            if mj_model.nu != 4:
                raise RuntimeError(f"MuJoCo 모델 nu가 4가 아님: nu={mj_model.nu}")
            for i in range(4):
                mj_data.qpos[i] = float(self.body[i].qi)
                mj_data.qvel[i] = float(self.body[i].dqi)
            mujoco.mj_forward(mj_model, mj_data)

        while self.t_c < self.t_e:
            des_pos = np.array(
                [path_x[self.index], path_y[self.index], path_z[self.index]],
                dtype=float,
            )
            des_roll = -np.pi / 2.0
            des_pitch = 0.0
            des_vel = np.array(
                [
                    path_vx[self.index],
                    path_vy[self.index],
                    path_vz[self.index],
                    0.0,
                    0.0,
                ],
                dtype=float,
            )

            self.position_calculation()
            self.velocity_calculation()

            ee = self.body[3]
            err_pos = des_pos - ee.re
            err_roll = wrap_to_pi(des_roll - float(ee.rpy[0]))
            err_pitch = wrap_to_pi(des_pitch - float(ee.rpy[1]))
            err = np.concatenate(
                (err_pos, np.array([err_roll, err_pitch], dtype=float))
            )

            ev = np.zeros(5, dtype=float)
            ev[0] = des_vel[0] - ee.dre[0]
            ev[1] = des_vel[1] - ee.dre[1]
            ev[2] = des_vel[2] - ee.dre[2]
            ev[3] = des_vel[3] - ee.wi[0]
            ev[4] = des_vel[4] - ee.wi[1]

            J = self.jacobian_calculation()
            Ke = Ks * err
            Kv = Kd * ev
            tau = J.T @ (Ke + Kv)

            if integrate_with_mujoco:
                if mujoco_gravity_comp:
                    v_save = mj_data.qvel.copy()
                    mj_data.qvel[:] = 0.0
                    mujoco.mj_forward(mj_model, mj_data)
                    tau_grav = np.asarray(
                        mj_data.qfrc_bias[: mj_model.nv], dtype=float
                    ).copy()
                    mj_data.qvel[:] = v_save
                    mujoco.mj_forward(mj_model, mj_data)
                    tau_pd_g = tau + tau_grav
                else:
                    tau_pd_g = tau
                tau_cmd = tau_pd_g * float(mujoco_pd_scale)
                tau_cmd = np.clip(
                    tau_cmd,
                    -float(mujoco_tau_limit),
                    float(mujoco_tau_limit),
                )
                mj_data.ctrl[:] = tau_cmd
                mujoco.mj_step(mj_model, mj_data)
                if not (
                    np.all(np.isfinite(mj_data.qpos))
                    and np.all(np.isfinite(mj_data.qvel))
                    and np.all(np.isfinite(mj_data.qacc))
                ):
                    raise RuntimeError(
                        f"MuJoCo 상태가 NaN/Inf로 발산했습니다 (t={self.t_c}). "
                        "mujoco_pd_scale 를 더 줄이거나 mujoco_tau_limit 를 낮추세요."
                    )
                for i in range(4):
                    self.body[i].qi = float(mj_data.qpos[i])
                    self.body[i].dqi = float(mj_data.qvel[i])
                    self.body[i].ddqi = float(mj_data.qacc[i])
                    self.body[i].tau = float(tau_cmd[i]) * self.body[i].gear
                self.position_calculation()
                self.velocity_calculation()
            else:
                tau_g = joint_gravity_torque()
                for i in range(4):
                    self.body[i].tau = (tau[i] + tau_g[i]) * self.body[i].gear

                self.Y = self.define_Y_vector()

                Y0 = self.Y.copy()
                k1 = self.analysis(Y0)
                k2 = self.analysis(Y0 + (self.h / 2) * k1)
                k3 = self.analysis(Y0 + (self.h / 2) * k2)
                k4 = self.analysis(Y0 + self.h * k3)
                self.Y = Y0 + (self.h / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

                self.analysis()

            self.data_save()

            print(self.t_c)

            self.t_c += self.h
            self.index += 1

        self.fp.close()

    def ik_task_error(self, des_pos, des_roll, des_pitch):
        """목표 위치·roll·pitch 대비 잔차 (5,) — 각도는 ``wrap_to_pi``."""
        ee = self.body[3]
        pos_err = np.asarray(des_pos, dtype=float) - ee.re
        roll_err = wrap_to_pi(des_roll - float(ee.rpy[0]))
        pitch_err = wrap_to_pi(des_pitch - float(ee.rpy[1]))
        return np.concatenate((pos_err, np.array([roll_err, pitch_err], dtype=float)))

    def run_ik(self):
        self.read_data()

        csv_path = (
            Path(__file__).resolve().parent / "../recurdyn/rec_data_path.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.001
        self.g = -9.80665
        self.t_e = float(self.rec_data[-1, 0])
        self.t_c = 0
        self.index = 0
        self.fp = open('python_data_path.csv', 'w+')

        wp_t = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
        wp_x = np.array([-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35], dtype=float)
        wp_y = np.array([0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15], dtype=float)
        # C++ ``run_vsd`` / ``run_ik`` 와 동일한 7개 z 웨이포인트(잘못된 3원소 배열 제거).
        wp_z = np.array([-0.2, -0.2, -0.2, -0.2, 0.13, 0.13, -0.2], dtype=float)

        stack_x = _path_build_full_quintic(wp_t, wp_x, self.h)
        stack_y = _path_build_full_quintic(wp_t, wp_y, self.h)
        stack_z = _path_build_full_quintic(wp_t, wp_z, self.h)
        path_x = stack_x[:, 0]
        path_vx = stack_x[:, 1]
        path_ax = stack_x[:, 2]
        path_y = stack_y[:, 0]
        path_vy = stack_y[:, 1]
        path_ay = stack_y[:, 2]
        path_z = stack_z[:, 0]
        path_vz = stack_z[:, 1]
        path_az = stack_z[:, 2]

        n_rec = self.rec_data.shape[0]
        print(
            f"path len x/y/z : {len(path_x)}, {len(path_y)}, {len(path_z)} "
            f"(rec_data rows {n_rec}; full quintic, 끝시각 샘플 포함)"
        )
        if len(path_x) != n_rec or len(path_y) != n_rec or len(path_z) != n_rec:
            raise ValueError(
                f"경로 샘플 수와 rec_data 행 수 불일치: path {len(path_x)}, rec {n_rec}"
            )

        for i in range(4):
            self.body[i].qi = self.rec_data[self.index, 31 + i]
            # self.body[i].dqi = self.rec_data[self.index, 35 + i]

        print(f"initial q : {self.body[0].qi}, {self.body[1].qi}, {self.body[2].qi}, {self.body[3].qi}")
        print(f"initial pos_d : {np.array([path_x[self.index], path_y[self.index], path_z[self.index]], dtype=float)}")

        while self.t_e >= self.t_c:
            des_pos = np.array([path_x[self.index], path_y[self.index], path_z[self.index]], dtype=float)
            des_roll = -np.pi / 2.0
            des_pitch = 0.0
            des_vel = np.array([path_vx[self.index], path_vy[self.index], path_vz[self.index], 0.0, 0.0], dtype=float)
            des_acc = np.array([path_ax[self.index], path_ay[self.index], path_az[self.index], 0.0, 0.0], dtype=float)

            self.position_calculation()

            err = self.ik_task_error(des_pos, des_roll, des_pitch)

            err_tol = 1e-3
            iter_count = 0
            damping = 1e-7
            alpha = 0.6
            while True:
                J = self.jacobian_calculation()
                JJT_reg = J @ J.T + (damping**2) * np.eye(5)
                delta_q = alpha * (J.T @ np.linalg.solve(JJT_reg, err))
                q_dot = J.T @ np.linalg.solve(JJT_reg, des_vel)

                for i in range(4):
                    self.body[i].qi += float(delta_q[i])
                    self.body[i].dqi = float(q_dot[i])

                self.position_calculation()
                self.velocity_calculation()

                err = self.ik_task_error(des_pos, des_roll, des_pitch)

                iter_count += 1
                if iter_count > 100:
                    print("IK failed to converge")
                    break

                if np.linalg.norm(err) < err_tol:
                    break

            self.data_save()

            print(f"time : {self.t_c}, \titeration : {iter_count}")

            self.t_c += self.h
            self.index += 1

        self.fp.close()

if __name__ == "__main__":
    main = ControlMain()
    # main.simple_run()
    # main.run()
    # main.run_ik()
    # main.run_vsd()
    main.run_vsd(integrate_with_mujoco=True, mujoco_gravity_comp=True)  # pip install mujoco