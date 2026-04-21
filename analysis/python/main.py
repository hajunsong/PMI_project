#!/usr/bin/env python3

from pathlib import Path

import numpy as np

from read_data import read_data as apply_model_read_data
from utils import mat2rpy, skew

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
        self.indx = 0
        self.fp = None
        self.base = Body()
        self.body = [Body() for _ in range(4)]
        self.rec_data = np.empty((0, 0))
        self.read_data()

    def read_data(self) -> None:
        apply_model_read_data(self)

    def position_calculation(self):
        for i in range(4):
            self.body[i].Aijpp = np.array([[np.cos(self.body[i].qi), -np.sin(self.body[i].qi), 0], [np.sin(self.body[i].qi), np.cos(self.body[i].qi), 0], [0, 0, 1]])
        
        prev = self.base
        for i in range(4):
            self.body[i].Ai = prev.Ai @ self.body[i].Cij @ self.body[i].Aijpp
            self.body[i].sij = prev.Ai @ self.body[i].sijp
            self.body[i].ri = prev.ri + self.body[i].sij
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
            self.body[i].Hi = prev.Ai @ self.body[i].Cij @ self.body[i].u_vec
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
            self.body[i].Qih_tau = np.concatenate(
                [np.zeros(3), self.body[i].Ti_tau * self.body[i].Hi]
            )
            # self.body[i].Qih = self.body[i].Qih + self.body[i].Qih_tau

    def EQM(self):
        self.body[3].Ki = self.body[3].Mih
        self.body[2].Ki = self.body[2].Mih + self.body[3].Ki
        self.body[1].Ki = self.body[1].Mih + self.body[2].Ki
        self.body[0].Ki = self.body[0].Mih + self.body[1].Ki

        self.body[3].Li = self.body[3].Qih
        self.body[2].Li = self.body[2].Qih + self.body[3].Li - self.body[3].Ki @ self.body[3].Di# - self.body[3].Qih_tau
        self.body[1].Li = self.body[1].Qih + self.body[2].Li - self.body[2].Ki @ self.body[2].Di# - self.body[2].Qih_tau
        self.body[0].Li = self.body[0].Qih + self.body[1].Li - self.body[1].Ki @ self.body[1].Di# - self.body[1].Qih_tau

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

    def data_save(self):
        self.fp.write(f'{self.t_c},')
        self.fp.write(f'{self.body[3].re[0]}, {self.body[3].re[1]}, {self.body[3].re[2]}, {self.body[3].rpy[0]}, {self.body[3].rpy[1]}, {self.body[3].rpy[2]},')
        self.fp.write(f'{self.body[3].dre[0]}, {self.body[3].dre[1]}, {self.body[3].dre[2]}, {self.body[3].wi[0]}, {self.body[3].wi[1]}, {self.body[3].wi[2]},')
        self.fp.write(f'{self.body[3].ddre[0]}, {self.body[3].ddre[1]}, {self.body[3].ddre[2]}, {self.body[3].dwi[0]}, {self.body[3].dwi[1]}, {self.body[3].dwi[2]},')
        self.fp.write(f'{self.body[0].qi_act}, {self.body[1].qi_act}, {self.body[2].qi_act}, {self.body[3].qi_act},')
        self.fp.write(f'{self.body[0].dqi_act}, {self.body[1].dqi_act}, {self.body[2].dqi_act}, {self.body[3].dqi_act},')
        self.fp.write(f'{self.body[0].ddqi_act}, {self.body[1].ddqi_act}, {self.body[2].ddqi_act}, {self.body[3].ddqi_act},')
        self.fp.write(f'{self.body[0].qi}, {self.body[1].qi}, {self.body[2].qi}, {self.body[3].qi},')
        self.fp.write(f'{self.body[0].dqi}, {self.body[1].dqi}, {self.body[2].dqi}, {self.body[3].dqi},')
        self.fp.write(f'{self.body[0].ddqi}, {self.body[1].ddqi}, {self.body[2].ddqi}, {self.body[3].ddqi},')
        self.fp.write('\n')

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

    def run(self):
        self.read_data()

        csv_path = (
            # Path(__file__).resolve().parent / "../recurdyn/rec_data_path.csv"
            # Path(__file__).resolve().parent / "../recurdyn/rec_data_free_fall.csv"
            Path(__file__).resolve().parent / "../recurdyn/rec_data_torque.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.0001
        self.g = -9.80665
        # rec_data 첫 열: 원 CSV의 시간. 마지막 행 시간으로 종료 시각 설정.
        self.t_e = float(self.rec_data[-1, 0])
        self.t_c = 0
        self.indx = 0
        self.fp = open('python_data.csv', 'w+')

        for i in range(4):
            self.body[i].qi = self.rec_data[self.indx, 31 + i]
            self.body[i].dqi = self.rec_data[self.indx, 35 + i]

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

            for i in range(4):
                self.body[i].qi_act = self.body[i].qi/self.body[i].gear
                self.body[i].dqi_act = self.body[i].dqi/self.body[i].gear
                self.body[i].ddqi_act = self.body[i].ddqi/self.body[i].gear

            self.data_save()

            print(self.t_c)

            self.t_c += self.h
            self.indx += 1

        self.fp.close()

    def run_ik(self):
        self.read_data()

        csv_path = (
            Path(__file__).resolve().parent / "../recurdyn/rec_data_motion.csv"
        ).resolve()
        rec_data_raw = np.loadtxt(csv_path, delimiter=",")
        self.rec_data = rec_data_raw[:, 1:]

        self.h = 0.0001
        self.g = -9.80665
        self.t_e = float(self.rec_data[-1, 0])
        self.t_c = 0
        self.indx = 0
        self.fp = open('python_data_motion.csv', 'w+')

        for i in range(4):
            self.body[i].qi = self.rec_data[self.indx, 31 + i]
            self.body[i].dqi = self.rec_data[self.indx, 35 + i]

        des_p = self.rec_data[self.indx, 1:3]
        des_r = body[1].qi + body[2].qi + body[3].qi

        des_dp = self.rec_data[self.indx, 7:10]
        des_w = self.rec_data[self.indx, 10:13]

        while self.t_e >= self.t_c:
            for i in range(4):
                self.body[i].qi_act = self.body[i].qi/self.body[i].gear
                # self.body[i].dqi_act = self.body[i].dqi/self.body[i].gear
                # self.body[i].ddqi_act = self.body[i].ddqi/self.body[i].gear

            self.position_calculation()
            err_p = des_p - self.body[3].re

            self.data_save()

            print(self.t_c)

            self.t_c += self.h
            self.indx += 1

        self.fp.close()

if __name__ == "__main__":
    main = ControlMain()
    # main.run()
    main.run_ik()