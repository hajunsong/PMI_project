"""베이스 및 링크 질량·기하·감속비·토크 등 모델 파라미터를 ControlMain 인스턴스에 채운다."""

import numpy as np

from utils import euler_zxz, skew

def read_data(ctrl) -> None:
    # base body parameters
    ctrl.base.Ai = np.eye(3)
    ctrl.base.ri = np.zeros(3)
    ctrl.base.wi = np.zeros(3)
    ctrl.base.dri = np.zeros(3)
    ctrl.base.wit = skew(ctrl.base.wi)
    ctrl.base.Yih = np.zeros(6)
    ctrl.base.dYih = np.zeros(6)

    # body1 parameters
    ctrl.body[0].qi = 0
    ctrl.body[0].qi_act = 0
    ctrl.body[0].gear = 32/60
    ctrl.body[0].dqi = 0
    ctrl.body[0].dqi_act = 0
    ctrl.body[0].ddqi = 0
    ctrl.body[0].ddqi_act = 0
    ctrl.body[0].sijp = np.array([0, 0, 0])
    ctrl.body[0].Cij = euler_zxz(0, np.pi, 0)
    ctrl.body[0].u_vec = np.array([0, 0, 1])
    ctrl.body[0].rhoip = np.array([0.0026336, -1.68446e-05, -0.111117])
    ctrl.body[0].Cii = euler_zxz(0, 0, 0)
    ctrl.body[0].mi = 10.0123496865811
    Ixx = 8.29601614566715e-002; Ixy = -3.28089653994623e-004;
    Iyy = 3.91199810914461e-002; Iyz = 2.09012210735718e-006;
    Izz = 6.49458588226152e-002; Izx = -1.19428008908532e-004;
    ctrl.body[0].Jip = np.array([[Ixx, Ixy, Izx], [Ixy, Iyy, Iyz], [Izx, Iyz, Izz]])
    ctrl.body[0].tau = 0

    # body2 parameters
    ctrl.body[1].qi = 0
    ctrl.body[1].qi_act = 0
    ctrl.body[1].gear = 360/54
    ctrl.body[1].dqi = 0
    ctrl.body[1].dqi_act = 0
    ctrl.body[1].ddqi = 0
    ctrl.body[1].ddqi_act = 0
    ctrl.body[1].sijp = np.array([0, 0, -0.22])
    ctrl.body[1].Cij = euler_zxz(0, np.pi / 2, 0)
    ctrl.body[1].u_vec = np.array([0, 0, 1])
    ctrl.body[1].rhoip = np.array([6.90559e-05, -0.0851548, -0.00686211])
    ctrl.body[1].Cii = euler_zxz(np.pi, np.pi / 2, np.pi)
    ctrl.body[1].mi = 10.4391437674567
    Ixx = 7.80970193464117e-002; Ixy = 3.53131298101708e-005;
    Iyy = 2.55871855807303e-002; Iyz = 5.47282289489732e-003;
    Izz = 7.45746466344453e-002; Izx = -8.81620777833472e-006;
    ctrl.body[1].Jip = np.array([[Ixx, Ixy, Izx], [Ixy, Iyy, Iyz], [Izx, Iyz, Izz]])
    ctrl.body[1].tau = 375

    # body3 parameters
    ctrl.body[2].qi = 0
    ctrl.body[2].qi_act = 0
    ctrl.body[2].gear = 360/108
    ctrl.body[2].dqi = 0
    ctrl.body[2].dqi_act = 0
    ctrl.body[2].ddqi = 0
    ctrl.body[2].ddqi_act = 0
    ctrl.body[2].sijp = np.array([0, -0.23, 0])
    ctrl.body[2].Cij = euler_zxz(-np.pi / 2, 0, 0)
    ctrl.body[2].u_vec = np.array([0, 0, 1])
    ctrl.body[2].rhoip = np.array([0.0969069, -3.20036e-05, -0.00548574])
    ctrl.body[2].Cii = euler_zxz(-np.pi / 2, np.pi / 2, np.pi)
    ctrl.body[2].mi = 10.3406497234359
    Ixx = 7.92872354716476e-002; Ixy = 8.26042768438978e-006;
    Iyy = 2.27404242493157e-002; Iyz = 4.31986606376804e-003;
    Izz = 7.81409250910383e-002; Izx = 5.43833216931251e-006;
    ctrl.body[2].Jip = np.array([[Ixx, Ixy, Izx], [Ixy, Iyy, Iyz], [Izx, Iyz, Izz]])
    ctrl.body[2].tau = 60

    # body4 parameters
    ctrl.body[3].qi = 0
    ctrl.body[3].qi_act = 0
    ctrl.body[3].gear = 360/108
    ctrl.body[3].dqi = 0
    ctrl.body[3].dqi_act = 0
    ctrl.body[3].ddqi = 0
    ctrl.body[3].ddqi_act = 0
    ctrl.body[3].sijp = np.array([0.23, 0, 0])
    ctrl.body[3].Cij = euler_zxz(0, np.pi, 0)
    ctrl.body[3].u_vec = np.array([0, 0, 1])
    ctrl.body[3].rhoip = np.array([0.0675884, 0.00443192, 0.000679202])
    ctrl.body[3].Cii = euler_zxz(-np.pi / 2, np.pi / 2, 0)
    ctrl.body[3].mi = 7.01416597186014
    Ixx = 3.93493190184971e-002; Ixy = -9.26169916890393e-004;
    Iyy = 1.11764686166838e-002; Iyz = 2.04330936352529e-004;
    Izz = 4.10218257620852e-002; Izx = -8.43406506016272e-006;
    ctrl.body[3].Jip = np.array([[Ixx, Ixy, Izx], [Ixy, Iyy, Iyz], [Izx, Iyz, Izz]])
    ctrl.body[3].tau = -3

    # end effector parameters
    ctrl.body[3].sep = np.array([0.18, 0, 0])
    ctrl.body[3].Ce = euler_zxz(-np.pi / 2, 0, 0)
