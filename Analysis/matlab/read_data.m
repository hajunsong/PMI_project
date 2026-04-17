function [base, body] = read_data()
        
    base.Ai = eye(3);
    base.ri = zeros(3,1);
    base.wi = zeros(3,1);
    base.dri = zeros(3,1);
    base.dYih = zeros(6,1);
    base.wit = skew(base.wi);
    base.Yih = zeros(6,1);

    body(1).qi = 0;
    body(1).qi_act = 0;
    body(1).gear = 32/60;
    body(1).dqi = 0;
    body(1).dqi_act = 0;
    body(1).sijp = [0;0;0];
    body(1).Cij = euler_zxz(0, 1.5708, 3.14159);
    body(1).u_vec = [0;0;1];
    body(1).rhoip = [0.0026336; -1.68446e-05; -0.111117];
    body(1).Cii = euler_zxz(0, 1.5708, 3.14159);
    body(1).mi = 10.0123496865811;
    body(1).Ixx = 8.29601614566715e-002; body(1).Ixy = -3.28089653994623e-004;
    body(1).Iyy = 3.91199810914461e-002; body(1).Iyz = 2.09012210735718e-006;
    body(1).Izz = 6.49458588226152e-002; body(1).Izx = -1.19428008908532e-004;
    body(1).Jip = [body(1).Ixx, body(1).Ixy, body(1).Izx; body(1).Ixy, body(1).Iyy, body(1).Iyz; body(1).Izx, body(1).Iyz, body(1).Izz];

    body(2).qi = 0;
    body(2).qi_act = 0;
    body(2).gear = 360/54;
    body(2).dqi = 0;
    body(2).dqi_act = 0;
    body(2).sijp = [0;0;-0.22];
    body(2).Cij = euler_zxz(0, 1.5708, 0);
    body(2).u_vec = [0;0;1];
    body(2).rhoip = [6.90559e-05;-0.0851548;-0.00686211];
    body(2).Cii = euler_zxz(3.14159, 0, 0);
    body(2).mi = 10.4391437674567;
    body(2).Ixx = 7.80970193464117e-002; body(2).Ixy = 3.53131298101708e-005;
    body(2).Iyy = 2.55871855807303e-002; body(2).Iyz = 5.47282289489732e-003;
    body(2).Izz = 7.45746466344453e-002; body(2).Izx = -8.81620777833472e-006;
    body(2).Jip = [body(2).Ixx, body(2).Ixy, body(2).Izx; body(2).Ixy, body(2).Iyy, body(2).Iyz; body(2).Izx, body(2).Iyz, body(2).Izz];

    body(3).qi = 0;
    body(3).qi_act = 0;
    body(3).gear = 360/108;
    body(3).dqi = 0;
    body(3).dqi_act = 0;
    body(3).sijp = [0;-0.23;0];
    body(3).Cij = euler_zxz(-1.5708, 0, 0);
    body(3).u_vec = [0;0;1];
    body(3).rhoip = [0.0969069;-3.20036e-05;-0.00548574];
    body(3).Cii = euler_zxz(-1.5708, 0, 0);
    body(3).mi = 10.3406497234359;
    body(3).Ixx = 7.92872354716476e-002; body(3).Ixy = 8.26042768438978e-006;
    body(3).Iyy = 2.27404242493157e-002; body(3).Iyz = 4.31986606376804e-003;
    body(3).Izz = 7.81409250910383e-002; body(3).Izx = 5.43833216931251e-006;
    body(3).Jip = [body(3).Ixx, body(3).Ixy, body(3).Izx; body(3).Ixy, body(3).Iyy, body(3).Iyz; body(3).Izx, body(3).Iyz, body(3).Izz];

    body(4).qi = 0;
    body(4).qi_act = 0;
    body(4).gear = 360/108;
    body(4).dqi = 0;
    body(4).dqi_act = 0;
    body(4).sijp = [0.23;0;0];
    body(4).Cij = euler_zxz(0, 3.14159, 0);
    body(4).u_vec = [0;0;1];
    body(4).rhoip = [0.0675884;0.00443192;0.000679202];
    body(4).Cii = euler_zxz(1.5708, 3.14159, 0);
    body(4).mi = 7.01416597186014;
    body(4).Ixx = 3.93493190184971e-002; body(4).Ixy = -9.26169916890393e-004;
    body(4).Iyy = 1.11764686166838e-002; body(4).Iyz = 2.04330936352529e-004;
    body(4).Izz = 4.10218257620852e-002; body(4).Izx = -8.43406506016272e-006;
    body(4).Jip = [body(4).Ixx, body(4).Ixy, body(4).Izx; body(4).Ixy, body(4).Iyy, body(4).Iyz; body(4).Izx, body(4).Iyz, body(4).Izz];

    body(4).sep = [0.18;0;0];
    body(4).Ce = euler_zxz(-1.5708, 0, 0);

end