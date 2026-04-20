function [base, body] = read_data()

    global base body
        
    base.Ai = eye(3);
    base.ri = zeros(3,1);
    base.wi = zeros(3,1);
    base.dri = zeros(3,1);
    base.wit = skew(base.wi);
    base.Yih = zeros(6,1);
    base.dYih = zeros(6,1);

    body(1).qi = 0;
    body(1).qi_act = 0;
    body(1).gear = 32/60;
    body(1).dqi = 0;
    body(1).dqi_act = 0;
    body(1).ddqi = 0;
    body(1).ddqi_act = 0;
    body(1).sijp = [0;0;0];
    body(1).Cij = euler_zxz(0, pi, 0);
    body(1).u_vec = [0;0;1];
    body(1).rhoip = [0.0026336; -1.68446e-05; -0.111117];
    body(1).Cii = euler_zxz(0, 0, 0);
    body(1).mi = 10.0123496865811;
    Ixx = 8.29601614566715e-002; Ixy = -3.28089653994623e-004;
    Iyy = 3.91199810914461e-002; Iyz = 2.09012210735718e-006;
    Izz = 6.49458588226152e-002; Izx = -1.19428008908532e-004;
    body(1).Jip = [Ixx, Ixy, Izx; Ixy, Iyy, Iyz; Izx, Iyz, Izz];
    body(1).tau = 0;

    body(2).qi = 0;
    body(2).qi_act = 0;
    body(2).gear = 360/54;
    body(2).dqi = 0;
    body(2).dqi_act = 0;
    body(2).ddqi = 0;
    body(2).ddqi_act = 0;
    body(2).sijp = [0;0;-0.22];
    body(2).Cij = euler_zxz(0, pi/2, 0);
    body(2).u_vec = [0;0;1];
    body(2).rhoip = [6.90559e-05;-0.0851548;-0.00686211];
    body(2).Cii = euler_zxz(pi, pi/2, pi);
    body(2).mi = 10.4391437674567;
    Ixx = 7.80970193464117e-002; Ixy = 3.53131298101708e-005;
    Iyy = 2.55871855807303e-002; Iyz = 5.47282289489732e-003;
    Izz = 7.45746466344453e-002; Izx = -8.81620777833472e-006;
    body(2).Jip = [Ixx, Ixy, Izx; Ixy, Iyy, Iyz; Izx, Iyz, Izz];
    body(2).tau = 375;

    body(3).qi = 0;
    body(3).qi_act = 0;
    body(3).gear = 360/108;
    body(3).dqi = 0;
    body(3).dqi_act = 0;
    body(3).ddqi = 0;
    body(3).ddqi_act = 0;
    body(3).sijp = [0;-0.23;0];
    body(3).Cij = euler_zxz(-pi/2, 0, 0);
    body(3).u_vec = [0;0;1];
    body(3).rhoip = [0.0969069;-3.20036e-05;-0.00548574];
    body(3).Cii = euler_zxz(-pi/2, pi/2, pi);
    body(3).mi = 10.3406497234359;
    Ixx = 7.92872354716476e-002; Ixy = 8.26042768438978e-006;
    Iyy = 2.27404242493157e-002; Iyz = 4.31986606376804e-003;
    Izz = 7.81409250910383e-002; Izx = 5.43833216931251e-006;
    body(3).Jip = [Ixx, Ixy, Izx; Ixy, Iyy, Iyz; Izx, Iyz, Izz];
    body(3).tau = 60;

    body(4).qi = 0;
    body(4).qi_act = 0;
    body(4).gear = 360/108;
    body(4).dqi = 0;
    body(4).dqi_act = 0;
    body(4).ddqi = 0;
    body(4).ddqi_act = 0;
    body(4).sijp = [0.23;0;0];
    body(4).Cij = euler_zxz(0, pi, 0);
    body(4).u_vec = [0;0;1];
    body(4).rhoip = [0.0675884;0.00443192;0.000679202];
    body(4).Cii = euler_zxz(-pi/2, pi/2, 0);
    body(4).mi = 7.01416597186014;
    Ixx = 3.93493190184971e-002; Ixy = -9.26169916890393e-004;
    Iyy = 1.11764686166838e-002; Iyz = 2.04330936352529e-004;
    Izz = 4.10218257620852e-002; Izx = -8.43406506016272e-006;
    body(4).Jip = [Ixx, Ixy, Izx; Ixy, Iyy, Iyz; Izx, Iyz, Izz];
    body(4).tau = -3;

    body(4).sep = [0.18;0;0];
    body(4).Ce = euler_zxz(-pi/2, 0, 0);

end