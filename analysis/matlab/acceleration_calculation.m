function acceleration_calculation()

    global base body

    % acceleration calculation
    prev = base;
    for i = 1 : 4
        body(i).dYih = prev.dYih + body(i).Bi*body(i).ddqi + body(i).Di;
        body(i).Ti = [eye(3), -body(i).rit;zeros(3), eye(3)];
        body(i).dTi = [zeros(3), -body(i).drit;zeros(3,6)];
        body(i).dYib = body(i).Ti*body(i).dYih + body(i).dTi*body(i).Yih;

        body(i).ddri = body(i).dYib(1:3,1);
        body(i).dwi = body(i).dYib(4:6,1);

        body(i).dwit = skew(body(i).dwi);
        body(i).ddric = body(i).ddri + body(i).dwit*body(i).rhoi + body(i).wit*body(i).wit*body(i).rhoi;
        prev = body(i);
    end

    body(4).ddre = body(4).ddri + body(4).dwit*body(4).se + body(4).wit*body(4).wit*body(4).se;

    for i = 1 : 4
        body(i).ddqi_act = body(i).ddqi/body(i).gear;
    end

end