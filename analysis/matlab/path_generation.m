function path = path_generation(x0, xf, tf, ta, h)

    td = tf - ta;
    vd = (xf - x0) / td;
    xa = x0 + 0.5 * ta * vd;
    xd = xf - 0.5 * ta * vd;

    % section of acceleration
    pos0 = x0; posf = xa; vel0 = 0; velf = vd; acc0 = 0; accf = 0; ts = ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*ts^2)/(2*ts^3);
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*ts^2)/(2*ts^4);
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*ts^2)/(2*ts^5);

    path = [];
    for t = 0 : h : ts - h
        path = [path;a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5];
    end

    % section of constant velocity
    pos0 = xa; posf = xd; vel0 = vd; velf = vd; acc0 = 0; accf = 0; ts = td - ta;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*ts^2)/(2*ts^3);
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*ts^2)/(2*ts^4);
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*ts^2)/(2*ts^5);

    for t = 0 : h : ts
        path = [path;a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5];
    end

    % section of deceleration
    pos0 = xd; posf = xf; vel0 = vd; velf = 0; acc0 = 0; accf = 0; ts = tf - td;
    a0 = pos0;
    a1 = vel0;
    a2 = acc0/2;
    a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*ts^2)/(2*ts^3);
    a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*ts^2)/(2*ts^4);
    a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*ts^2)/(2*ts^5);

    for t = h : h : ts
        path = [path;a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5];
    end

end