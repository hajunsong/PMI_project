function position_calculation

    global base body

    % position and orientation calculation
    for i = 1 : 4
        body(i).Aijpp = [cos(body(i).qi), -sin(body(i).qi), 0;sin(body(i).qi), cos(body(i).qi), 0;0, 0, 1];
    end

    prev = base;
    for i = 1 : 4
        body(i).Ai = prev.Ai*body(i).Cij*body(i).Aijpp;
        body(i).sij = prev.Ai*body(i).sijp;
        body(i).ri = prev.ri + body(i).sij;
        prev = body(i);
    end
    
    body(4).se = body(4).Ai*body(4).sep;
    body(4).re = body(4).ri + body(4).se;
    body(4).Ae = body(4).Ai*body(4).Ce;
    body(4).rpy = mat2rpy(body(4).Ae);

    for i = 1 : 4
        body(i).rhoi = body(i).Ai*body(i).rhoip;
        body(i).ric = body(i).ri + body(i).rhoi;
    end

end