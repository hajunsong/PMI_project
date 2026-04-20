function velocity_calculation()

    global base body

    % velocity calculation
    prev = base;
    for i = 1 : 4
        body(i).Hi = prev.Ai*body(i).Cij*body(i).u_vec;
        body(i).wi = prev.wi + body(i).Hi*body(i).dqi;
        body(i).wit = skew(body(i).wi);
        body(i).dri = prev.dri + prev.wit*body(i).sij;
        prev = body(i);
    end

    body(4).dre = body(4).dri + body(4).wit*body(4).se;

    prev = base;
    for i = 1 : 4
        body(i).rit = skew(body(i).ri);
        body(i).Bi = [body(i).rit*body(i).Hi;body(i).Hi];
        body(i).drit = skew(body(i).dri);
        body(i).dric = body(i).dri + body(i).wit*body(i).rhoi;
        body(i).dHi = prev.wit*body(i).Hi;
        body(i).Di = [body(i).drit*body(i).Hi + body(i).rit*body(i).dHi;body(i).dHi]*body(i).dqi;
        body(i).Yih = prev.Yih + body(i).Bi*body(i).dqi;
        prev = body(i);
    end

end