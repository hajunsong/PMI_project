function mass_force_calculation()

    global body g
    
    % mass matrix and force vector calculation
    for i = 1 : 4
        body(i).Ai_Cii = body(i).Ai*body(i).Cii;
        body(i).Jic = body(i).Ai_Cii*body(i).Jip*body(i).Ai_Cii';
        body(i).rict = skew(body(i).ric);
        body(i).drict = skew(body(i).dric);

        body(i).fic = [0;0;body(i).mi*g];
        body(i).tic = [0;0;0];

        body(i).Mih = [body(i).mi*eye(3), -body(i).mi*body(i).rict;
                        body(i).mi*body(i).rict, body(i).Jic - body(i).mi*body(i).rict*body(i).rict];
        body(i).Qih = [body(i).fic + body(i).mi*body(i).drict*body(i).wi;
                        body(i).tic + body(i).rict*body(i).fic + body(i).mi*body(i).rict*body(i).drict*body(i).wi - body(i).wit*body(i).Jic*body(i).wi];
        body(i).Ti_tau = body(i).tau/body(i).gear;
        body(i).Qih_tau = [zeros(3,1); body(i).Ti_tau*body(i).Hi];
        % body(i).Qih = body(i).Qih + body(i).Qih_tau;
    end

end