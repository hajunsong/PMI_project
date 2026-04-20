function Yp = analysis
    
    global base body Y g

    % Y2qdq();

    % for i = 1 : 4
    %     body(i).qi = body(i).qi_act*body(i).gear;
    %     body(i).dqi = body(i).dqi_act*body(i).gear;
    % end

    position_calculation();
    velocity_calculation();

    mass_force_calculation();

    ddq = EQM();

    % for i = 1 : 4
    %     body(i).ddqi = ddq(i);
    % end

    acceleration_calculation();

    Yp = dqddq2Yp();
end