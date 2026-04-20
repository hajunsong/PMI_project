function define_Y_vector
    
    global body
    global Y
    
    Y(1,1) = body(1).qi_act;
    Y(2,1) = body(2).qi_act;
    Y(3,1) = body(3).qi_act;
    Y(4,1) = body(4).qi_act;
    Y(5,1) = body(1).dqi_act;
    Y(6,1) = body(2).dqi_act;
    Y(7,1) = body(3).dqi_act;
    Y(8,1) = body(4).dqi_act;

end