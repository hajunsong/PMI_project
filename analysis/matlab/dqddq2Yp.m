function Yp = dqddq2Yp()
    
    global body

    Yp = [body(1).dqi_act; 
          body(2).dqi_act; 
          body(3).dqi_act; 
          body(4).dqi_act; 
          body(1).ddqi_act; 
          body(2).ddqi_act; 
          body(3).ddqi_act; 
          body(4).ddqi_act];

end