function ddq = EQM(body)
    
    body(4).Ki = body(4).Mih;
    body(3).Ki = body(3).Mih + body(4).Ki;
    body(2).Ki = body(2).Mih + body(3).Ki;
    body(1).Ki = body(1).Mih + body(2).Ki;

    body(4).Li = body(4).Qih;
    body(3).Li = body(3).Qih + body(4).Li - body(4).Ki*body(4).Di;
    body(2).Li = body(2).Qih + body(3).Li - body(3).Ki*body(3).Di;
    body(1).Li = body(1).Qih + body(2).Li - body(2).Ki*body(2).Di;

    M = [body(1).Bi'*body(1).Ki*body(1).Bi body(1).Bi'*body(2).Ki*body(2).Bi body(1).Bi'*body(3).Ki*body(3).Bi body(1).Bi'*body(4).Ki*body(4).Bi;
         body(2).Bi'*body(2).Ki*body(1).Bi body(2).Bi'*body(2).Ki*body(2).Bi body(2).Bi'*body(3).Ki*body(3).Bi body(2).Bi'*body(4).Ki*body(4).Bi;
         body(3).Bi'*body(3).Ki*body(1).Bi body(3).Bi'*body(3).Ki*body(2).Bi body(3).Bi'*body(3).Ki*body(3).Bi body(3).Bi'*body(4).Ki*body(4).Bi;
         body(4).Bi'*body(4).Ki*body(1).Bi body(4).Bi'*body(4).Ki*body(2).Bi body(4).Bi'*body(4).Ki*body(3).Bi body(4).Bi'*body(4).Ki*body(4).Bi];

    Q = [body(1).Bi'*(body(1).Li - body(1).Ki*(body(1).Di)); 
         body(2).Bi'*(body(2).Li - body(2).Ki*(body(1).Di + body(2).Di)); 
         body(3).Bi'*(body(3).Li - body(3).Ki*(body(1).Di + body(2).Di + body(3).Di)); 
         body(4).Bi'*(body(4).Li - body(4).Ki*(body(1).Di + body(2).Di + body(3).Di + body(4).Di))];

    ddq = M\Q;

end