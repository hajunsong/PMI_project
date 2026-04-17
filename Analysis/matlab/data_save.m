function data_save(fp, t_c, body)
   
    fprintf(fp, '%3.5f,', t_c);
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f,', body(1).qi_act, body(2).qi_act, body(3).qi_act, body(4).qi_act);
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f,', body(1).qi, body(2).qi, body(3).qi, body(4).qi);
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f,', body(4).re(1), body(4).re(2), body(4).re(3), body(4).rpy(1), body(4).rpy(2), body(4).rpy(3));
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f,', body(4).dre(1), body(4).dre(2), body(4).dre(3), body(4).wi(1), body(4).wi(2), body(4).wi(3));
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f, %3.7f, %3.7f,', body(4).ddre(1), body(4).ddre(2), body(4).ddre(3), body(4).dwi(1), body(4).dwi(2), body(4).dwi(3));
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f,', body(1).dqi_act, body(2).dqi_act, body(3).dqi_act, body(4).dqi_act);
    fprintf(fp, '%3.7f, %3.7f, %3.7f, %3.7f,', body(1).dqi, body(2).dqi, body(3).dqi, body(4).dqi);
    fprintf(fp, '\n');
    
end