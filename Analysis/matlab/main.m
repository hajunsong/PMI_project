clc; clear all; close all;

[base, body] = read_data();

% rec_data_raw = readtable('../recurdyn/rec_data_motion.csv');
rec_data_raw = readtable('../recurdyn/rec_data_free_fall.csv');
rec_data = rec_data_raw{:, 2:end};

t_c = 0;
h = 0.001;
g = -9.80665;

fp = fopen('mat_data.csv', 'w+');

for indx = 1 : size(rec_data, 1)
    for i = 1 : 4
        body(i).qi_act = rec_data(indx, 1+i);
        body(i).dqi_act = rec_data(indx, 27+i);

        body(i).qi = body(i).qi_act*body(i).gear;
        body(i).dqi = body(i).dqi_act*body(i).gear;
    end

    body = position_calculation(base, body);
    body = velocity_calculation(base, body);

    body = mass_force_calculation(body, g);

    ddq = EQM(body);

    for i = 1 : 4
        body(i).ddqi = ddq(i);
    end
    % body(1).ddqi = -2*pi*2*pi*sin(pi*t_c*2)*3*body(1).gear;
    % body(2).ddqi = -2*pi*2*pi*sin(pi*t_c*2)*0.2*body(2).gear;
    % body(3).ddqi = -2*pi*2*pi*sin(pi*t_c*2)*0.2*body(3).gear;
    % body(4).ddqi = -2*pi*2*pi*sin(pi*t_c*2)*0.2*body(4).gear;

    body = acceleration_calculation(base, body);

    data_save(fp, t_c, body);

    fprintf('%3.5f\n', t_c);

    t_c = t_c + h;
end

fclose(fp);

plotting;
