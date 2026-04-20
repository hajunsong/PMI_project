clc; clear all; close all;

format long g

global base body t_c h g fp Y

read_data();

% rec_data_raw = readtable('../recurdyn/rec_data_motion.csv');
% rec_data_raw = readtable('../recurdyn/rec_data_free_fall.csv');
% rec_data_raw = readtable('../recurdyn/rec_data_torque.csv');
rec_data_raw = readtable('../recurdyn/rec_data_path.csv');
rec_data = rec_data_raw{:, 2:end};

t_c = 0;
h = 0.001;
g = -9.80665;
t_e = 3;
indx = 1;

fp = fopen('mat_data.csv', 'w+');

for i = 1 : 4
    body(i).qi_act = rec_data(1, 19+i);
    body(i).dqi_act = rec_data(1, 23+i);
end

define_Y_vector();

while t_e > t_c
    for i = 1 : 4
        body(i).qi = rec_data(indx, 31+i);
        body(i).dqi = rec_data(indx, 35+i);
        body(i).ddqi = rec_data(indx, 39+i);
    end

    Yp = analysis();

    % runge-kutta 4th order method
    % k1 = Yp;
    % y2 = Y + (h/2)*k1;
    % k2 = analysis();
    % y3 = Y + (h/2)*k2;
    % k3 = analysis();
    % y4 = Y + h*k3;
    % k4 = analysis();
    % Y = Y + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

    % tspan = [t_c, t_c+h];
    % [t, Y] = ode45(@(t, Y) analysis(), tspan, Y);
    % Y = Y(end,:);

    % tspan = [t_c, t_c+h];
    % options = odeset('BDF', 'off', 'RelTol', 1e-3, 'AbsTol', 1e-6);
    % [t, Y] = ode15s(@(t, Y) analysis(), tspan, Y, options);
    % Y = Y(end,:);

    data_save(fp, t_c, body);

    fprintf('%3.5f\n', t_c);

    t_c = t_c + h;
    indx = indx + 1;
end

fclose(fp);

plotting;

% waitfor(gcf);
