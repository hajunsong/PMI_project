clc; clear all; close all;

format long g

[base, body] = read_data();

q_init = [0.40489179, -1.9254499, -0.41545447, 0.80069097];

t_c = 0;
h = 0.001;
g = -9.80665;
t_e = 3;

wp_t = [0, 0.5, 1, 1.5, 2, 2.5, 3];
wp_x = [-0.35, -0.25, 0.25, 0.35, 0.18, -0.18, -0.35];
wp_y = [0.15, -0.28, -0.28, 0.15, 0.37, 0.37, 0.15];
wp_z = [-0.2, 0.13, -0.2];
const_roll = -pi/2;
const_pitch = pi;

path_x1 = path_generation(wp_x(1), wp_x(2), wp_t(2) - wp_t(1), 0.1, h);
path_x2 = path_generation(wp_x(2), wp_x(3), wp_t(3) - wp_t(2), 0.1, h);
path_x3 = path_generation(wp_x(3), wp_x(4), wp_t(4) - wp_t(3), 0.1, h);
path_x4 = path_generation(wp_x(4), wp_x(5), wp_t(5) - wp_t(4), 0.1, h);
path_x5 = path_generation(wp_x(5), wp_x(6), wp_t(6) - wp_t(5), 0.1, h);
path_x6 = path_generation(wp_x(6), wp_x(7), wp_t(7) - wp_t(6), 0.1, h);
path_x = [path_x1; path_x2; path_x3; path_x4; path_x5; path_x6];

path_y1 = path_generation(wp_y(1), wp_y(2), wp_t(2) - wp_t(1), 0.1, h);
path_y2 = path_generation(wp_y(2), wp_y(3), wp_t(3) - wp_t(2), 0.1, h);
path_y3 = path_generation(wp_y(3), wp_y(4), wp_t(4) - wp_t(3), 0.1, h);
path_y4 = path_generation(wp_y(4), wp_y(5), wp_t(5) - wp_t(4), 0.1, h);
path_y5 = path_generation(wp_y(5), wp_y(6), wp_t(6) - wp_t(5), 0.1, h);
path_y6 = path_generation(wp_y(6), wp_y(7), wp_t(7) - wp_t(6), 0.1, h);
path_y = [path_y1; path_y2; path_y3; path_y4; path_y5; path_y6];

path_z1 = path_generation(0, wp_z(1), wp_t(2) - wp_t(1), 0.1, h);
path_z2 = path_generation(0, wp_z(1), wp_t(3) - wp_t(2), 0.1, h);
path_z3 = path_generation(0, wp_z(1), wp_t(4) - wp_t(3), 0.1, h);
path_z4 = path_generation(0, wp_z(2), wp_t(5) - wp_t(4), 0.1, h);
path_z5 = path_generation(0, wp_z(3), wp_t(6) - wp_t(5), 0.1, h);
path_z6 = path_generation(0, wp_z(3), wp_t(7) - wp_t(6), 0.1, h);
path_z = [path_z1; path_z2; path_z3; path_z4; path_z5; path_z6];

fp = fopen('mat_data_path.csv', 'w+');

indx = 1;

while t_e > t_c
    x_des = path_x(indx);
    y_des = path_y(indx);
    z_des = path_z(indx);

    

    fprintf('%3.5f\n', t_c);
    t_c = t_c + h;
    indx = indx + 1;
end

fclose(fp);

plotting('../recurdyn/rec_data_path.csv', 'mat_data_path.csv');