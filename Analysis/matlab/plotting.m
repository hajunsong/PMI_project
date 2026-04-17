% rec_data와 mat_data.csv 비교
% rec_data_raw = readtable('../recurdyn/rec_data_motion.csv');
rec_data_raw = readtable('../recurdyn/rec_data_free_fall.csv');
rec_data = rec_data_raw{:, 2:end};
mat_data_raw = readtable('mat_data.csv');
mat_data = mat_data_raw{:, 1:end};

% 시간 데이터
t_rec = rec_data(:, 1);
t_mat = mat_data(:, 1);

% 모터각도 비교 (q1, q2, q3, q4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, 1+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 1+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['q_', num2str(i), ' (rad)']);
    title(['Joint Angle q_', num2str(i)]);
    grid on;
    legend;
    hold off;
end

% 관절각도 비교 (q1, q2, q3, q4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, 5+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 5+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['q_', num2str(i), ' (rad)']);
    title(['Joint Angle q_', num2str(i)]);
    grid on;
    legend;
    hold off;
end

% 엔드 이펙터 위치 비교 (rx, ry, rz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, 9+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 9+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['r_', char(119+i), ' (m)']);
        title(['End-Effector Position r_', char(119+i)]);
    else
        ylabel(['\theta_', char(119+i-3), ' (rad)']);
        title(['End-Effector Orientation \theta_', char(119+i-3)]);
    end
    grid on;
    legend;
    hold off;
end

% 엔드 이펙터 속도 비교 (선속도 vx, vy, vz, 각속도 wx, wy, wz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, 15+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 15+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['v_', char(119+i), ' (m/s)']);
        title(['End-Effector Linear Velocity v_', char(119+i)]);
    else
        ylabel(['\omega_', char(119+i-3), ' (rad/s)']);
        title(['End-Effector Angular Velocity \omega_', char(119+i-3)]);
    end
    grid on;
    legend;
    hold off;
end

% 엔드 이펙터 가속도 비교 (선가속도 ax, ay, az, 각가속도 alphax, alphay, alphaz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, 21+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 21+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['a_', char(119+i), ' (m/s^2)']);
        title(['End-Effector Linear Acceleration a_', char(119+i)]);
    else
        ylabel(['\alpha_', char(119+i-3), ' (rad/s^2)']);
        title(['End-Effector Angular Acceleration \alpha_', char(119+i-3)]);
    end
    grid on;
    legend;
    hold off;
end

% 모터각 속도 비교 (dq1_act, dq2_act, dq3_act, dq4_act)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, 27+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 27+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['dq_', num2str(i), '_act (rad/s)']);
    title(['Motor Angle Velocity dq_', num2str(i), '_act']);
    grid on;
    legend;
    hold off;
end

% 관절각 속도 비교 (dq1, dq2, dq3, dq4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, 31+i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, 31+i), 'r--', 'LineWidth', 1.5, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    title(['Joint Angle Velocity dq_', num2str(i)]);
    grid on;
    legend;
    hold off;
end