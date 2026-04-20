function plotting(rec_path, mat_path)
% rec_data와 mat_data.csv 비교
% rec_data_raw = readtable('../recurdyn/rec_data_motion.csv');
% rec_data_raw = readtable('../recurdyn/rec_data_free_fall.csv');
% rec_data_raw = readtable('../recurdyn/rec_data_torque.csv');
if nargin < 1
    rec_path = '../recurdyn/rec_data_path.csv';
end
if nargin < 2
    mat_path = 'mat_data.csv';
end
rec_data_raw = readtable(rec_path);
rec_data = rec_data_raw{:, 2:end};
mat_data_raw = readtable(mat_path);
mat_data = mat_data_raw{:, 1:end};

% 시간 데이터
t_rec = rec_data(:, 1);
t_mat = mat_data(:, 1);

index_end_pos = 1; % 엔드 이펙터 위치 데이터의 시작 열 인덱스
index_end_vel = 7; % 엔드 이펙터 속도 데이터의 시작 열 인덱스
index_end_acc = 13; % 엔드 이펙터 가속도 데이터 시작 열 인덱스
index_q_act = 19; % 모터각도 데이터 시작 열 인덱스
index_dq_act = 23; % 모터각 속도 데이터 시작 열 인덱스
index_ddq_act = 27; % 모터각 가속도 데이터 시작 열 인덱스
index_q = 31; % 관절각도 데이터 시작 열 인덱스
index_dq = 35; % 관절각 속도 데이터 시작 열 인덱스
index_ddq = 39; % 관절각 가속도 데이터 시작 열 인덱스

% 엔드 이펙터 위치 비교 (rx, ry, rz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, index_end_pos+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_end_pos+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['r_', char(119+i), ' (m)']);
        title(['End-Effector Position r_', char(119+i)]);
    else
        ylabel(['\theta_', char(119+i-3), ' (rad)']);
        title(['End-Effector Orientation \theta_', char(119+i-3)]);
    end
    grid on;
    if i == 3
        legend('Location', 'northeast');
    end
    hold off;
end

% 엔드 이펙터 속도 비교 (선속도 vx, vy, vz, 각속도 wx, wy, wz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, index_end_vel+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_end_vel+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['v_', char(119+i), ' (m/s)']);
        title(['End-Effector Linear Velocity v_', char(119+i)]);
    else
        ylabel(['\omega_', char(119+i-3), ' (rad/s)']);
        title(['End-Effector Angular Velocity \omega_', char(119+i-3)]);
    end
    grid on;
    if i == 3
        legend('Location', 'northeast');
    end
    hold off;
end

% 엔드 이펙터 가속도 비교 (선가속도 ax, ay, az, 각가속도 alphax, alphay, alphaz)
figure('Position', [100, 100, 1200, 600]);

for i = 1:6
    subplot(2, 3, i);
    plot(t_rec, rec_data(:, index_end_acc+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_end_acc+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    if i <= 3
        ylabel(['a_', char(119+i), ' (m/s^2)']);
        title(['End-Effector Linear Acceleration a_', char(119+i)]);
    else
        ylabel(['\alpha_', char(119+i-3), ' (rad/s^2)']);
        title(['End-Effector Angular Acceleration \alpha_', char(119+i-3)]);
    end
    grid on;
    if i == 3
        legend('Location', 'northeast');
    end
    hold off;
end

% 모터각도 비교 (q1, q2, q3, q4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_q_act+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_q_act+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['q_', num2str(i), ' (rad)']);
    title(['Motor Angle q_', num2str(i)]);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

% 관절각도 비교 (q1, q2, q3, q4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_q+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_q+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['q_', num2str(i), ' (rad)']);
    title(['Joint Angle q_', num2str(i)]);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

% 모터각 속도 비교 (dq1_act, dq2_act, dq3_act, dq4_act)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_dq_act+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_dq_act+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['dq_', num2str(i), 'act (rad/s)']);
    title(['Motor Angle Velocity dq_', num2str(i), '_act']);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

% 관절각 속도 비교 (dq1, dq2, dq3, dq4)
figure('Position', [100, 100, 1200, 800]);

for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_dq+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_dq+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    title(['Joint Angle Velocity dq_', num2str(i)]);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

% 모터각 가속도 비교 (ddq1_act, ddq2_act, ddq3_act, ddq4_act)
figure('Position', [100, 100, 1200, 800]);
for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_ddq+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_ddq+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['ddq_', num2str(i), ' (rad/s^2)']);
    title(['Motor Angle Acceleration ddq_', num2str(i)]);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

% 관절각 가속도 비교 (ddq1, ddq2, ddq3, ddq4)
figure('Position', [100, 100, 1200, 800]);
for i = 1:4
    subplot(2, 2, i);
    plot(t_rec, rec_data(:, index_ddq_act+i), 'b-', 'LineWidth', 2, 'DisplayName', 'RecurDyn');
    hold on;
    plot(t_mat, mat_data(:, index_ddq_act+i), 'r--', 'LineWidth', 2, 'DisplayName', 'MATLAB');
    xlabel('Time (s)');
    ylabel(['ddq_', num2str(i), ' (rad/s^2)']);
    title(['Joint Angle Acceleration ddq_', num2str(i)]);
    grid on;
    if i == 2
        legend('Location', 'northeast');
    end
    hold off;
end

end