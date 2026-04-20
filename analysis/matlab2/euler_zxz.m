% 회전 행렬 함수
function R = euler_zxz(psi, theta, phi)
    % psi, theta, phi를 받아서 Z-X-Z 순서의 Euler 회전 행렬을 만듦
    % Rz(psi)
    Rz_psi = [cos(psi), -sin(psi), 0;
               sin(psi), cos(psi), 0;
               0, 0, 1];
    
    % Rx(theta)
    Rx_theta = [1, 0, 0;
          0, cos(theta), -sin(theta);
          0, sin(theta), cos(theta)];
    
    % Rz(phi)
    Rz_phi = [cos(phi), -sin(phi), 0;
               sin(phi), cos(phi), 0;
               0, 0, 1];
    
    % Combined rotation matrix: R = Rz(psi) * Rx(theta) * Rz(phi)
    R = Rz_psi * Rx_theta * Rz_phi;
end 