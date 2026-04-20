function traj = waypoint_time_path(wp_t, wp_x, method)
%WAYPOINT_TIME_PATH  웨이포인트 (wp_t, wp_x)로 시간 구간별 연속 경로 생성.
%
%   traj = waypoint_time_path(wp_t, wp_x)
%   traj = waypoint_time_path(wp_t, wp_x, method)
%
%   wp_t : 시각 노드 벡터 (단조 증가), 예 [0, 0.5, 1, ...]
%   wp_x : 해당 시각의 목표 x (wp_t와 같은 길이)
%   method : 구간 연결 방식 (기본값 'spline')
%       'linear' — 구간별 선형
%       'pchip'  — 구간별 단조 보존 3차 Hermite (과도한 오버슛 완화)
%       'spline' — 전역 조각 3차 스플라인 (곡선 매끄러움, 웨이포인트 통과)
%
%   사용:
%       x = traj.x(t_c);      % 현재 시간 t_c에 대한 경로점 x (스칼라/행·열 벡터)
%       traj.wp_t, traj.wp_x — 입력 웨이포인트
%       traj.method
%       traj.n_seg           — 시간 구간 개수 (= numel(wp_t)-1)
%
%   wp_t(1) ~ wp_t(end) 밖 시간은 interp1 규칙상 NaN (외삽 안 함).

    if nargin < 3 || isempty(method)
        method = 'spline';
    end

    wp_t = wp_t(:).';
    wp_x = wp_x(:).';

    if numel(wp_t) ~= numel(wp_x)
        error('waypoint_time_path: wp_t 와 wp_x 길이가 같아야 합니다.');
    end

    if numel(wp_t) < 1
        error('waypoint_time_path: 웨이포인트가 비어 있습니다.');
    end

    if numel(wp_t) >= 2 && any(diff(wp_t) <= 0)
        error('waypoint_time_path: wp_t 는 엄격히 증가해야 합니다.');
    end

    traj.wp_t = wp_t;
    traj.wp_x = wp_x;
    traj.method = method;
    traj.n_seg = max(0, numel(wp_t) - 1);

    traj.x = @(t) eval_wp(wp_t, wp_x, method, t);
end

function x = eval_wp(wp_t, wp_x, method, t)
    if numel(wp_t) == 1
        x = wp_x(1) + zeros(size(t));
        return;
    end

    sz = size(t);
    tc = t(:);

    switch lower(method)
        case {'linear', 'pchip', 'spline'}
            xc = interp1(wp_t, wp_x, tc, lower(method), NaN);
        otherwise
            error('waypoint_time_path: 지원하지 않는 method 입니다 (linear|pchip|spline).');
    end

    x = reshape(xc, sz);
end
