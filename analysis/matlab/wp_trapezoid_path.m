function traj = wp_trapezoid_path(wp_t, wp_x, ta, h)
%WP_TRAPEZOID_PATH  웨이포인트 (wp_t, wp_x)를 구간별 사다리꼴(5차) 프로파일로 연결.
%
%   각 구간 k: wp_x(k) → wp_x(k+1), 구간 시간 tf_k = wp_t(k+1)-wp_t(k),
%   로컬 시간 0…tf_k 에 대해 quintic_trapezoid_path 와 동일한 가속·등속·감속.
%   검측점에서 이전 구간 끝·다음 구간 시작 속도가 0이므로 위치만 연속으로 맞춤.
%
%   traj = wp_trapezoid_path(wp_t, wp_x, ta)
%   traj = wp_trapezoid_path(wp_t, wp_x, ta, h)   % h: 구간별 샘플용(선택)
%
%   ta : 각 구간에 동일하게 적용되는 가속·감속 시간 (모든 구간에서 tf_k > 2*ta 필요)
%   traj.x(t) : 절대 시간 t에 대한 목표값 (스칼라/배열). 웨이포인트 시간 범위 밖은 NaN.
%
%   traj.seg{k} : k번째 구간의 quintic_trapezoid_path 결과 구조체

    if nargin < 4
        h = [];
    end

    wp_t = wp_t(:).';
    wp_x = wp_x(:).';

    if numel(wp_t) ~= numel(wp_x)
        error('wp_trapezoid_path: wp_t 와 wp_x 길이가 같아야 합니다.');
    end
    if numel(wp_t) < 2
        error('wp_trapezoid_path: 웨이포인트는 최소 2개 필요합니다.');
    end
    if any(diff(wp_t) <= 0)
        error('wp_trapezoid_path: wp_t 는 엄격히 증가해야 합니다.');
    end

    nseg = numel(wp_t) - 1;
    dt = diff(wp_t);
    if any(dt <= 2 * ta)
        error('wp_trapezoid_path: 모든 구간에서 wp_t(k+1)-wp_t(k) > 2*ta 이어야 합니다 (현재 ta=%g).', ta);
    end

    seg = cell(1, nseg);
    for k = 1:nseg
        seg{k} = quintic_trapezoid_path(wp_x(k), wp_x(k+1), dt(k), ta, h);
    end

    traj.wp_t = wp_t;
    traj.wp_x = wp_x;
    traj.ta = ta;
    traj.seg = seg;
    traj.n_seg = nseg;

    traj.x = @(t) eval_wp_trap(seg, wp_t, nseg, t);
end

function x = eval_wp_trap(seg, wp_t, nseg, t)
    sz = size(t);
    tc = t(:);
    x = nan(size(tc));

    for k = 1:nseg
        lo = wp_t(k);
        hi = wp_t(k+1);
        if k < nseg
            mask = tc >= lo & tc < hi;
        else
            mask = tc >= lo & tc <= hi;
        end
        if any(mask)
            loc = tc(mask) - lo;
            x(mask) = seg{k}.x(loc);
        end
    end

    x = reshape(x, sz);
end
