function result = estimate_world_coordinates(imagePath)
% estimate_world_coordinates Estimate camera pose (world coordinates) from one image
% Usage:
%   res = estimate_world_coordinates('../../cache/camera/temp_img.png');
%
% Output (struct):
%   res.T_world_from_camera   - 4x4 homogeneous transform (world <- camera)
%   res.position_world        - [x y z] camera position in world coords
%   res.euler_deg             - [yaw, pitch, roll] degrees (ZYX / yaw-pitch-roll)
%   res.quaternion            - [w x y z]
%   res.detections            - array of per-tag diagnostics (id, reprojErr, tagCornersImg, worldCorners, T_world_from_cam_i)
%   res.fused                 - details of fused pose (weights, inliers, translations, quaternions)
%
% Requirements:
%  - cameraParams.mat must exist and contain variable cameraParams (MATLAB cameraParameters)
%  - tagWorldMap.mat must exist and contain struct tagWorldMap where
%        tagWorldMap(tagIndex).id = tagID
%        tagWorldMap(tagIndex).worldCorners = 4x3 matrix in order [tl; tr; br; bl]
%    OR a containers.Map with keys = tagID (as string or number) and values = 4x3 matrices.
%  - Computer Vision Toolbox: readAprilTag, extrinsics, worldToImage, undistortPoints
%
% If you don't have tagWorldMap prepared, see comments below for example generator.

%% --- 1. load cameraParams & tagWorldMap ---
% try common paths
candidates = {fullfile(pwd,'cameraParams.mat'), fullfile(pwd,'..','..','cache','camera','cameraParams.mat'), ...
              fullfile('..','cache','camera','cameraParams.mat'), 'cameraParams.mat'};
cameraParams = [];
for i=1:numel(candidates)
    if exist(candidates{i}, 'file')
        s = load(candidates{i});
        fn = fieldnames(s);
        cameraParams = s.(fn{1}); % assume cameraParams is first
        break;
    end
end
if isempty(cameraParams)
    error('找不到 cameraParams.mat（请把由 estimateCameraParameters 生成的 cameraParams.mat 放到当前目录或 ../../cache/camera/ ）。');
end

% load tagWorldMap
mapCandidates = {fullfile(pwd,'tagWorldMap.mat'), fullfile(pwd,'..','..','cache','camera','tagWorldMap.mat'), ...
                 fullfile('..','cache','camera','tagWorldMap.mat')};
tagWorldMap = [];
for i=1:numel(mapCandidates)
    if exist(mapCandidates{i}, 'file')
        s = load(mapCandidates{i});
        % try find a variable named tagWorldMap
        if isfield(s,'tagWorldMap')
            tagWorldMap = s.tagWorldMap;
        else
            % try any variable that looks like a map or struct of tag corners
            fn = fieldnames(s);
            tagWorldMap = s.(fn{1});
        end
        break;
    end
end
if isempty(tagWorldMap)
    error(['找不到 tagWorldMap.mat（应包含变量 tagWorldMap，映射 tagID -> 4x3 worldCorners）。', newline, ...
           '示例结构：tagWorldMap(1).id = 0; tagWorldMap(1).worldCorners = [x1 y1 z1; x2 y2 z2; x3 y3 z3; x4 y4 z4];']);
end

% normalize tagWorldMap access: allow containers.Map or struct array
mapIsContainer = isa(tagWorldMap, 'containers.Map');
if ~mapIsContainer
    % allow struct array with fields .id and .worldCorners
    if isstruct(tagWorldMap) && isfield(tagWorldMap,'id') && isfield(tagWorldMap,'worldCorners')
        % ok
    else
        error('tagWorldMap 格式不被识别。请参考函数注释准备 tagWorldMap（struct array 或 containers.Map）。');
    end
end

%% --- 2. read image and detect tags ---
I = imread(imagePath);
if size(I,3)==3
    gray = rgb2gray(I);
else
    gray = I;
end

% optionally undistort the whole image for display; for PnP we will undistort points
try
    imgUndist = undistortImage(I, cameraParams);
catch
    imgUndist = I; % if undistortImage not available, proceed with raw
end

% detect AprilTags
tagFamily = 'tag36h11';
try
    detections = readAprilTag(imgUndist, tagFamily);
catch ME
    error('readAprilTag 调用失败：%s\n请确认 Computer Vision Toolbox 已安装并支持 readAprilTag。', ME.message);
end

if isempty(detections)
    error('图像中未检测到任何 AprilTag。');
end

%% --- 3. process each detection: undistort points, PnP (extrinsics) ---
detectionsInfo = []; % struct array
for k = 1:numel(detections)
    id = detections(k).ID;
    corners_img = detections(k).CornerPoints; % 4x2 (tl,tr,br,bl) - 注意顺序依实现可能不同
    % ensure corners are Nx2 double
    corners_img = double(corners_img);
    % undistort corner points to match cameraParams intrinsics
    ptsUnd = undistortPoints(corners_img, cameraParams); % Nx2
    
    % get world corners for this tag from tagWorldMap
    worldCorners = getWorldCornersForID(tagWorldMap, id);
    if isempty(worldCorners)
        warning('未在 tagWorldMap 找到 ID=%d 的 worldCorners，跳过该 tag。', id);
        continue;
    end
    % ensure worldCorners is 4x3 (tl,tr,br,bl)
    if size(worldCorners,1) ~= 4 || size(worldCorners,2) < 3
        error('tag %d 的 worldCorners 必须为 4x3。', id);
    end
    
    % PnP: extrinsics expects imagePoints (undistorted) and worldPoints
    try
        [R, t] = extrinsics(ptsUnd, worldCorners, cameraParams);
        % extrinsics returns R, t such that X_cam = R * X_world' + t'
    catch ME
        warning('extrinsics 失败 for tag %d: %s', id, ME.message);
        continue;
    end
    
    % compute reprojection to measure error
    proj = worldToImage(cameraParams.Intrinsics, R, t, worldCorners);
    reprojErrs = sqrt(sum((proj - corners_img).^2, 2)); % per corner in px (use original measured corners)
    meanReproj = mean(reprojErrs);
    
    % compute pixel size measure (diagonal)
    diagPx = norm(corners_img(1,:) - corners_img(3,:));
    
    % transform to camera pose in world coordinates:
    % given: X_cam = R * X_world + t   =>  X_world = R'*(X_cam - t)
    % camera-to-world transform: [R', -R'*t]
    R_c2w = R';
    t_c2w = -R' * t';
    T_world_from_cam_i = eye(4);
    T_world_from_cam_i(1:3,1:3) = R_c2w;
    T_world_from_cam_i(1:3,4) = t_c2w;
    
    % store
    detinfo.id = id;
    detinfo.corners_img = corners_img;
    detinfo.corners_undistorted = ptsUnd;
    detinfo.worldCorners = worldCorners;
    detinfo.R = R;
    detinfo.t = t;
    detinfo.T_world_from_cam = T_world_from_cam_i;
    detinfo.reprojPerCorner = reprojErrs;
    detinfo.meanReproj = meanReproj;
    detinfo.diagPx = diagPx;
    detectionsInfo = [detectionsInfo; detinfo];
end

if isempty(detectionsInfo)
    error('所有检测到的 tags 都无法做 extrinsics，检查 tagWorldMap 与 detections 匹配。');
end

%% --- 4. fuse multiple tag-based camera poses into one camera pose ---
% Build arrays
M = numel(detectionsInfo);
trans = zeros(M,3);
quats = zeros(M,4); % MATLAB quat as [w x y z] from rotm2quat
weights = zeros(M,1);
ids = zeros(M,1);
reprojs = zeros(M,1);
for i=1:M
    T = detectionsInfo(i).T_world_from_cam;
    trans(i,:) = T(1:3,4)';
    Rcw = T(1:3,1:3);
    q = rotm2quat(Rcw); % quaternions [w x y z]
    % ensure sign consistency relative to first quat
    if i==1
        qref = q;
    else
        if dot(qref,q) < 0
            q = -q;
        end
    end
    quats(i,:) = q;
    % weight: use diagPx (bigger tag in pixels better) and inverse reproj
    weights(i) = detectionsInfo(i).diagPx / (1 + detectionsInfo(i).meanReproj);
    ids(i) = detectionsInfo(i).id;
    reprojs(i) = detectionsInfo(i).meanReproj;
end

% outlier rejection by translation distance to median
medT = median(trans,1);
dists = vecnorm(trans - medT, 2, 2);
inlierMask = dists < max(0.25, 2*median(dists)); % 0.25m or 2*median fallback
if sum(inlierMask) < 1
    inlierMask = true(size(inlierMask)); % be conservative
end

% apply mask
trans_in = trans(inlierMask,:);
quats_in = quats(inlierMask,:);
weights_in = weights(inlierMask);
ids_in = ids(inlierMask);
reprojs_in = reprojs(inlierMask);

% weighted translation
t_fused = sum((trans_in .* weights_in),1) / sum(weights_in);

% weighted quaternion average (simple weighted sum then normalize)
q_weighted = sum(quats_in .* weights_in,1) / sum(weights_in);
q_weighted = q_weighted / norm(q_weighted);
R_fused = quat2rotm(q_weighted);

T_world_from_cam = eye(4);
T_world_from_cam(1:3,1:3) = R_fused;
T_world_from_cam(1:3,4) = t_fused';

%% --- 5. pack results ---
result = struct();
result.T_world_from_camera = T_world_from_cam;
result.position_world = t_fused;
% convert rotation to yaw-pitch-roll (ZYX)
rpy = rotm2eul(R_fused, 'ZYX'); % returns [z y x] in radians
result.euler_deg = rad2deg(rpy); % [yaw, pitch, roll]
result.quaternion = q_weighted; % [w x y z]
result.detections = detectionsInfo;
result.fused.weights = weights_in;
result.fused.ids = ids_in;
result.fused.reproj = reprojs_in;
result.fused.inlierMask = inlierMask;
result.fused.translations = trans_in;
result.fused.quaternions = quats_in;

%% --- 6. display summary ---
fprintf('Estimations from %d tags, %d inliers used.\n', M, sum(inlierMask));
fprintf('Fused camera position (world): [%.3f %.3f %.3f] m\n', t_fused(1), t_fused(2), t_fused(3));
fprintf('Fused camera orientation (yaw,pitch,roll deg): [%.2f %.2f %.2f]\n', result.euler_deg(1), result.euler_deg(2), result.euler_deg(3));
fprintf('Per-tag mean reproj errors (px):\n');
for i=1:numel(detectionsInfo)
    fprintf('  id=%d, reproj=%.3f px, diag=%.1f px\n', detectionsInfo(i).id, detectionsInfo(i).meanReproj, detectionsInfo(i).diagPx);
end

end

%% ---------------- helper ----------------
function worldCorners = getWorldCornersForID(tagWorldMap, id)
% accepts either struct array tagWorldMap with fields .id and .worldCorners,
% or containers.Map keyed by id (string/num) with value=4x3 matrix.
worldCorners = [];
if isa(tagWorldMap,'containers.Map')
    key = id;
    if isKey(tagWorldMap, key)
        worldCorners = tagWorldMap(key);
    else
        % try string key
        key = num2str(id);
        if isKey(tagWorldMap, key)
            worldCorners = tagWorldMap(key);
        end
    end
    return;
end
% struct array
if isstruct(tagWorldMap)
    for k=1:numel(tagWorldMap)
        if isequal(tagWorldMap(k).id, id)
            worldCorners = tagWorldMap(k).worldCorners;
            return;
        end
    end
end
end