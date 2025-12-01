function write_extrinsics_viz_proto(results, intr, outpath)
% WRITE_EXTRINSICS_VIZ_PROTO Write a VRML group showing camera frustums & axes for Webots.
%   write_extrinsics_viz_proto(results, intr, outpath)
%   - results: struct array from your pipeline. Each element should have:
%       .T_world_from_cam   4x4 transform (world_from_cam)
%       .cam_position_world 1x3 camera position
%       .worldCorners       4x3 tag corners (optional)
%       .id, .meanReproj
%   - intr: cameraIntrinsics object
%   - outpath: path to write the VRML file (e.g. 'Simulator/Worlds/ExtrinsicsViz.wrl')
%
% Save the file, then paste the generated Group { ... } into your .wbt children or
% load the .wrl (or convert to PROTO) in Webots.

if nargin < 1 || isempty(results)
    error('results must be a non-empty struct array');
end
if nargin < 2 || isempty(intr)
    error('intr (cameraIntrinsics) is required');
end
if nargin < 3 || isempty(outpath)
    outpath = fullfile(pwd,'ExtrinsicsViz.wrl');
end

% parameters for frustum drawing
depth = 0.5;       % distance from camera in meters for the frustum plane (adjustable)
axisLen = 0.12;    % length of axis arrows
boxOffset = 0.02;
boxSize = 0.02;

% get intrinsics
fx = intr.intr.FocalLength(1); fy = intr.intr.FocalLength(2);
cx = intr.intr.PrincipalPoint(1); cy = intr.intr.PrincipalPoint(2);
W = intr.intr.ImageSize(2); H = intr.intr.ImageSize(1);

% image corners (TL TR BR BL)
imgPts = [1 1; W 1; W H; 1 H];

% compute camera-space frustum corners at depth
camPts = zeros(4,3);
for k=1:4
    u = imgPts(k,1); v = imgPts(k,2);
    x = (u - cx) / fx;
    y = (v - cy) / fy;
    camPts(k,:) = [x*depth, y*depth, depth];
end

% open file
fid = fopen(outpath,'w','n','UTF-8');
if fid < 0
    error('Cannot open %s for writing', outpath);
end

fprintf(fid, '#VRML_SIM R2025a utf8\n');
fprintf(fid, 'Group {\n  children [\n');

for i=1:numel(results)
    res = results(i);
    if ~isfield(res,'T_world_from_cam') || isempty(res.T_world_from_cam)
        warning('result %d missing T_world_from_cam, skipping', i);
        continue;
    end
    T = res.T_world_from_cam; % world_from_cam
    % compute world coords of camera frustum plane corners
    Pcam = [camPts'; ones(1,4)];
    Pw_h = T * Pcam; % 4x4 hom column
    Pw = Pw_h(1:3,:)'; % 4x3 world points

    % camera origin in world
    originw_h = T * [0;0;0;1];
    originw = originw_h(1:3)';

    baseName = sprintf('camera_%02d', res.id);

    % 1) wireframe lines (origin + 4 corners)
    fprintf(fid, '    Shape {\n');
    fprintf(fid, '      geometry IndexedLineSet {\n');
    fprintf(fid, '        coord Coordinate { point [\n');
    % origin
    fprintf(fid, '          %.6g %.6g %.6g,\n', originw(1), originw(2), originw(3));
    % four corners with commas
    for kk = 1:4
        if kk < 4
            comma = ',';
        else
            comma = '';
        end
        fprintf(fid, '          %.6g %.6g %.6g%s\n', Pw(kk,1), Pw(kk,2), Pw(kk,3), comma);
    end
    fprintf(fid, '        ] }\n');
    fprintf(fid, '        coordIndex [ 0,1,-1, 0,2,-1, 0,3,-1, 0,4,-1, 1,2,-1, 2,3,-1, 3,4,-1, 4,1,-1 ]\n');
    fprintf(fid, '      }\n');
    fprintf(fid, '      appearance Appearance { material Material { emissiveColor 1 0 0 } }\n');
    fprintf(fid, '    }\n\n');

    % 2) semi-transparent frustum face
    fprintf(fid, '    Shape {\n');
    fprintf(fid, '      geometry IndexedFaceSet {\n');
    fprintf(fid, '        coord Coordinate { point [\n');
    for kk = 1:4
        if kk < 4
            comma = ',';
        else
            comma = '';
        end
        fprintf(fid, '          %.6g %.6g %.6g%s\n', Pw(kk,1), Pw(kk,2), Pw(kk,3), comma);
    end
    fprintf(fid, '        ] }\n');
    fprintf(fid, '        coordIndex [ 0,1,2,3,-1 ]\n');
    fprintf(fid, '        solid FALSE\n');
    fprintf(fid, '      }\n');
    fprintf(fid, '      appearance Appearance { material Material { diffuseColor 0.8 0.8 0.2 } }\n');
    fprintf(fid, '    }\n\n');

    % 3) axis lines at camera origin (X red, Y green, Z blue)
    R_c2w = T(1:3,1:3);
    xend = originw + (axisLen * R_c2w(:,1))';
    yend = originw + (axisLen * R_c2w(:,2))';
    zend = originw + (axisLen * R_c2w(:,3))';

    % X axis
    fprintf(fid, '    Shape { geometry IndexedLineSet { coord Coordinate { point [ %.6g %.6g %.6g, %.6g %.6g %.6g ] } coordIndex [ 0,1 ] } appearance Appearance { material Material { emissiveColor 1 0 0 } } }\n', ...
        originw(1), originw(2), originw(3), xend(1), xend(2), xend(3));
    % Y axis
    fprintf(fid, '    Shape { geometry IndexedLineSet { coord Coordinate { point [ %.6g %.6g %.6g, %.6g %.6g %.6g ] } coordIndex [ 0,1 ] } appearance Appearance { material Material { emissiveColor 0 1 0 } } }\n', ...
        originw(1), originw(2), originw(3), yend(1), yend(2), yend(3));
    % Z axis
    fprintf(fid, '    Shape { geometry IndexedLineSet { coord Coordinate { point [ %.6g %.6g %.6g, %.6g %.6g %.6g ] } coordIndex [ 0,1 ] } appearance Appearance { material Material { emissiveColor 0 0 1 } } }\n', ...
        originw(1), originw(2), originw(3), zend(1), zend(2), zend(3));

    fprintf(fid, '\n');

    % 4) small box above camera as marker
    bx = originw(1); by = originw(2); bz = originw(3);
    fprintf(fid, '    Transform { translation %.6g %.6g %.6g children [ Shape { geometry Box { size %.6g %.6g %.6g } appearance Appearance { material Material { diffuseColor 0.1 0.1 0.1 } } } ] }\n', ...
        bx, by + boxOffset, bz, boxSize, boxSize, 0.005);

    % colored sphere indicating reproj magnitude (red intensity proportional)
    if isfield(res,'meanReproj') && ~isempty(res.meanReproj)
        val = res.meanReproj;
    else
        val = 0;
    end
    r = min(1, max(0, val / 200)); % scale to 0..1 (tune divisor as needed)
    fprintf(fid, '    Transform { translation %.6g %.6g %.6g children [ Shape { geometry Sphere { radius 0.01 } appearance Appearance { material Material { diffuseColor %.3g 0 0 } } } ] }\n\n', ...
        bx + 0.03, by + boxOffset, bz, r);

    % 5) draw tag quad if present
    if isfield(res,'worldCorners') && ~isempty(res.worldCorners)
        wc = res.worldCorners;
        fprintf(fid, '    Shape { geometry IndexedFaceSet { coord Coordinate { point [\n');
        for kk = 1:4
            if kk < 4
                comma = ',';
            else
                comma = '';
            end
            fprintf(fid, '          %.6g %.6g %.6g%s\n', wc(kk,1), wc(kk,2), wc(kk,3), comma);
        end
        fprintf(fid, '        ] } coordIndex [ 0,1,2,3,-1 ] solid FALSE } appearance Appearance { material Material { diffuseColor 0.3 0.6 0.9 } } }\n\n');
    end

end

fprintf(fid, '  ]\n}\n');
fclose(fid);
fprintf('Wrote extrinsics viz to: %s\n', outpath);
end