% crop_and_center_tag36h11.m
% 说明：
% - 扫描 ./tag36h11/*.png
% - 去掉白边（自动检测），等比缩放并居中放入 1024x1024 画布
% - 结果保存到 ./tag36h11_cropped 文件夹（若不存在则创建）
% - 可选是否把输出强制为二值（0 和 255）
%
% 直接运行本脚本即可；若要修改路径或参数，请在脚本顶部调整。

%% ========== 用户可修改参数 ==========
srcFolder = fullfile(pwd, 'tag36h11');       % 源文件夹（默认当前目录下的 tag36h11）
outFolder = fullfile(pwd, 'tag36h11_cropped'); % 输出文件夹
outSize = 1024;            % 输出 PNG 的大小（正方形）
marginPixels = 0;          % 在放入 1024x1024 时四周额外保留的像素 (>=0)。如果想留静默区可设为 e.g. 16
forceBinaryOutput = true;  % 是否强制输出为二值 (0/255)。如果 false，会保留彩色/灰度
whiteThreshold = 250;      % 判定为"白"的灰度阈值（0-255），越大越严格。如果图片有压缩伪影可把它调小
minComponentArea = 50;     % 忽略小连通域（噪点），单位像素
debugShow = false;         % 若要每张显示处理过程改为 true（会弹图，处理慢）
%% ====================================

% 检查输入目录
if ~isfolder(srcFolder)
    error('找不到源文件夹：%s\n请确认路径并重试。', srcFolder);
end
if ~exist(outFolder, 'dir')
    mkdir(outFolder);
end

% 获取 png 列表
files = dir(fullfile(srcFolder, '*.png'));
if isempty(files)
    fprintf('源文件夹 %s 中没有 PNG 文件。\n', srcFolder);
    return;
end

fprintf('开始处理 %d 个 PNG 文件：\n', numel(files));

for k = 1:numel(files)
    fname = fullfile(srcFolder, files(k).name);
    try
        [I, map, alpha] = imread(fname); %#ok<ASGLU,ASGLU>
    catch ME
        fprintf('无法读取 %s：%s ，跳过。\n', files(k).name, ME.message);
        continue;
    end

    % 将索引图像转换为 RGB
    if ~isempty(map)
        I = ind2rgb(I, map);
        I = im2uint8(I);
    end

    % 统一为 uint8 RGB/灰度
    if isa(I, 'double') || isa(I, 'single')
        I = im2uint8(I);
    end

    % 获取 image size and channels
    [H, W, C] = size(I);

    % 生成灰度图用于判断"白边"——优先使用 alpha 通道（若存在）
    if exist('alpha', 'var') && ~isempty(alpha)
        % alpha 通道存在：透明处不计为 tag
        alphaImg = alpha;
        if isa(alphaImg, 'logical')
            alphaMask = alphaImg;
        else
            alphaMask = alphaImg > 0; % 非透明算有效像素
        end
        % 如果是 RGB，把 alpha 无关白区域也判为白
        if C == 3
            gray = rgb2gray(I);
        else
            gray = squeeze(I);
        end
        % 把 alpha==0 的像素强制设为白（255），避免被当成 tag
        gray(~alphaMask) = 255;
    else
        % 没有 alpha：直接计算灰度
        if C == 3
            gray = rgb2gray(I);
        else
            gray = squeeze(I);
        end
    end

    % 生成初始二值 mask：非白区域为目标
    % 注意：如果图像是反色（白底黑码），则目标为灰度 < threshold
    mask = gray < whiteThreshold;

    % 如果上面产生的 mask 非常稀疏，尝试降低阈值（更包容）
    if nnz(mask) < 10
        % 降低阈值一步
        mask = gray < (whiteThreshold - 30);
    end

    % 去小噪点，填孔
    mask = bwareaopen(mask, minComponentArea);
    mask = imclose(mask, strel('square', 3)); % 小闭运算填充边缘
    mask = imfill(mask, 'holes');

    % 如果 mask 全为 0，说明阈值没找到任何非白像素，尝试反向检测（处理反色图）
    if ~any(mask(:))
        % 尝试寻找暗色区域以外的最大连通域作为背景反色情形
        mask = gray > (255 - (whiteThreshold - 10));
        mask = bwareaopen(mask, minComponentArea);
        mask = imfill(mask, 'holes');
        if ~any(mask(:))
            fprintf('警告：%s 未检测到非白区域，跳过。 (%s)\n', files(k).name, fname);
            continue;
        else
            % 反色检测成功，取反：这里反转 mask（因为上面检测到的是白背景）
            mask = ~mask;
        end
    end

    % 选择最大的连通域（通常为 tag 本体）
    cc = bwconncomp(mask);
    if cc.NumObjects > 1
        stats = cellfun(@numel, cc.PixelIdxList);
        [~, idxmax] = max(stats);
        mainMask = false(size(mask));
        mainMask(cc.PixelIdxList{idxmax}) = true;
    else
        mainMask = mask;
    end

    % 获取主连通域的边界框
    props = regionprops(mainMask, 'BoundingBox');
    if isempty(props)
        fprintf('警告：%s 未找到有效连通域，跳过。\n', files(k).name);
        continue;
    end
    bbox = props(1).BoundingBox; % [x y width height]
    x1 = floor(bbox(1)) + 1;
    y1 = floor(bbox(2)) + 1;
    bw_tag = max(1, y1) : min(H, y1 + bbox(4) - 1);
    bx_tag = max(1, x1) : min(W, x1 + bbox(3) - 1);

    % 为保险起见，在 bbox 外延一点像素（防止裁剪到边界）
    pad = 0;
    x1p = max(1, x1 - pad);
    y1p = max(1, y1 - pad);
    x2p = min(W, x1 + bbox(3) - 1 + pad);
    y2p = min(H, y1 + bbox(4) - 1 + pad);

    cropped = I(y1p:y2p, x1p:x2p, :);

    % 将裁剪后的 tag 等比缩放到合适大小（考虑 marginPixels）
    [hCrop, wCrop, ~] = size(cropped);
    targetMax = outSize - 2 * marginPixels;
    if targetMax <= 0
        error('marginPixels 太大导致可用区域 <= 0，请减小 marginPixels。');
    end
    scale = targetMax / max(hCrop, wCrop);
    newH = max(1, round(hCrop * scale));
    newW = max(1, round(wCrop * scale));
    % 使用最近邻插值以保持锐利的方块边缘（AprilTag 很重要）
    resized = imresize(cropped, [newH, newW], 'nearest');

    % 若需要强制二值输出：基于灰度进行阈值化并扩展为三通道
    if forceBinaryOutput
        if size(resized,3) == 3
            g2 = rgb2gray(resized);
        else
            g2 = resized;
        end
        % 自动阈值（Otsu），但尽量和 whiteThreshold 联合使用以稳健
        t = graythresh(g2) * 255;
        % 选择更保守的阈值：取 min(t, whiteThreshold-1)
        thr = min(t, whiteThreshold - 1);
        bwres = g2 < thr;
        % 将二值扩展回三通道（黑=0，白=255）
        resized = uint8(~bwres) * 255; % ~bwres: 1->白  (so white=255)
        resized = repmat(resized, [1,1,3]);
    else
        % 保证 uint8
        if ~isa(resized, 'uint8')
            resized = im2uint8(resized);
        end
        if size(resized,3)==1
            resized = repmat(resized, [1,1,3]);
        end
    end

    % 在 1024x1024 画布中居中放置
    canvas = uint8(255 * ones(outSize, outSize, 3)); % 白底
    r0 = floor((outSize - newH)/2) + 1;
    c0 = floor((outSize - newW)/2) + 1;
    r1 = r0 + newH - 1;
    c1 = c0 + newW - 1;
    canvas(r0:r1, c0:c1, :) = resized;

    % 可选：如果原图是单通道灰度且未强制二值，保存为灰度 PNG
    % 为简单统一，统一保存为 RGB PNG

    outName = fullfile(outFolder, files(k).name);
    try
        imwrite(canvas, outName, 'PNG');
        fprintf('已保存：%s\n', outName);
    catch ME
        fprintf('保存 %s 失败：%s\n', outName, ME.message);
    end

    if debugShow
        figure(1); clf;
        subplot(2,2,1), imshow(I), title('原图');
        subplot(2,2,2), imshow(mainMask), title('检测到的主连通域');
        subplot(2,2,3), imshow(cropped), title('裁剪结果');
        subplot(2,2,4), imshow(canvas), title('最终 1024x1024');
        drawnow;
    end
end

fprintf('全部处理完成，输出保存在：%s\n', outFolder);