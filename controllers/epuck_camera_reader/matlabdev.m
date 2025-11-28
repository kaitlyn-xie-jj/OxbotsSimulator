I = imread("../../cache/camera/temp_img.png");
tagFamily = "tag36h11";
[id,loc,detectedFamily] = readAprilTag(I,tagFamily);
for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily(idx));
 
        % Insert markers to indicate the locations
        markerRadius = 2;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx),repmat(markerRadius,numCorners,1)];
        I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
        imwrite(I,"../../cache/camera/temp_img_recognised.png");
end
imshow(I);