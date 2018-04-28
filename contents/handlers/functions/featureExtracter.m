% This function is utilized in the vehicle tracking algorithm to detect the
% vehicle in every frame which is the one that operator wants to track.
% Also this functions gets the centroids of the detected vehicles which
% will be fed to the Kalman filter for tracking purpose.
%
% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin

function [out1] = featureExtracter(BinaryVideo, VideoFrames, Orig_Patch_Height, Orig_Patch_Width, Orig_Features)
    
    [~, ~, Num_of_Frames] = size(BinaryVideo);
    points_array = [];
    
    for frameNum = 1 : Num_of_Frames
        cc = bwconncomp(BinaryVideo(:, :, frameNum));
        stats = regionprops(cc, 'BoundingBox', 'Centroid');
        DistArr = 0;
        if cc.NumObjects ~= 0
            for objectNum = 1 : cc.NumObjects
                xMin   = uint16(round(stats(objectNum).BoundingBox(1)));
                yMin   = uint16(round(stats(objectNum).BoundingBox(2)));
                width  = uint16(round(stats(objectNum).BoundingBox(3)));
                height = uint16(round(stats(objectNum).BoundingBox(4)));

                patch = imresize(VideoFrames(yMin : yMin + height - 1, ...
                   xMin : xMin + width - 1, :, frameNum), ...
                                [Orig_Patch_Height, Orig_Patch_Width]);
       
                featureVector = extractHOGFeatures(patch);
                % euclidian distance
                DistArr(objectNum) = sqrt(sum((featureVector - Orig_Features) .^ 2)); 
            end
    
            [~, x] = min(DistArr);
            point(frameNum, :) = stats(x).Centroid;
        else
            point(frameNum, :) = NaN;
        end
        points_array = [points_array, point(frameNum, :)'];
        %Detection_RGB = insertMarker(VideoFrames(:, :, :, frameNum), ...
                       % point(frameNum, :), 'color', 'red', 'size', 3);
        %FinalVideoforDetection(:, :, :, frameNum) = Detection_RGB;    
    end
    
    out1 = points_array;
    %out2 = FinalVideoforDetection;
end