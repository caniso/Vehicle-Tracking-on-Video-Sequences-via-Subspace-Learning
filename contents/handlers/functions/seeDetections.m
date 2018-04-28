% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin

function seeDetections(video, binaryVideo)
    
    [~, ~, Num_of_Frames] = size(binaryVideo);
    figure;
    for frameNum = 1 : Num_of_Frames
        cc = bwconncomp(binaryVideo(:, :, frameNum));
        stats = regionprops(cc, 'BoundingBox');
        imshow(video(:, :, frameNum)), title('Detected Objects');
        for objectNum = 1 : cc.NumObjects
            rectangle('Position', stats(objectNum).BoundingBox, ...
                                  'EdgeColor', 'r', 'LineWidth', 1);
        end
        pause(0.0005)
    end
end