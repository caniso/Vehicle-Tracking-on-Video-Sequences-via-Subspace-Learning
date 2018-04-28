% This function is utilized in the vehicle tracking algorithm to get the
% needed threshold values for thresholding processes.
%
% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin

function [out1, out2] = thresholdingMethod(grayLevelVideo, ForegroundModel, BackgrounModel, frameAmount)
    
    MovingObjDiff(:, :, :) = abs(grayLevelVideo(:, :, 1 : frameAmount) - BackgrounModel(:, :, 1 : frameAmount));
    MovingObjSum = 0;
    for frameNum = 1 : frameAmount
        MovingObjSum = MovingObjDiff(:, :, frameNum) + MovingObjSum;
    end
    tempMaxVal = max(max(MovingObjSum));
    [x, y] = find(MovingObjSum == tempMaxVal);

    tempMat = ForegroundModel(x, :, :);
    profMat = squeeze(tempMat);
    meanVal = mean(mean(profMat));
    maxVal  = max(max(profMat));
    minVal  = min(min(profMat));
    % 'adj1' and 'adj2' are adjustment values. It depends on the video 
    % which is being processed. 
    % For 'sherbrooke' video  adj1 = 1.1  and adj2 = 1.46
    % For 'stMarc'     video  adj1 = 1.0  and adj2 = 1.2
    % For 'whitecar'   video  adj1 = 0.66 and adj2 = 1.0
    adj1 = 0.66;
    adj2 = 1.0;
    level1  = (meanVal + maxVal) * (1 / 2) * (adj1);
    level2  = (meanVal + minVal) * (1 / 2) * (adj2);

    out1 = level1;
    out2 = level2;
end