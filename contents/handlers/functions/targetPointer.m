% This function is utilized in the vehicle tracking algorithm to select the
% vehicle which will be tracked and get the HOG features of the selected
% region.
%
% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin

function [out1, out2, out3, varargout] = targetPointer(VideoFrames, occFlag)
    image = VideoFrames;
    figure, imshow(image);
    rectangular = getrect;
    rectangular = uint16(rectangular);
    xMin        = rectangular(1);
    yMin        = rectangular(2);
    width       = rectangular(3);
    height      = rectangular(4);
    patch = image(yMin : yMin + height - 1, xMin : xMin + width - 1, :);

    % If needed, create an occlusion on the image.
    if occFlag
       Occlusion       = getrect;
       Occlusion       = uint16(Occlusion);
       xMinOcclusion   = Occlusion(1);
       yMinOcclusion   = Occlusion(2);
       widthOcclusion  = Occlusion(3);
       heightOcclusion = Occlusion(4);
       varargout{1}    = ...
           [xMinOcclusion, yMinOcclusion, widthOcclusion, heightOcclusion];
    else
       varargout{1} = [];
    end

    % HOG feature extraction
    [OriginalFeatures, hogVisual] = extractHOGFeatures(patch);
    % figure, imshow(patch), hold on, plot(hogVisual);
    figure, imshow(patch);

    out1 = OriginalFeatures;
    out2 = height;
    out3 = width;
end