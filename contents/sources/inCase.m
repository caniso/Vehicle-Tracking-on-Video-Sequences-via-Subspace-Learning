%% See Moving Objects at RGB Space
% Convert 'BinaryVideo_Opened' as if it is RGB. Then, apply AND.
% for frameNum = 1 : Num_of_Frames
%     rgbFrames = Images(:, :, :, frameNum);
%     Mask = BinaryVideo_Opened(:, :, frameNum);
%     Mask_RGB = cat(3, Mask, Mask, Mask);
%     Video_RGB(:, :, :, frameNum) = rgbFrames .* (Mask_RGB & rgbFrames);
% end
% implay(Video_RGB)