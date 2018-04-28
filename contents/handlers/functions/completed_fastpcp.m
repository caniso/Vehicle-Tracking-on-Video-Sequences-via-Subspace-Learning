% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin

function [out1, out2] = completed_fastpcp(grayVideo)
    % A pre-process as reshaping has to be done before using 'FPCP'
    % function.
    [~, ~, Num_of_Frames] = size(grayVideo);
    for frameNum = 1 : Num_of_Frames
        I = grayVideo(:, :, frameNum);
        [m, n] = size(I);
        V(:, frameNum) = reshape(I, [m * n, 1]);
    end
    
    lambda = 1 / sqrt(max(size(V)));
    [L, S, S_P] = fastpcp(V, lambda);
    Foreground = mat2gray(reshape(S, [m, n, Num_of_Frames]));
    Background = reshape(L, [m, n, Num_of_Frames]);
    
    out1 = Foreground;
    out2 = Background;
end