% In this work, we aimed to develop a novel vehicle tracking algorithm by
% using FPCP (Fast Principal Component Pursuit) for background modeling and
% Kalman filter for tracking purposes. In this main source file comments
% are used to explain details of our algorithm. If you want to have a more
% detailed explanation please visit the IEEE Xplore web site and search for
% 'Vehicle Tracking on Video Sequences via Subspace Learning' article 
% (as it is given below):
%
% Article    : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Abdülhakim Gültekin and Ýsmail Can Büyüktepe
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : github.com/AbdulhakimGultekin
%% Import Related Functions
clc, clearvars, close all;

foldername = '..\handlers';                                % DO NOT CHANGE
foldername_functions = '..\handlers\functions';            % DO NOT CHANGE
foldername_videos = '..\handlers\videos';                  % DO NOT CHANGE
addpath(foldername, foldername_functions, foldername_videos);
%% Video Frames' Acquisition
tic;
video = VideoReader('whitecar.mp4');
frameNum = 1;
while hasFrame(video)
    Images(:, :, :, frameNum) = im2double(readFrame(video));
    grayVideo(:, :, frameNum) = rgb2gray(Images(:, :, :, frameNum));
    frameNum = frameNum + 1;
end 
Num_of_Frames = frameNum - 1;
%% Vehicle Pointing
% In this section, operator selects the vehicle, that it wants to track,
% by mouse. HOG features are extracted from the RGB video frames at the
% selected region.
occlusionFlag = 1;          % If needed set as 1.
[
OriginalFeatures,      ...
original_patch_height, ...
original_patch_width,  ...
OcclusionParameters    ...  % For cases it is needed.
] = targetPointer(Images(:, :, :, 1), occlusionFlag);

%% Pre-process(Gamma Correction and Smoothing)
% Gamma correction is applied on all frames with 0.5 gamma value.
% Regard that the pixel values are double numbers in the range of [0 1].
for frameNum = 1 : Num_of_Frames
    grayVideo(:, :, frameNum) = imadjust(grayVideo(:, :, frameNum), [], [], 0.5);
end
% Smoothing mask applied to remove noise effect.
averaging_mask = ones(3) / 9;
grayVideo(:, :, :) = imfilter(grayVideo(:, :, :), averaging_mask);
%% Using Fast Principal Component Pursuit(FPCP) for Background Modeling
[Foreground, Background] = completed_fastpcp(grayVideo);
%% Pixel Values of a Specific Row/Column for Thresholding 
% This algorithm has been generated for vehicle detection. On this section,
% a specific row/column over which cars pass is considered to 
% determine threshold values. Row/column vectors is taken from every
% frame of foreground model. Mean values of found values are utilized to
% get threshold values.
frameNum_for_thresholding = 10;
[level1, level2] = thresholdingMethod(grayVideo, Foreground, Background, frameNum_for_thresholding);
%% Find Moving Objects 
% Threshold the foreground model once for light objects and once for darks.
% Then, apply OR operation to get desired output.
for frameNum = 1 : Num_of_Frames
    BW1(:, :, frameNum) = imbinarize(Foreground(:, :, frameNum), level1);
    BW2(:, :, frameNum) = Foreground(:, :, frameNum) <= level2;
    BinaryVideo(:, :, frameNum) = BW1(:, :, frameNum) | BW2(:, :, frameNum);
end
% implay(BinaryVideo);
%% Morphological Operation
se1 = strel('rectangle', [10 5]);
BinaryVideo_Closed = imclose(BinaryVideo, se1);
% implay(BinaryVideo_Closed);

%  If needed, apply 'opening' process on the closed binary video.
% se2 = strel('rectangle', [5 5]);
% BinaryVideo_Opened = imopen(BinaryVideo_Closed, se2);
% implay(BinaryVideo_Opened);
%% Representation of Detections
% To visualize detections on every frame uncomment the code below.
% seeDetections(grayVideo, BinaryVideo_Closed);
%% Feature Extraction for Tracking
% In this section, HOG features are extracted for every blob in 
% every frame. By using 'euclidian distance' method, 
% the similarity between these extracted features
% and the features extracted at the beginning of the algorithm is 
% evaluated. Using this similarity, the centroid of the blob which is 
% closest to the chosen one is regarded as the point which will be feeded to 
% the Kalman filter.
[feeding_points] = ...
featureExtracter       ...
(                      ...
BinaryVideo_Closed,    ...
Images,                ...
original_patch_height, ...
original_patch_width,  ...
OriginalFeatures       ...
);

[estimationPointsArr,RGBImg] = DoKalmanFilter(feeding_points,Images,OcclusionParameters,occlusionFlag);

% So as to visualize detection results, play 'DetectionVideo' by uncommenting
% the code two rows below.
toc;
%implay(detectionVideo);
showOutput( estimationPointsArr,RGBImg,Num_of_Frames );