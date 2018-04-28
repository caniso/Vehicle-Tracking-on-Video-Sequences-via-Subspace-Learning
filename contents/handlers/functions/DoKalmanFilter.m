function [ estimationPointsArr,RGBImg] = DoKalmanFilter( pointsArr, RGBImg , occParameters,occFlag )
%
%
%
%Article     : ieeexplore.com/
% Date       : 28.04.2018
% Authors    : Ýsmail Can Büyüktepe
% Institute  : Gebze Technical University
% Conference : SÝU 2018 (Signal Processing and Communucations Applications)
% Github     : 
%  
 
%% Initial value assignment of Kalman filter parameters
dt = 1;                                                                    % Sampling rate;                                                                            
initialCovariance = 1e6*eye(4);                                            % Definition of P_initial (Covariance Matrix)
A=[1 0 dt 0 ; 0 1 0 dt ; 0 0 1 0 ; 0 0 0 1];                               % State transition matrix;
B=[(dt^2/2) 0 ; 0 (dt^2/2); dt  0; 0 dt];                                  % Control matrix;
H=[1 0 0 0 ; 0 1 0 0];                                                     % measurement matrix; %Covariance matrix initialize
 
meas_noise_var = 15;                                                        % Sensor covariance amplitude
R = meas_noise_var*eye(2);                                                 % Sensor covariance noise
q_std = .5;                                                                % Process noise covariance amplitude
Q = q_std * (B*B');                                                        % process noise covariance
 
initialState =[pointsArr(:,1);2;2];                                        % the first value of the measurement was taken 
                                                                           % as the position value.x and y position
numbFrame = size(pointsArr,2);       
estimationPointsArr = zeros(4,numbFrame);                                   
estimationPointsArr(:,1)=initialState;
covMatrix=initialCovariance;
estimatePoint=initialState;
 
[boundaryX,boundaryY,~,~]= size(RGBImg);
 
offset=3;
Count=0;
Tresh=15;
OcclusionThresh = 75;

%% To start Kalman Filter Tracking
if occFlag   
RGBImg(occParameters(2):occParameters(2)+occParameters(4),...
    occParameters(1):occParameters(1)+occParameters(3),:,:) = 0;

for t=2:dt:numbFrame
   
   check=isnan(pointsArr(1:2,t));
    if ~check
        if (pointsArr(1,t)>= occParameters(1)) & (pointsArr(1,t)<=(occParameters(1)+occParameters(4))) ...
                & (pointsArr(2,t)>=occParameters(2)) & (pointsArr(2,t)<= (occParameters(2)+occParameters(3)))
            pointsArr(:,t)= NaN;
        end
    end
    
    if (estimatePoint(1)>(boundaryX-offset)) || (estimatePoint(2)> (boundaryY-offset)) || (estimatePoint(1)< (1+offset )) || (estimatePoint(2)< (1+offset))
        isTracking= false;
    else
         %To begin with Prediction section
         estimatePoint=A*estimatePoint;                                   
         covMatrix=A* covMatrix* A' + Q;                                    % Prediction of covariance matrix.
         tf = isnan(pointsArr(1,2:t));                                      % is there a measurement at time t  
         if ~tf                                                             % if there is a valid measurement
             Distance = norm(estimatePoint(1:2)- pointsArr(1:2,t));
             if Distance <= Tresh
                %Calculate the Kalman Gain
                K = covMatrix * H' * inv(H*covMatrix*H' + R);                 
                %Correction of prior estimation.
                corEstimationPoint = estimatePoint + K *( pointsArr(:,t) - (H * estimatePoint));     
                estimatePoint = corEstimationPoint;
                estimationPointsArr(:,t) = corEstimationPoint;             % estimation points total matrix
                %Correction of covarianceMatrix.
                postCovMatrix = (eye(4) - K*H) *covMatrix ;                 
                covMatrix = postCovMatrix;
             end
         else
             Count = Count + 1;                                            % for occulution counter
         end
         
         if Count <= OcclusionThresh
             isTracking=true;
             estimationPointsArr(:,t)=estimatePoint;
         else
             isTracking=false;
         end
    end
    
end %end for loop

else
    for t = 2 : dt :numbFrame

        if (estimatePoint(1) > boundaryX-offset) || (estimatePoint(2) > boundaryY-offset) || (estimatePoint(1) < 1+offset) || (estimatePoint(2) < 1+offset)
            isTracking = false;
        else
            %To begin with Prediction section
            estimatePoint = A*estimatePoint;                                % State transition matrix
            covMatrix = A* covMatrix*A' + Q ;                               % Prediction of covariance matrix.
            tf = isnan(pointsArr(1:2,t));                                   % is there a measurement at time t
            if ~tf                                                          % if there is a valid measurement
                Distance = norm(estimatePoint(1:2)-pointsArr(1:2,t));
                if Distance <= Tresh
                    %Calculate the Kalman Gain
                    K = covMatrix * H' * inv(H*covMatrix*H' + R);
                    %Correction of X_prediction.
                    corEstimationPoint = estimatePoint + K *( pointsArr(:,t) - (H * estimatePoint));
                    estimatePoint = corEstimationPoint;
                    estimationPointsArr(:,t) = corEstimationPoint; %prediction total matrix

                    %Correction of P_prediction.
                    postCovMatrix = (eye(4) - K*H) *covMatrix ;
                    covMatrix = postCovMatrix;
                end
            else
                Count = Count + 1;              % for occlusion
            end
            if Count <= OcclusionThresh
                isTracking = true;
                estimationPointsArr(:,t) = estimatePoint;
            else
                isTracking = false;
            end
        end

    end
        
end %end if statement

 
end % end Function
