function [  ] = showOutput( estimationPointsArr,RGBImg,numFrame )
%UNTÝTLED Summary of this function goes here
%   Detailed explanation goes here

trackingVideo=zeros(size(RGBImg));
trackingVideo(:,:,:,1) = RGBImg(:,:,:,1);
 
for t=2:1:numFrame
     if estimationPointsArr(1,t)~=0 & estimationPointsArr(2,t)~=0
            trackingVideo(:,:,:,t)=insertMarker(RGBImg(:,:,:,t),estimationPointsArr(1:2,t)','color','red','size',10 );
        else
            trackingVideo(:,:,:,t)=RGBImg(:,:,:,t);
     end
end
implay(trackingVideo);
end  %function end

