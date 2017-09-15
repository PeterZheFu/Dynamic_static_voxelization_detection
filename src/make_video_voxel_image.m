% Convert Between Image Sequences and Video
% This example shows how to convert between video files and sequences of image 
% files using VideoReader and VideoWriter.
% 
% The sample file named |shuttle.avi| contains 121 frames. Convert the frames 
% to image files using VideoReader and the |imwrite| function. Then, convert the 
% image files to an AVI file using VideoWriter.
% 
% Copyright 2014 The MathWorks, Inc.
% % Setup
% Create a temporary working folder to store the image sequence.
%
% workingDir = tempname;
% mkdir(workingDir)
% mkdir(workingDir,'images')

close all; clear all

%%
workingDir = 'C:\Users\uidn0952\Documents\GitHub\Final_dyn_sta\video';
% imageNames = dir("C:\Users\uidn0952\Documents\GitHub\Final_dyn_sta\images\video\")
imageNames = dir(fullfile(workingDir,'images','*.png'));
imageNames = {imageNames.name}';

%%
%% Create New Video with the Image Sequence
% Construct a VideoWriter object, which creates a Motion-JPEG AVI file by default.
%%
outputVideo = VideoWriter(fullfile(workingDir,'shuttle_out.avi'));
outputVideo.FrameRate = 12;
open(outputVideo)
%% 
% Loop through the image sequence, load each image, and then write it to 
% the video.
%%
for ii = 1:length(imageNames)
    ii
   img = imread(fullfile(workingDir,'images',imageNames{ii}));
   %img_down = img(1:2:end,1:2:end,:);
   img_down = imresize(img, 0.7);
   writeVideo(outputVideo,img_down)
end


%% 
% Finalize the video file.
%%
close(outputVideo)
%% View the Final Video
% Construct a reader object.
%%
shuttleAvi = VideoReader(fullfile(workingDir,'shuttle_out.avi'));
%% 
% Create a MATLAB movie struct from the video frames.
%%
ii = 1;
while hasFrame(shuttleAvi)
   mov(ii) = im2frame(readFrame(shuttleAvi));
   ii = ii+1;
end
%% 
% Resize the current figure and axes based on the video's width and height, 
% and view the first frame of the movie.
%%
figure 
imshow(mov(1).cdata, 'Border', 'tight') 
%% 
% Play back the movie once at the video's frame rate.
%%
movie(mov,1,shuttleAvi.FrameRate)
%% Credits
% Video of the Space Shuttle courtesy of NASA.