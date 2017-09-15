
%% 3D Voxel Grid
% Inputs: Velodyne points and GPS/IMU localization
% Output: Static / Dynamic environment modeling
% Alireza Asvadi, 2015 July
%% clear memory & command window
clc
clear all
close all
%% setting 
st           = Fstt;
%% main
  st.st.st = 58;
  st.st.tn = 58;
for frame    =  st.st.st : st.st.tn;             % frame number 1: 25
close all
frame

%run_demoVelodyne (base_dir, calib_dir)

%% dynamic / static modeling    
[In, prm]    = Fint(st, frame);                  % ground parameters and voxelize integrate points 
Bm           = Fmdl(In.mat, prm, st, frame);     % remove dynamic voxels and build the background model
Fm           = Ffrg(Bm.mat, prm, st, frame);     % compute foreground voxels
%% discriminative analysis
[Bg, ~, ~]   = Fltr(Bm, Fm, st, 100);            % background model
[Fg, ~, ~]   = Fltr(Fm, Bm, st, 5);              % foreground model
%% plot

h = figure('units','normalized','outerposition',[0 0 1 1], 'Visible', 'off');
% figure('units','normalized','outerposition',[0 0 1 1])
%Fplot(st, Bg, Fg, prm, frame)  
% figure('units','normalized','outerposition',[0 0 1 1])
Fplot_fst(st, Bg, Fg, prm, frame)
% fpath = "C:\Users\uidn0952\Documents\GitHub\Final_dyn_sta\images";
% fname = strcat('voxels_frame_',  num2str(frame), '.png');
%saveas(gcf, strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png'));
% print -djpeg strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png');
saveas(h, strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png'));
pause(0.5)
% close all;
end

