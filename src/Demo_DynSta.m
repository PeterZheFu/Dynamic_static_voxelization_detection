
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
  st.st.st = 1;
  st.st.tn = 1;
for frame    =  st.st.st : st.st.tn;             % frame number 1: 25

disp(['Processing frame ', num2str(frame), ' out of ',  num2str(st.st.tn)]);

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
%%

% fpath = 'C:\Users\FU000\Documents\GitHub\Dynamic_static_voxelization_detection';

fpath = st.dr.mdr; %'~/continental/kitti/2011_09_26/2011_09_26_drive_0005_sync';
[status, msg, msgID] = mkdir(fullfile(fpath, 'sequence_0005_four_images_per_frame_images'));
fname = strcat('labeled_voxels_frame_', sprintf('%03d', num2str(frame)), '.jpg');
saveas(h, fullfile(fpath, 'sequence_0005_four_images_per_frame_images', fname))

%%
%saveas(gcf, strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png'));
% print -djpeg strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png');
% saveas(h, strcat('voxels_and_image_frame_', sprintf('%03d', frame), '.png'));  %fullfile(fname, filename), 'jpeg'
% close all;
end

