
%% 3D Voxel Grid
% Inputs: Velodyne points and GPS/IMU localization
% Output: Static / Dynamic environment modeling
% Alireza Asvadi, 2015 July
%% clear memory & command window
clc; clear all; close all;
sequence_no = 9;
if_digits = 1;
%% setting 
st           = Fstt(sequence_no, if_digits);
%% main
st.st.st = 58;
%   st.st.tn = 1;
tic
for frame    =  st.st.st : st.st.tn;             % frame number 1: 25

disp(['Processing frame ', num2str(frame), ' out of ',  num2str(st.st.tn)]);

%run_demoVelodyne (base_dir, calib_dir)

%% dynamic / static modeling    
[In, prm]    = Fint(st, frame);                  % ground parameters and voxelize integrate points 
Bm           = Fmdl(In.mat, prm, st, frame);     % remove dynamic voxels and build the background model
Fm           = Ffrg(Bm.mat, prm, st, frame);     % compute foreground voxels
%% discriminative analysis
[Bg, ~, ~]   = Fltr(Bm, Fm, st, 100);           % background model
[Fg, ~, ~]   = Fltr(Fm,  Bm, st, 5);             % foreground model
%% plot

h = figure('units','normalized','outerposition',[0 0 1 1], 'Visible', 'off')
% figure('units','normalized','outerposition',[0 0 1 1])
%Fplot(st, Bg, Fg, prm, frame)  
% figure('units','normalized','outerposition',[0 0 1 1])

%% four image plot
Fplot_fst(st, Bg, Fg, prm, frame)
%Fplt(st, Bg, prm, frame, [1,0,0])
%%


fpath = st.dr.save;
% [status, msg, msgID] = mkdir(fullfile(fpath, 'sequence_0005_four_images_per_frame_images'));
fname = strcat(num2str(sprintf('%04d', sequence_no)),'_', num2str(sprintf('%03d', frame)), '_voxels with labels four images', '.jpg');
saveas(h, fullfile(fpath, fname));
clear h, Bg, Bm, Fg, Fm, In, prm;
toc
end

