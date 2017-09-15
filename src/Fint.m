function [In, prm] = Fint(st, frame)

% use temporal information: integrate points
% output: hst.[pts] cropped integrated points

%% load and integrate points
ref             = FIloa(st, frame);                      % use current coordinate as a reference coordinate 
hst.pts         = [];                                    % integrated points (history)
for frm         = frame : -1 : frame - st.rd.dl + 1      % go over previous frames [number of frames to integrate]
    if  frm         > 0                                      % minimum start farme is 1
        prv             = FIloa(st, frm);                        % load points
        
        prv             = FItns(prv, ref);                       % transform to the current coordinate
        hst.pts         = [hst.pts; prv.prj];                    % integrate points: insert new points        
        
    end
end
%visualize_kitti_scatter3(hst.pts, 'Uncropped velodyne points to be inside the local grid and image (hst.ptn)', frame - 1);
hst             = FIcrp(hst, st, frame);                 % crop points to the inside the local grid and image
% visualize_kitti_scatter3(hst.pts, 'Integrated points from history (hst.pts)', frame);

prm             = FIprm(hst.pts, st);                    % compute surface pieces' parameters                  
In              = Fvox(prm, st, hst.pts, frame);
% PATCH_3Darray(In.mat.occ)

end

function pts    = FIloa(st, frame)

% simple load
% output: pts.[pts ptn rtn trn] all points 

%% transformation matrixes [rotation 3x3, translation 3x1]
transform_all       = st.dt.pose(:, :);                                     % transformation matrix in camera coordinate
transform = transform_all{frame};

pts.rtn         = transform(1:3, 1:3);                                         % rotation    3x3
pts.trn         = transform(1:3, 4);                                           % translation 3x1
%% velodyne points [x, y, z, r] total number of pointsx4
fid.pts         = fopen(sprintf('%s\\%010d.bin', st.dr.pts, frame - 1), 'rb');   % read from directory of points (number of frames in each seq.)
velodyne        = fread(fid.pts, [4 inf], 'single')';                          % velodyne points [x, y, z, r] (total number of pointsx4)
fclose(fid.pts);                                                               % close fid
pts.pts         = velodyne(:, 1:3);                                            % velodyne points [x, y, z] (total number of pointsx3)
pts.pts(:,3) = pts.pts(:,3) + st.bias;
pts.ptn         = pts.pts * pts.rtn' + repmat(pts.trn', size(pts.pts, 1), 1);  % transformed velodyne points (Xp = RX + T)


end

function prv    = FItns(prv, ref)                                                % previous and reference(current)

% transform previous points on the current coordinates
% output: prv.[new] that is projected points on the current coordinate

%% translate and rotate previous points using current transformation
prv.prj(:, 1:3) = prv.ptn(:, 1:3) - repmat(ref.trn', size(prv.pts, 1), 1);     % translation
prv.prj(:, 1:3) = (ref.rtn \ prv.prj(:, 1:3)')';                               % rotation

end

function hst    = FIcrp(hst, st, frame)

% crop points to the inside the local grid and image
% output: hst.[pts] that is croped integrated points

%% velodyne points [x, y, z, r total number of pointsx4]
ins.grd         = ((hst.pts(:,1) > (st.vm.xb)) & (hst.pts(:,1) < st.vm.xf) & ...       % filter points to inside
                   (hst.pts(:,2) > (st.vm.yr)) & (hst.pts(:,2) < st.vm.yl) & ...       % /outside the local grid
                   (hst.pts(:,3) > (st.vm.zd)) & (hst.pts(:,3) < st.vm.zu));
hst.pts         = hst.pts(ins.grd, 1:3);                                             % velodyne points in local grid           

% remove all points behind image plane (approximation), commented as they
% may affect closest ground stripe plane estimation
% idx = hst.pts(:,1)<5;
% hst.pts(idx,:) = [];

%visualize_kitti_scatter3(hst.pts, 'hst.pts', frame)


%% incorporate image and color data [pts col ref pxs]
% wrong calculation pixel           = hst.pts * st.dt.clb;                                               % velodyne points on image plane

% p_in = hst.pts;
% p_in(:,4) = 1;
% pixel = (st.dt.clb * p_in')';
% 
% 
% pixel(:, 1)     = pixel(:, 1)./pixel(:, 3); 
% pixel(:, 2) = pixel(:, 2)./pixel(:, 3);  % point's x & y are cor. to image's c & nr - r (nr: nnumber of raws)
% %pixel           = round(pixel(:, 1:2));                                              % interpolate (it is not that much precise, round is enough!)
% 
% image           = imread(sprintf('%s\\%010d.png', st.dr.img, frame - 1));              % load image (number of frames in each seq.)
% ins.img         = (pixel(:, 1) >= 1) & (pixel(:, 1) <= size(image, 2)) & (pixel ...  % index of pixels inside grid and image
%                   (:, 2) >= 1) & (pixel(:, 2) <= size(image, 1));
% hst.pts         = hst.pts(ins.img, :);                                               % velodyne points in local grid & image
% 
% 
% 
% velo_img = project(hst.pts, st.dt.clb);


% fig = figure('Position',[20 100 size(image,2) size(image,1)]); axes('Position',[0 0 1 1]);
% imshow(image); hold on;
% cols = jet;
% for i=1:size(velo_img,1)
%   i
%   col_idx = round(64*5/hst.pts(i,1))
%   plot(velo_img(i,1),velo_img(i,2),'o','LineWidth',4,'MarkerSize',1,'Color',cols(col_idx,:));
% end


end

function prm   = FIprm(pts, st)

% compute surface pieces' parameters
% output: parameters
% clf
%% compute each surface piece parameters using least square method 
prm            = zeros(st.rd.no, 3);                                          % keep pieces' slopes 
base           = -st.bias;                                                    % find base of each piece for outlier rejection task 
for pci        = 1 : st.rd.no                                                 % index of the first piece : index of the last piece
    sp             = st.vm.xb + (pci - 1) * st.rd.pc;                             % start of the current piece (sp) x   
    ep             = sp + st.rd.pc;                                               % end of the current piece (ep) x
    pc             = pts((pts(:, 1) > sp) & (pts(:, 1) < ep), :);                 % take the current piece's points
    pc((pc(:, 3) < base - st.rd.ou) | (pc(:, 3) > base + st.rd.ou), :) = [];      % reject outlier
    pln            = ([pc(:, 1) pc(:, 2) ones(size(pc, 1), 1)] \ pc(:, 3))';      % fit a plane
    prm(pci, : )   = pln;                                                         % put the plane coefficient in the row of slope matrix 
    %% check to see if two consecutive pieces' parameters are acceptable or not 
    if pci         > 1                                                            % if the next piece is not valid use previous piece
        if (abs(atan(prm(pci, 1 )) - atan(prm(pci - 1, 1 ))) > st.rd.sd) || ...       % if the slope dif. between previous plane and current piece is more
           (sqrt(sum((prm(pci, : ) - prm(pci - 1, : )).^2)) > st.rd.vd)               % than 15 degree or the total dif. is more than a threshold
        prm(pci, : )   = prm(pci - 1, : );                                            % put the previous plane coeffient instead of new one
        pln            = prm(pci - 1, : );                                            % use the previous plane coeffient
        end    
    end
    base           = pln(1) * ep + pln(3);       % compute base for the next piece for outlier rejection task
    %% plot
%     figure
%     visualize_kitti_scatter3(pc, 'pp, ground points', 0)
%     t              = 1;
%     [xx, yy]       = meshgrid(sp : t : ep, st.vm.yr : t : st.vm.yl);
%     zz             = pln(1) * xx + pln(2) * yy + pln(3);
%     hold on
%     surf(xx, yy, zz, 'edgecolor', 'none')
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
%     hold off

end
end

function In    = Fvox(prm, st, input, frame)

%% remove ground points 
pts.pts        = [];
for pci        = 1 : st.rd.no    % index of the first piece : index of the last piece
    sp             = st.vm.xb + (pci - 1) * st.rd.pc;     % start of the current piece (sp) x   
    ep             = sp + st.rd.pc;   
    % end of the current piece (ep) x
    pc             = input((input(:, 1) > sp) & (input(:, 1) < ep), :);                            % take the current piece's points
    pln            = prm(pci, :);      % take the current estimated piece's parameter
    nrm            = cross([0 0 pln(3)] - [1 1 sum(pln)], [0 0 pln(3)] - [0 1 pln(2) + pln(3)]);   % surface normal (the slope of normal line) 

    %draw surface normal
    v1=[0,0, 4 ];
    v2=[nrm(1), nrm(2), nrm(3)];
    v=[v2;v1];
    
    
    t              = (pc(:, 3) - pln(1) .* pc(:,1) - pln(2) * pc(:, 2) - pln(3)) ...               % t is a variable
                     ./ ( pln(1) .* nrm(1)+ pln(2) .* nrm(2) - nrm(3));
    pp             = [pc(:, 1) + nrm(1) .* t, pc(:, 2) + nrm(2) * t, pc(:, 3) + nrm(3) * t];       % projected points on the surface
    %pp(:,3) = pp(:,3) - st.bias;
%     visualize_kitti_scatter3(pp, 'pp, ground points', 0)
%     plot3(v(:,1),v(:,2),v(:,3),'r')
    x = pc(:,1);
    y = pc(:,2);
    z = pc(:,3);
%     scatter3(x, y, z, 1, z);
%     title(['ground points for stripe number ', num2str(pci)])
    
    id             = (pc(:, 3) - pp(:, 3)) < (st.rd.rm ); % | (abs((pc(:, 3) - pp(:, 3))) < (st.rd.rm + st.bias)); % remove points under height                      
    pc(id, :)      = []; 
    %pc(:, 3) = pc(:, 3) + st.bias;
    pts.pts        = [pts.pts; pc];  % velodyne points in local grid
%     visualize_kitti_scatter3(pts.pts, 'pts.pts, with ground points removed', 0)
%     plot3(v(:,1),v(:,2),v(:,3),'r')
    %title(['stripe number ', num2str(pci)])
    
end
%% voxelize points 
% pts.idx        = floor([(pts.pts(:,1) - st.vm.xb)/st.vx.x + 1, (pts.pts(:,2) - ...             % quantize and transform point's index 
%                  st.vm.yr)/st.vx.y + 1, (pts.pts(:,3) - st.vm.zd) / st.vx.z + 1]);

pts.idx        = floor([(pts.pts(:,1) - st.vm.xb)/st.vx.x + 1, (pts.pts(:,2) - ...             % quantize and transform point's index 
                 st.vm.yr)/st.vx.y + 1, (pts.pts(:,3)) / st.vx.z + 1]);
             
% vox_scatter3(pts.idx, 'pts.idx', frame)
pts.idx(pts.idx(:, 1) > st.vx.ix, 1) = st.vx.ix;            
% vox_scatter3(pts.idx, 'pts.idx', frame)
pts.idx(pts.idx(:, 2) > st.vx.iy, 2) = st.vx.iy; 
% vox_scatter3(pts.idx, 'pts.idx', frame)
%pts.idx(pts.idx(:, 3) > st.vx.iz, 3) = st.vx.iz;
pts.idx(pts.idx(:, 3) > st.vx.iz, 3) = st.vx.iz;
% vox_scatter3(pts.idx, 'pts.idx', frame)
min(pts.idx)
pts.idx = pts.idx(all(pts.idx>0, 2), :);

mat.occ        = accumarray(pts.idx, 1, [st.vx.ix, st.vx.iy, st.vx.iz]);   % matrix with number of points in each cell
% pts.ids        = floor([pts.pts(:,1) / st.vx.x, pts.pts(:,2) / st.vx.y, pts.pts(:,3) / st.vx.z]);    % quantize start point
% pts.ids(:, 1)  = pts.ids(:, 1) * st.vx.x;                                                      % x voxel start points in real coordinate
% pts.ids(:, 2)  = pts.ids(:, 2) * st.vx.y;                                                      % y voxel start points in real coordinate
% pts.ids(:, 3)  = pts.ids(:, 3) * st.vx.z;                                                      % z voxel start points in real coordinate
pts.ids        = (floor(pts.pts / st.vx.x)) * st.vx.x;                                         % fast 
% vox_scatter3(mat.occ, 'mat.occ',  frame)
% PATCH_3Darray(mat.occ)
[uni, idx, ~]  = unique(pts.idx, 'rows'); pts.uni = uni;                                       % unique indexes
unq            = pts.ids(idx, :);         pts.unq = unq;                                       % unique start point locations
%% compact
In.pts         = pts;
In.mat         = mat;

end
