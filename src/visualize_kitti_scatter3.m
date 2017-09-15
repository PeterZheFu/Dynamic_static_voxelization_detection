function [] = visualize_kitti_scatter3(X, my_title, my_frame_no)
x = X(:, 1);
y = X(:, 2);
z = X(:, 3);

figure('units','normalized','outerposition',[0 0 1 1])

scatter3(x, y, z, 1, z);
%
hold on
plotcube([30 20 0.05],[ -5 -10 0],.2,[1 0 0]); 
plotcube([30 0.05 5], [-5 -10 -3], .1, [0 1 0]);
plotcube([50 0.05 5], [-5 10 -3], .1, [0 1 0]);


view(-85, 23);

xlabel('X')
ylabel('Y')
zlabel('Z')
%set(gca,'XLim',[-80 80],'YLim',[-70 70],'ZLim',[-2 6])
title([my_title, ' frame no ', num2str(my_frame_no)]);




