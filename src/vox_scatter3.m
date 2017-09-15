function [] = vox_scatter3(X, my_title, frame)
x = X(:, 1);
y = X(:, 2);
z = X(:, 3);

figure('units','normalized','outerposition',[0 0 1 1])

scatter3(x, y, z, 1, z);

view(-76, 15)

xlabel('X')
ylabel('Y')
zlabel('Z')
title([my_title, ' frame no ', num2str(frame)])





%set(gca,'XLim',[-80 80],'YLim',[-70 70],'ZLim',[-2 6])
% title([my_title, ' frame no ', num2str(my_frame_no)])




