clc; clear;

nx = 100; ny = 100;
obstacle = ones(nx, ny);

% rngstate = rng();
% rng(rngstate)
% save("rnd_2","rngstate");
% Comment those two lines to change the randomly generated environment
load rnd_2
rng(rngstate)

% obstacle(55:75,60:85) = 0;

dx = 1; dy = 1;

nb_of_obstacles = 15;
min_width = 2*1/dy; max_width = 20*1/dy;
min_height = 2*1/dx; max_height = 20*1/dx;

sp_o = [5 5];
ep_o = [10 90];

obstacle(1:40, 15:30) = 0;

for i = 1:nb_of_obstacles
    row_1 = 0 + randi(nx);
    row_2 = row_1 + min_width + randi(max_width - min_width);
    row_1 = min(row_1, nx); row_2 = min(row_2, nx);

    col_1 = 0 + randi(ny);
    col_2 = col_1 + min_height + randi(max_height - min_height);
    col_1 = min(col_1, ny); col_2 = min(col_2, ny);

    cond_1 = (row_1 <= sp_o(2)) && (sp_o(2) <= row_2);
    cond_2 = (col_1 <= sp_o(1)) && (sp_o(1) <= col_2);

    cond_3 = (row_1 <= ep_o(2)) && (ep_o(2) <= row_2);
    cond_4 = (col_1 <= ep_o(1)) && (ep_o(1) <= col_2);

    if  ~((cond_1 && cond_2) || (cond_3 && cond_4))
        obstacle(row_1:row_2,col_1:col_2) = 0;
    end
end


% Convert the image to a speed map
SpeedImage=obstacle+0.001;

% Set the source to end of the maze
SourcePoint=[sp_o(1); sp_o(2)];

% Calculate the distance map (distance to source)
tic
DistanceMap= msfm2d(SpeedImage, SourcePoint, false, false); 
toc

x = meshgrid(1:100, 1:100); y = x';
ground_truth = sqrt((x-50).^2 + (y-50).^2);
difference = DistanceMap-ground_truth;
minimum = min(min(difference));
maximum = max(max(difference));

% clf
% mesh(difference,'FaceLighting','phong','FaceColor','interp',...
%     'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
% colormap(jet)
% view(0,90)
% axis equal
% axis([1 ny 1 nx])
% hold on
% 
% contour3(difference, linspace(minimum, maximum, 20), 'LineWidth', 2,...
%         'EdgeColor', 'k');
% 
% grid off
% set(gca, 'xtick', [-1e6 1e6]);
% set(gca, 'ytick', [-1e6 1e6]);
% set(gca,'LooseInset',get(gca,'TightInset'));

addpath(genpath('functions'))
addpath(genpath('shortestpath'))
StartPoint = [ep_o(1); ep_o(2)]; % [97; 91];
tic
ShortestLine=shortestpath(DistanceMap,StartPoint,SourcePoint);
toc

minimum = 0; %min(min(DistanceMap));
maximum = 165; % DistanceMap(100, 100); % max(max(DistanceMap));
DistanceMap(DistanceMap>=maximum) = 0;
clf
mesh(DistanceMap,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(jet)
clim([0 maximum])
view(0,90)
axis equal
axis([1 ny 1 nx])
hold on

contour3(DistanceMap, linspace(minimum, maximum, 40), 'LineWidth', 2,...
        'EdgeColor', 'k');

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

% imwrite(obstacle, "compare_1.png");
% obstacle = imread("compare_1.png");
% obstacle = imrotate(obstacle,90);
% obstacle = flip(obstacle);
% imwrite(obstacle, "compare_1.png");

plot3(sp_o(2), sp_o(1), 550,'o',...
'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
'MarkerSize', 12, 'LineWidth', 1)

plot3(ep_o(2), ep_o(1), DistanceMap(ep_o(1), ep_o(2))+255,'o',...
'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
'MarkerSize', 12, 'LineWidth', 1)

% Plot the shortest route
hold on
elevation = zeros(size(ShortestLine,1),1);
for i = 1:size(ShortestLine,1)
    elevation(i) = DistanceMap(round(ShortestLine(i,1)),...
        round(ShortestLine(i,2))) + 225;
end
hold on, plot3(ShortestLine(:,2),ShortestLine(:,1),elevation,...
    'LineWidth', 3,'color','magenta')

%% Expfig
% addpath(genpath('expfig'))
% export_fig FMM_x -r400 -transparent -png
% % export_fig FMM_1 -transparent -png -native -preserve_size