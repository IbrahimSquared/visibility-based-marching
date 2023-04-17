% Load a maze image
I1=im2double(imread('images/maze.gif'));
[nx, ny] = size(I1);
% Convert the image to a speed map
SpeedImage=I1*1000+0.0001;
% Set the source to end of the maze
SourcePoint=[792;11];
% Calculate the distance map (distance to source)
tic
DistanceMap= msfm2d(SpeedImage, SourcePoint, true, true); 
toc
% Show the distance map
% figure, imshow(DistanceMap,[0 3400])
clf
mesh(DistanceMap,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(jet)
view(0,90)
axis([1 nx 1 ny])
hold on

% Trace shortestline from StartPoint to SourcePoint
% StartPoint=[9;14];
StartPoint = [449;203];
tic
ShortestLine=shortestpath(DistanceMap,StartPoint,SourcePoint);
toc
% Plot the shortest route
% hold on, plot(ShortestLine(:,2),ShortestLine(:,1),'r','LineWidth',3)
hold on
elevation = zeros(size(ShortestLine,1),1);
for i = 1:size(ShortestLine,1)
    elevation(i) = DistanceMap(round(ShortestLine(i,1)),...
        round(ShortestLine(i,2))) + 10;
end
hold on, plot3(ShortestLine(:,2),ShortestLine(:,1),elevation,...
    'LineWidth',3,'color','black')


d_3 = 0;
for i = 1:size(ShortestLine,1)-1
    d_3 = d_3 + norm(ShortestLine(i+1,:)-ShortestLine(i,:));
end
d_3

% clf
% mesh(SpeedImage,'FaceLighting','phong','FaceColor','interp',...
% 'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
% colormap(jet)
% view(0,90)