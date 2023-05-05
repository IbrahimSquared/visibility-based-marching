clear; clc;
% Clear all figures
FigList = findall(groot, 'Type', 'figure');
for iFig = 1:numel(FigList)
    try
        clf(FigList(iFig));
    catch
        % Nothing to do
    end
end

% fid = fopen('visibility_based_global_path_planner.bat','w');
% fprintf(fid,'%s\n','set path=%path:C:\Program Files\MATLAB\R2022b\bin\win64;=%');
% fprintf(fid,'%s\n','visibility_based_global_path_planner.exe');
% fclose(fid);
system('visibility_based_global_path_planner.bat');
% return

%% Parser - parse settings

% Open the file for reading
fid = fopen('config/settings.config', 'r');

% Define the format string for textscan
formatSpec = '%s';

% Read the settings using textscan
settingsCell = textscan(fid, formatSpec, 'Delimiter', '=', 'CommentStyle', '#');

% Close the file
fclose(fid);

% Convert the cell array to a struct
settings = struct();
for i = 1:2:length(settingsCell{1})
    key = settingsCell{1}(i);
    value = settingsCell{1}(i+1);
    % Check if the value is a number
    if ~isnan(str2double(value{1}))
        % Convert the value to a number
        value = str2double(value{1});
    end
    % Save the key-value pair in the settings struct
    if strcmp(key{1}, 'imagePath') || strcmp(key{1}, 'initialFrontline')
        % Don't convert imagePath and initialFrontline to numbers
        settings.(key{1}) = value{1};
    else
        settings.(key{1}) = value;
    end
    
    if strcmp(key{1}, 'initialFrontline')
        settings.(key{1}) = eval(strrep(strrep(value{1}, '{', '['), '}', ']'));
    end
end

%% Visibility field (basically occupancy grid)
if settings.saveVisibilityField && settings.saveResults
    filename_visibilityField = "output/visibilityField.txt";
    T_visibilityField = readtable(filename_visibilityField,'Delimiter',' ');
    visibilityField = T_visibilityField.Variables;
    plotVisibilityField = false;
    
    if plotVisibilityField
        [nx, ny] = size(visibilityField);

        figure(6)
        set(gcf, 'Name', 'Visibility field (occupancy grid)')
        clf
        mesh(visibilityField,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(gray)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on

        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
    end
end

%% Vstar
if settings.vstar && settings.saveResults
    if settings.savePath
        filename_vstar_path = "output/vstar_path.txt";
        T_vstar_path = readtable(filename_vstar_path, 'Delimiter',' ');
        vstar_path = T_vstar_path.Variables;
        vstar_path = vstar_path + 1;
    end

    if settings.saveFScore
        filename_vstar_fScore = "output/vstar_fScore.txt";
        T_vstar_fScore = readtable(filename_vstar_fScore,'Delimiter',' ');
        vstar_fScore = T_vstar_fScore.Variables;
        [nx, ny] = size(vstar_fScore);
    end

    if settings.saveGScore
        filename_vstar_gScore = "output/vstar_gScore.txt";
        T_vstar_gScore = readtable(filename_vstar_gScore,'Delimiter',' ');
        vstar_gScore = T_vstar_gScore.Variables;
        vstar_gScore(isinf(vstar_gScore)) = 0;
        [nx, ny] = size(vstar_gScore);
    end

    if settings.saveLightSources
        filename_vstar_lightSources = "output/vstar_lightSources.txt";
        T_vstar_lightSources = readtable(filename_vstar_lightSources, 'Delimiter',' ');
        vstar_lightSources = T_vstar_lightSources.Variables;
        vstar_lightSources = vstar_lightSources + 1;
        plotLightSources = false;
    end

    if settings.savePath
        d_vstar = 0;
        elevation = zeros(size(vstar_path,1), 1);
        pt_old = vstar_path(1,:);
        for i = 1:size(vstar_path,1)
            pt = vstar_path(i,:);
            d_vstar = d_vstar + norm(pt-pt_old);
            elevation(i) = vstar_fScore(pt(end,1), pt(end,2))+10;
            pt_old = pt;
        end
    end

    if settings.saveFScore
        figure(15)
        set(gcf, 'Name', 'VStar FScore')
        clf
        mesh(vstar_fScore,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(jet)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on
        
        vstar_fScore(isinf(vstar_fScore)) = 0;
        minimum = min(min(vstar_fScore));
        maximum =  max(max(vstar_fScore));
        contour3(vstar_fScore+1, linspace(minimum, maximum, 40), 'LineWidth', 3,...
                'EdgeColor', 'k');
            
        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        
        if settings.savePath
            ep_o = [settings.target_x, settings.target_y] + 1;
            sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
            plot3(ep_o(2), ep_o(1), vstar_fScore(ep_o(1), ep_o(2))+1e2,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
            plot3(sp_o(2), sp_o(1), vstar_fScore(sp_o(1), sp_o(2))+1e2,'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 24, 'LineWidth', 3)
        end
    end
    
    if settings.saveGScore
        figure(1)
        set(gcf, 'Name', 'VStar GScore')
        minimum = min(min(vstar_gScore));
        maximum =  max(max(vstar_gScore));
        
        vstar_gScore(vstar_gScore==0) = inf;
        vstar_gScore(76, 76) = 0;
        clf
        mesh(vstar_gScore,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(jet)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on
        
        
        contour3(vstar_gScore+4, linspace(minimum, maximum, 40), 'LineWidth', 3,...
                'EdgeColor', 'k');

        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));

        if settings.saveLightSources && plotLightSources
            for i = 1:size(vstar_lightSources, 1)
                pt_ = vstar_lightSources(i,:);
                plot3(pt_(2), pt_(1), vstar_fScore(pt_(1),pt_(2))+10,'o',...
                    'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                    'MarkerSize', 10, 'LineWidth', 3)
            end
        end

        if settings.savePath
            plot3(vstar_path(:,2), vstar_path(:,1), elevation,...
                 'Color', 'magenta', 'LineWidth', 3);
            ep_o = [settings.target_x, settings.target_y] + 1;
            plot3(ep_o(2), ep_o(1), vstar_gScore(ep_o(1), ep_o(2))+1e2,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
            sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
            plot3(sp_o(2), sp_o(1), 0+1e2,'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 24, 'LineWidth', 3)

        end
    end
end

%% Astar
if settings.astar && settings.saveResults

    if settings.savePath
        filename_astar_path = "output/astar_path.txt";
        T_astar_path = readtable(filename_astar_path, 'Delimiter',' ');
        astar_path = T_astar_path.Variables;
        astar_path = astar_path + 1;
    end
    
    if settings.saveFScore
        filename_astar_fScore = "output/astar_fScore.txt";
        T_astar_fScore = readtable(filename_astar_fScore,'Delimiter',' ');
        astar_fScore = T_astar_fScore.Variables;
        [nx, ny] = size(astar_fScore);
    end

    if settings.saveGScore
        filename_astar_gScore = "output/astar_gScore.txt";
        T_astar_gScore = readtable(filename_astar_gScore,'Delimiter',' ');
        astar_gScore = T_astar_gScore.Variables;
        [nx, ny] = size(astar_gScore);
    end
    
    if settings.saveFScore
        figure(16)
        set(gcf, 'Name', 'AStar FScore')
        clf
        mesh(astar_fScore,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(jet)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on
        
        astar_fScore(isinf(astar_fScore)) = 0;
        minimum = min(min(astar_fScore));
        maximum =  max(max(astar_fScore));
        contour3(astar_fScore+1, linspace(minimum, maximum, 40), 'LineWidth', 3,...
                'EdgeColor', 'k');
            
        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        
        ep_o = [settings.target_x, settings.target_y] + 1;
        plot3(ep_o(2), ep_o(1), astar_fScore(ep_o(1), ep_o(2))+1e2,'o',...
            'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
            'MarkerSize', 24, 'LineWidth', 3)
        sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
        plot3(sp_o(2), sp_o(1), astar_fScore(sp_o(1), sp_o(2))+1e2,'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 24, 'LineWidth', 3)
    end

    if settings.savePath
        d_astar = 0;
        elevation = zeros(size(astar_path, 1), 1);
        pt_old = astar_path(1,:);
        for i = 1:size(astar_path,1)
            pt = astar_path(i,:);
            d_astar = d_astar + norm(pt-pt_old);
            elevation(i) = astar_fScore(pt(end,1), pt(end,2))+10;
            pt_old = pt;
        end
    end

    if settings.saveGScore
        figure(2)
        set(gcf, 'Name', 'Classic AStar solution')
        clf
        mesh(astar_gScore,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(jet)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on

        astar_gScore(isinf(astar_gScore)) = 0;
        minimum = min(min(astar_gScore));
        maximum =  max(max(astar_gScore));
        contour3(astar_gScore+2, linspace(minimum, maximum, 40), 'LineWidth', 3,...
                'EdgeColor', 'k');
        
        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        
        if settings.savePath
            plot3(astar_path(:,2), astar_path(:,1), elevation,...
             'Color', 'magenta', 'LineWidth', 3);
            ep_o = [settings.target_x, settings.target_y] + 1;
            sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
            plot3(ep_o(2), ep_o(1), astar_gScore(ep_o(1), ep_o(2))+1e3,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 1)
            plot3(sp_o(2), sp_o(1), astar_gScore(sp_o(1), sp_o(2))+1e3,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)

        end
    end
end

%% Distance function
if settings.distanceFunction && settings.saveResults
    filename_distanceFunction = "output/distanceFunction.txt";
    T_distanceFunction = readtable(filename_distanceFunction,'Delimiter',' ');
    distanceFunction = T_distanceFunction.Variables;
    
    [nx, ny] = size(distanceFunction);
    
    figure(3)
    set(gcf, 'Name', 'Computed distance function')
    clf
    mesh(distanceFunction,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(jet)
    view(0,90)
    axis equal
    axis([1 ny 1 nx])
    hold on

    minimum = min(min(distanceFunction));
    maximum =  max(max(distanceFunction));
    contour3(distanceFunction, linspace(minimum, maximum, 15), 'LineWidth', 3,...
            'EdgeColor', 'k');
    shading flat
    
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
end

%% Visibility-based
if settings.visibilityBasedSolver && settings.saveResults

    filename_visibilityBased = "output/visibilityBased.txt";
    T_visibilityBased = readtable(filename_visibilityBased,'Delimiter',' ');
    visibilityBased = T_visibilityBased.Variables;
    visibilityBased(isinf(visibilityBased)) = 0;
    
    minimum = min(min(visibilityBased));
    maximum = max(max(visibilityBased));
    
    visibilityBased(visibilityBased == 0) = maximum;

    [nx, ny] = size(visibilityBased);

    if settings.saveCameFrom
        filename_visibilityBased_cameFrom = "output/visibilityBased_cameFrom.txt";
        T_visibilityBased_cameFrom = readtable(filename_visibilityBased_cameFrom,'Delimiter',' ');
        visibilityBased_cameFrom = T_visibilityBased_cameFrom.Variables;
        plotCameFrom = false;
    end

    if settings.saveLightSources
        filename_visibilityBased_lightSources = "output/visibilityBased_lightSources.txt";
        T_visibilityBased_lightSources = readtable(filename_visibilityBased_lightSources, 'Delimiter',' ');
        visibilityBased_lightSources = T_visibilityBased_lightSources.Variables;
        visibilityBased_lightSources = visibilityBased_lightSources + 1;
        plotLightSources = false;
    end

    figure(4);
    set(gcf, 'Name', 'Visibility-based solver')
    clf
    m = mesh(visibilityBased,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(jet)
    caxis([minimum maximum])
    view(0,90)
    axis equal
    axis([1 ny 1 nx])
    hold on
    
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));

    contour3(visibilityBased, linspace(minimum, maximum, 40), 'LineWidth', 3,...
            'EdgeColor', 'k');
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));


    if settings.saveLightSources && plotLightSources
        for i = 1:size(visibilityBased_lightSources, 1)
            pt_ = visibilityBased_lightSources(i,:);
            plot3(pt_(2), pt_(1), visibilityBased(pt_(1),pt_(2))+1e4+10,'o',...
                'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                'MarkerSize', 10, 'LineWidth', 2)
        end
    end
    
    % The code below works only when initial frontline has 1 point entry as
    % it does not search for the initial frontline point that is closest
    sp_o = settings.initialFrontline + 1;
    % ep_o can actually be any generic point as we are using cameFrom
    % to find the path to any queried location
    ep_o = [settings.target_x, settings.target_y] + 1;
    
    shading flat
    
    if settings.saveLightSources && settings.saveCameFrom && settings.savePath
        pt = ep_o;
        points = [pt];
        d_visibilityBased = 0;
        while true 
            if pt(1) == sp_o(1) && pt(2) == sp_o(2)
                break
            end
            pt = visibilityBased_lightSources(visibilityBased_cameFrom(pt(1), pt(2)) + 1, :);
            points(end+1, :) = pt;
        
            d_visibilityBased = d_visibilityBased + ...
                norm(points(end,:) - points(end-1,:));
        
            line([points(end,2),points(end-1,2)],...
                [points(end,1),points(end-1,1)], ...
                [visibilityBased(points(end,1), points(end,2))+10 ...
                 visibilityBased(points(end-1,1),points(end-1,2))+10],...
                'LineWidth', 3, "color", "magenta")

            plot3(ep_o(2), ep_o(1), visibilityBased(ep_o(1), ep_o(2))+15,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)

            sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
            plot3(sp_o(2), sp_o(1), visibilityBased(sp_o(1), sp_o(2))+1e2,'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 24, 'LineWidth', 3)
        end
    end

    if settings.saveCameFrom && plotCameFrom
%         figure(5);
%         set(gcf, 'Name', 'Visibility-based cameFrom')
%         clf
%         m = mesh(visibilityBased_cameFrom, 'FaceLighting','phong','FaceColor','interp',...
%             'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
%         colormap(lines)
%         view(0,90)
%         axis equal
%         axis([1 ny 1 nx])
%         hold on
        
        figure(5)
        % Define the grid size and number of colors
        gridSize = size(visibilityField, 1);
        numColors = size(visibilityBased_lightSources, 1);

        % Define the range of values for each color
        % colorRanges = linspace(1, gridSize^2, numColors+1);
        colorRanges = linspace(1, numColors+1, numColors+1);
        
        % Create a random grid with integer values between 1 and gridSize^2
        % grid = randi(gridSize^2, gridSize, gridSize);
        % grid = visibilityBased_cameFrom;
        
        % Create a random obstacle matrix
        obstacleMatrix = imcomplement(visibilityField);
        % obstacleMatrix = randi([0,1], gridSize, gridSize);

        % Create the colormap
        % colors = hsv(randperm(numColors);
        colorIndices = randperm(numColors);
        colors = jet(numColors);
        colors = colors(colorIndices, :);

        % Display the grid with obstacles
        imagesc(visibilityBased_cameFrom);
        % set(gca, 'YDir', 'reverse');
        % mesh(visibilityBased_cameFrom);
        colormap(colors);

        % Create a black mask for the obstacle positions
        mask = repmat(obstacleMatrix, [1, 1, 3]);
        mask(:, :, 1) = mask(:, :, 1) * 0;
        mask(:, :, 2) = mask(:, :, 2) * 0;
        mask(:, :, 3) = mask(:, :, 3) * 0;

        % Overlay the black mask on top of the grid
        hold on;
        h = image(mask);
        set(h, 'AlphaData', obstacleMatrix);

        % Add color ranges to the colorbar
        c = colorbar;
        ticks = linspace(1, numColors+1, numColors+1) - 0.5;
        tickLabels = cellstr(num2str((1:numColors)', 'Light Source %d'));
        c.Ticks = ticks;
        c.TickLabels = tickLabels;
        c.TickLength = 0;
        c.TickLabelInterpreter = 'latex';
        c.Label.HorizontalAlignment = "center";
        
        % Change the y-axis direction and axis limits to match the grid
        % axis ij;
        % axis equal;
        % xlim([0.5 gridSize+0.5]);
        % ylim([0.5 gridSize+0.5]);

        % hold off;

        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca, 'ztick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        set(gca, 'YDir','normal')
        
        view(0,90)
        axis equal
        axis([1 ny 1 nx])

        sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
        plot3(sp_o(2), sp_o(1), 0+1e2,'o',...
        'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
        'MarkerSize', 24, 'LineWidth', 3)

        if settings.saveLightSources
           for i = 1:size(visibilityBased_lightSources, 1)
            pt_ = visibilityBased_lightSources(i,:);
            plot3(pt_(2), pt_(1), 1,'o',...
                'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                'MarkerSize', 10, 'LineWidth', 2)
           end
        end
    end
end




%% Comparison - display path distance results
if settings.saveResults 
    if settings.visibilityBasedSolver && settings.saveLightSources && settings.saveCameFrom && settings.savePath
        sprintf('Visibility-based path distance: %0.6f', d_visibilityBased)
    end
    if settings.vstar && settings.savePath
        sprintf('VStar path distance: %0.6f', d_vstar)
    end
    if settings.astar && settings.savePath
        sprintf('AStar path distance: %0.6f', d_astar)
    end
end

% sprintf('visibilityBased distance: %0.6f', visibilityBased(ep_o(1),ep_o(2)))

% [d_visibilityBased d_vstar d_astar]
% err = 100 * (max(d_visibilityBased, d_vstar) - min(d_visibilityBased, d_vstar)) / max(d_visibilityBased, d_vstar)
% if err > 0
%     points = flip(points);
%     try
%         [vstar_path(:, 1) vstar_path(:, 2) points(:, 1) points(:, 2)]
%     catch
%     end
% end

%% Speed field (basically occupancy grid)
if settings.saveSpeedField && settings.saveResults
    filename_speedField = "output/speedField.txt";
    T_speedField = readtable(filename_speedField,'Delimiter',' ');
    speedField = T_speedField.Variables;
    plotSpeedField = false;
    
    if plotSpeedField
        [nx, ny] = size(speedField);

        figure(7)
        set(gcf, 'Name', 'Speed field')
        clf
        mesh(speedField,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(gray)
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on

        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
    end
end

%% Fast marching
if settings.msfm
    addpath(genpath("fast_marching"))
    % Load a maze image
    I1 = visibilityField;
    [nx, ny] = size(I1);
    % Convert the image to a speed map
    SpeedImage = I1;
    % I1; % imcomplement(I1)*2;
    % Set the source to end of the maze
    SourcePoint = [sp_o(1); sp_o(2)];
    % SourcePoint = [582; 618];
    % Calculate the distance map (distance to source)
    tic
    DistanceMap = msfm2d(SpeedImage, SourcePoint, true, true); 
    toc
    
    max_lim = sqrt(nx^2 + ny^2);
    DistanceMap(DistanceMap>max_lim) = 0;
    
    % Show the distance map
    % figure, imshow(DistanceMap,[0 3400])
    figure(16)
    clf
    set(gcf, 'Name', 'MSFM solution')
    mesh(DistanceMap,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(jet)
    % caxis([0 800])
    view(0,90)
    axis equal
    axis([1 ny 1 nx])
    hold on
    
    DistanceMap(isinf(DistanceMap)) = 0;
    minimum = min(min(DistanceMap));
    maximum = max_lim; % 800; % max(max(DistanceMap));
    contour3(DistanceMap+1, linspace(minimum, maximum, 60), 'LineWidth', 2,...
            'EdgeColor', 'k');
    
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));

    sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
    plot3(sp_o(2), sp_o(1), 0+1e2,'o',...
    'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
    'MarkerSize', 24, 'LineWidth', 3)
end

% Trace shortestline from StartPoint to SourcePoint
% StartPoint=[9;14];
% StartPoint = [449;203];
% tic
% ShortestLine=shortestpath(DistanceMap,StartPoint,SourcePoint);
% toc
% % Plot the shortest route
% % hold on, plot(ShortestLine(:,2),ShortestLine(:,1),'r','LineWidth',3)
% hold on
% elevation = zeros(size(ShortestLine,1),1);
% for i = 1:size(ShortestLine,1)
%     elevation(i) = DistanceMap(round(ShortestLine(i,1)),...
%         round(ShortestLine(i,2))) + 10;
% end
% hold on, plot3(ShortestLine(:,2),ShortestLine(:,1),elevation,...
%     'LineWidth',3,'color','black')
% 
% 
% d_3 = 0;
% for i = 1:size(ShortestLine,1)-1
%     d_3 = d_3 + norm(ShortestLine(i+1,:)-ShortestLine(i,:));
% end
% d_3

% clf
% mesh(SpeedImage,'FaceLighting','phong','FaceColor','interp',...
% 'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
% colormap(jet)
% view(0,90)

%% Error with MSFM
% if settings.visibilityBasedSolver && settings.msfm && settings.saveResults
%     wave_error = DistanceMap - visibilityBased;
%     figure(18)
%     clf
%     set(gcf, 'Name', 'Proposed vs MSFM Error')
%     mesh(wave_error,'FaceLighting','phong','FaceColor','interp',...
%         'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
%     colormap(jet)
%     % caxis([0 800])
%     view(0,90)
%     % axis equal
%     axis([1 ny 1 nx])
%     hold on
%     
%     % minimum = min(min(wave_error));
%     minimum = -0.3;
%     maximum = max(max(wave_error)); % 800; % max(max(DistanceMap));
%     contour3(wave_error, linspace(minimum, maximum, 60), 'LineWidth', 2,...
%             'EdgeColor', 'k');
%     caxis([minimum maximum])
%     
%     grid off
%     set(gca, 'xtick', [-1e6 1e6]);
%     set(gca, 'ytick', [-1e6 1e6]);
%     set(gca,'LooseInset',get(gca,'TightInset'));
% end

%% Compare against BWDIST
bwdist_comp = false;
if bwdist_comp
    obstacle = zeros(nx, ny);
    obstacle(501, 501) = 1;
    bwdist_wave = bwdist(obstacle);
    bwdist_err = bwdist_wave - visibilityBased;
    figure(19)
    clf
    set(gcf, 'Name', 'Error against BWDIST')
    mesh(bwdist_err,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(jet)
    % caxis([0 800])
    view(0,90)
    % axis equal
    axis([1 ny 1 nx])
    hold on
    
%     minimum_bwdist = min(min(bwdist_err));
%     maximum_bwdist = max(max(bwdist_err)); % 800; % max(max(DistanceMap));
%     contour3(bwdist_err, linspace(minimum_bwdist, maximum_bwdist, 60), 'LineWidth', 2,...
%             'EdgeColor', 'k');
%     caxis([minimum_bwdist maximum_bwdist])
    
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
end

%%
% plot3(sp_o(2), sp_o(1), visibilityBased(ep_o(1), ep_o(2))+1e4+15,'o',...
%     'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
%     'MarkerSize', 20, 'LineWidth', 3)

% sp_o = [501 501];
% plot3(sp_o(2), sp_o(1), visibilityBased(sp_o(1), sp_o(2))+1e4+15,'o',...
%     'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
%     'MarkerSize', 24, 'LineWidth', 3)

% sp_o = [75 75];
% plot3(sp_o(2), sp_o(1), visibilityBased(sp_o(1), sp_o(2))+1e4+15,'o',...
%     'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
%     'MarkerSize', 24, 'LineWidth', 3)
% 
% sp_o = [75 485];
% plot3(sp_o(2), sp_o(1), visibilityBased(sp_o(1), sp_o(2))+1e4+15,'o',...
%     'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
%     'MarkerSize', 24, 'LineWidth', 3)

% {500, 500, 74, 74, 74, 484}
%% Expfig
% addpath(genpath('expfig'))
% export_fig werwqwsd -r250 -transparent -png
% % export_fig visibilityBased_x -r400 -transparent -png
% % export_fig ble -transparent -png -native -preserve_size