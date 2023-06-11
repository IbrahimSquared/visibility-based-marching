clc; clear;
% Clear all figures
FigList = findall(groot, 'Type', 'figure');
for iFig = 1:numel(FigList)
    try
        clf(FigList(iFig));
    catch
        % Nothing to do
    end
end

system('vbs.bat');

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

% Original start and end points (works for point to point path planning,
% not intended to be part of the VBM. VBM gives paths to all grid points,
% which can be read as shown in an example below).
sp_o = [settings.initialFrontline(1), settings.initialFrontline(2)] + 1;
ep_o = [settings.target_x, settings.target_y] + 1;

%%
if settings.saveResults
    %% Processed/used visibility field / occupancy grid
    if settings.saveVisibilityField
        filename_visibilityField = "output/visibilityField.txt";
        T_visibilityField = readtable(filename_visibilityField,'Delimiter',' ');
        visibilityField = T_visibilityField.Variables;
        
        [nx, ny] = size(visibilityField);

        figure(1)
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
    %% VBS VBM
    if settings.visibilityBasedSolver
        filename_visibilityBased = "output/visibilityBased.txt";
        T_visibilityBased = readtable(filename_visibilityBased,'Delimiter',' ');
        visibilityBased = T_visibilityBased.Variables;
        visibilityBased(isinf(visibilityBased)) = 0;
        
        if settings.saveCameFrom
            filename_visibilityBased_cameFrom = "output/visibilityBased_cameFrom.txt";
            T_visibilityBased_cameFrom = readtable(filename_visibilityBased_cameFrom,'Delimiter',' ');
            visibilityBased_cameFrom = T_visibilityBased_cameFrom.Variables + 1;
        end
        
        % VBM results
        figure(2);
        set(gcf, 'Name', 'Visibility-based solver')
        clf
        m = mesh(visibilityBased,'FaceLighting','phong','FaceColor','interp',...
            'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
        colormap(jet)
        
        minimum = min(min(visibilityBased));
        maximum = max(max(visibilityBased));
%         visibilityBased_ = visibilityBased;
%         visibilityBased(visibilityBased == 0) = maximum;
        
        for i = 1:2:length(settings.initialFrontline)
            piv = [settings.initialFrontline(i), settings.initialFrontline(i+1)] + 1;
            visibilityBased(piv(1), piv(2)) = 0;
        end

        [nx, ny] = size(visibilityBased);

        caxis([minimum maximum])
        view(0,90)
        axis equal
        axis([1 ny 1 nx])
        hold on
        contour3(visibilityBased+1, linspace(minimum, maximum, 40), 'LineWidth', 3,...
                'EdgeColor', 'k');

        grid off
        set(gca, 'xtick', [-1e6 1e6]);
        set(gca, 'ytick', [-1e6 1e6]);
        set(gca, 'ztick', [-1e6 1e6]);
        set(gca,'LooseInset',get(gca,'TightInset'));
       
        for i = 1:2:length(settings.initialFrontline)
            piv = [settings.initialFrontline(i), settings.initialFrontline(i+1)] + 1;
            plot3(piv(1), piv(2), visibilityBased(piv(1), piv(2))+50,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
        end
        
        
        % Plot light sources/pivots
        if settings.saveLightSources
            filename_visibilityBased_lightSources = "output/visibilityBased_lightSources.txt";
            T_visibilityBased_lightSources = readtable(filename_visibilityBased_lightSources, 'Delimiter',' ');
            visibilityBased_lightSources = T_visibilityBased_lightSources.Variables;
            visibilityBased_lightSources = visibilityBased_lightSources + 1;
            plotlightSources = false;
            if plotlightSources
                for i = 1:size(visibilityBased_lightSources, 1)
                    pt_ = visibilityBased_lightSources(i,:);
                    plot3(pt_(1), pt_(2), maximum+10,'o',...
                        'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                        'MarkerSize', 10, 'LineWidth', 2)
                end
            end
        end
        
        % Enable if you want to plot a path read from the C matrix
        % pt need not be ep_o, can be any other feasible point in
        % the grid
        if settings.saveCameFrom && settings.saveLightSources
            if true 
                pt = ep_o;
                plot3(ep_o(1), ep_o(2), visibilityBased(ep_o(2), ep_o(1))+50,'o',...
                      'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                      'MarkerSize', 24, 'LineWidth', 3)
                if (ep_o(1) <= nx && ep_o(2) <= ny) 
                    points = [pt];
                    while true
                        if pt(1) == sp_o(1) && pt(2) == sp_o(2)
                            break
                        end
                        pt = visibilityBased_lightSources(visibilityBased_cameFrom(pt(2), pt(1)), :);
                        points(end+1, :) = pt;
                        
                        line([points(end,1),points(end-1,1)],...
                            [points(end,2),points(end-1,2)], ...
                            [visibilityBased(points(end,2), points(end,1))+10 ...
                             visibilityBased(points(end-1,2), points(end-1,1))+10],...
                            'LineWidth', 4, "color", "magenta")
                    end
                end
            end
        end
    
        % CameFrom matrix C
        if (settings.saveCameFrom && settings.saveVisibilityField)
            figure(3)
            % Define the grid size and number of colors
            gridSize = size(visibilityField, 1);
            % gridSize = [nx, ny];
            numColors = size(visibilityBased_lightSources, 1);

            % Define the range of values for each color
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
            % mesh(visibilityBased_cameFrom,'FaceLighting','phong','FaceColor','interp',...
            % 'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
            view(0,90)
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

            grid off
            set(gca, 'xtick', [-1e6 1e6]);
            set(gca, 'ytick', [-1e6 1e6]);
            set(gca, 'ztick', [-1e6 1e6]);
            set(gca,'LooseInset',get(gca,'TightInset'));
            set(gca, 'YDir','normal')

            view(0,90)
            axis equal
            axis([1 ny 1 nx])
        end
    end
    
    %% Euclidean distance function settings (ESDF)
	if (settings.distanceFunction && settings.saveDistanceFunction)
        filename_distanceFunction = "output/distanceFunction.txt";
        T_distanceFunction = readtable(filename_distanceFunction,'Delimiter',' ');
        distanceFunction = T_distanceFunction.Variables;
        
        figure(4)
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

    %% Astar settings
    if settings.astar 
        if settings.savePath
            filename_astar_path = "output/astar_path.txt";
            T_astar_path = readtable(filename_astar_path, 'Delimiter',' ');
            astar_path = T_astar_path.Variables + 1;
        end
        
        if settings.saveFScore
            filename_astar_fScore = "output/astar_fScore.txt";
            T_astar_fScore = readtable(filename_astar_fScore,'Delimiter',' ');
            astar_fScore = T_astar_fScore.Variables;
            
            figure(5)
            set(gcf, 'Name', 'AStar FScore')
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

            plot3(ep_o(1), ep_o(2), astar_fScore(ep_o(2), ep_o(1))+1e2,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
            plot3(sp_o(1), sp_o(2), astar_fScore(sp_o(2), sp_o(1))+1e2,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
        end
        
        if settings.saveGScore
            filename_astar_gScore = "output/astar_gScore.txt";
            T_astar_gScore = readtable(filename_astar_gScore,'Delimiter',' ');
            astar_gScore = T_astar_gScore.Variables;
            
            d_astar = 0;
            elevation = zeros(size(astar_path, 1), 1);
            pt_old = astar_path(1,:);
            for i = 1:size(astar_path,1)
                pt = astar_path(i,:);
                d_astar = d_astar + norm(pt-pt_old);
                elevation(i) = astar_fScore(pt(end,2), pt(end,1))+10;
                pt_old = pt;
            end
            figure(6)
            set(gcf, 'Name', 'AStar gScore')
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

            plot3(astar_path(:,1), astar_path(:,2), elevation,...
                'Color', 'magenta', 'LineWidth', 3);
            plot3(ep_o(1), ep_o(2), astar_gScore(ep_o(2), ep_o(1))+1e3,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 1)
            plot3(sp_o(1), sp_o(2), astar_gScore(sp_o(2), sp_o(1))+1e3,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
        end
    end
    
    %% Vstar settings
    if settings.vstar
        if settings.saveFScore
            filename_vstar_fScore = "output/vstar_fScore.txt";
            T_vstar_fScore = readtable(filename_vstar_fScore,'Delimiter',' ');
            vstar_fScore = T_vstar_fScore.Variables;
            
            figure(7)
            set(gcf, 'Name', 'VStar FScore')
            clf
            mesh(vstar_fScore,'FaceLighting','phong','FaceColor','interp',...
                'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
            colormap(jet)
            view(0,90)
            axis equal
            axis([1 ny 1 nx])
            hold on

            minimum = min(min(vstar_fScore));
            vstar_fScore(isinf(vstar_fScore)) = 0;
            maximum =  max(max(vstar_fScore));
            contour3(vstar_fScore+1, linspace(minimum, maximum, 5), 'LineWidth', 3,...
                    'EdgeColor', 'k');

            grid off
            set(gca, 'xtick', [-1e6 1e6]);
            set(gca, 'ytick', [-1e6 1e6]);
            set(gca,'LooseInset',get(gca,'TightInset'));

            plot3(ep_o(1), ep_o(2), vstar_fScore(ep_o(2), ep_o(1))+1e2,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
            plot3(sp_o(1), sp_o(2), vstar_fScore(sp_o(2), sp_o(1))+1e2,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
        end
        
        if settings.saveGScore
            filename_vstar_gScore = "output/vstar_gScore.txt";
            T_vstar_gScore = readtable(filename_vstar_gScore,'Delimiter',' ');
            vstar_gScore = T_vstar_gScore.Variables;
            vstar_gScore(isinf(vstar_gScore)) = 0;
            
            figure(8)
            set(gcf, 'Name', 'VStar GScore')
            % minimum = min(min(vstar_gScore));
            % maximum =  max(max(vstar_gScore));
            % 
            % vstar_gScore(vstar_gScore==0) = inf;
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

            plot3(ep_o(1), ep_o(2), vstar_gScore(ep_o(2), ep_o(1))+1e2,'o',...
                'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)
            plot3(sp_o(1), sp_o(2), vstar_gScore(sp_o(2), sp_o(1))+1e2,'o',...
                'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
                'MarkerSize', 24, 'LineWidth', 3)

            if settings.savePath
                filename_vstar_path = "output/vstar_path.txt";
                T_vstar_path = readtable(filename_vstar_path, 'Delimiter',' ');
                vstar_path = T_vstar_path.Variables + 1;

                d_vstar = 0;
                elevation = zeros(size(vstar_path,1), 1);
                pt_old = vstar_path(1,:);
                for i = 1:size(vstar_path,1)
                    pt = vstar_path(i,:);
                    d_vstar = d_vstar + norm(pt-pt_old);
                    elevation(i) = vstar_gScore(pt(end,2), pt(end,1))+10;
                    pt_old = pt;
                end
                plot3(vstar_path(:,1), vstar_path(:,2), elevation,...
                 'Color', 'magenta', 'LineWidth', 3);
            end
        
        end
        
        %% Misc
        if settings.saveLightSources
            filename_vstar_lightSources = "output/vstar_lightSources.txt";
            T_vstar_lightSources = readtable(filename_vstar_lightSources, 'Delimiter',' ');
            vstar_lightSources = T_vstar_lightSources.Variables + 1;
        end
    end
end

%% MSFM
if settings.msfm && settings.saveResults && settings.saveVisibilityField
    addpath(genpath("fast_marching"))
    % Load a maze image
    I1 = visibilityField;
    [nx, ny] = size(I1);
    % Convert the image to a speed map
    SpeedImage = I1;
    % I1; % imcomplement(I1)*2;
    % Set the source to end of the maze
    SourcePoint = [sp_o(2); sp_o(1)];
    % SourcePoint = [582; 618];
    % Calculate the distance map (distance to source)
    tic
    DistanceMap = msfm2d(SpeedImage, SourcePoint, true, true); 
    toc
    
    max_lim = 1000;
    DistanceMap(DistanceMap>max_lim) = 0;
    
    % Show the distance map
    % figure, imshow(DistanceMap,[0 3400])
    figure(9)
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
    contour3(DistanceMap, linspace(minimum, maximum, 60), 'LineWidth', 2,...
            'EdgeColor', 'k');
    
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
end

%% Test
% Q = 1./distanceFunction; % .* visibilityBased;
% figure(10);
% set(gcf, 'Name', 'Visibility-based solver')
% clf
% m = mesh(Q,'FaceLighting','phong','FaceColor','interp',...
%     'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
% colormap(jet)
% 
% % minimum = min(min(Q));
% % maximum = max(max(Q));
% 
% % Q(Q == 0) = maximum;
% 
% [nx, ny] = size(Q);
% 
% % caxis([0 150])
% % zlim([0 150])
% view(0,90)
% % axis equal
% axis([1 ny 1 nx])
% % hold on
% % contour3(Q+1, linspace(minimum, maximum, 40), 'LineWidth', 3,...
% %         'EdgeColor', 'k');
% 
% grid off
% set(gca, 'xtick', [-1e6 1e6]);
% set(gca, 'ytick', [-1e6 1e6]);
% set(gca, 'ztick', [-1e6 1e6]);
% set(gca,'LooseInset',get(gca,'TightInset'));

%% Expfig  - sample usage
% addpath(genpath('expfig'))
% export_fig test_image -r250 -transparent -png