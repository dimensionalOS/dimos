clc;
clear all;
close all;

%% generate path
%{.
dis = 1.0;
angle = 27;
deltaAngle = angle / 3;
scale = 0.65;

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');

for shift1 = -angle : deltaAngle : angle
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];
    
    pathStartR = 0 : 0.01 : dis;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
    
    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));
    
    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];
    
    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;
                          3 * dis, shift3, 0];

                pathR = 0 : 0.01 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];
                
                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);
        end
    end
    
    groupID = groupID + 1
end

pathID

fileID = fopen('startPaths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathStartAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d\n', pathStartAll);
fclose(fileID);

fileID = fopen('paths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);

fileID = fopen('pathList.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathList, 2));
fprintf(fileID, 'property float end_x\n');
fprintf(fileID, 'property float end_y\n');
fprintf(fileID, 'property float end_z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathList);
fclose(fileID);

pause(1.0);
%}

%% find correspondence
%{.
voxelSize = 0.02;
searchRadius = 0.45;
offsetX = 3.2;
offsetY = 4.5;
voxelNumX = 161;
voxelNumY = 451;

fprintf('\nPreparing voxels\n');

indPoint = 1;
voxelPointNum = voxelNumX * voxelNumY;
voxelPoints = zeros(voxelPointNum, 2);
for indX = 0 : voxelNumX - 1
    x = offsetX - voxelSize * indX;
    scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX;
    for indY = 0 : voxelNumY - 1
        y = scaleY * (offsetY - voxelSize * indY);

        voxelPoints(indPoint, 1) = x;
        voxelPoints(indPoint, 2) = y;
        
        indPoint  = indPoint + 1;
    end
end

plot3(voxelPoints(:, 1), voxelPoints(:, 2), zeros(voxelPointNum, 1), 'k.');
pause(1.0);

fprintf('\nCollision checking\n');

[ind, dis] = rangesearch(pathAll(1 : 2, :)', voxelPoints, searchRadius);

fprintf('\nSaving correspondences\n');

fileID = fopen('correspondences.txt', 'w');

for i = 1 : voxelPointNum
    fprintf(fileID, '%d ', i - 1);
    
    indVoxel = sort(ind{i});
    indVoxelNum = size(indVoxel, 2);
    
    pathIndRec = -1;
    for j = 1 : indVoxelNum
        pathInd = pathAll(4, indVoxel(j));
        if pathInd == pathIndRec
            continue;
        end

        fprintf(fileID, '%d ', pathInd);
        pathIndRec = pathInd;
    end
    fprintf(fileID, '-1\n');
    
    if mod(i, 1000) == 0
        i
    end
end

fclose(fileID);

fprintf('\nProcessing complete\n');
%}
