clc;
clear;

fig = findobj('Type', 'figure', 'Name', 'PID-контроллер');
if ~isempty(fig)
    clf(fig);
else
    fig = figure('Name', 'PID-контроллер', 'NumberTitle', 'off');
end
tabGroup = uitabgroup(fig);
tabBasis = uitab( tabGroup, 'Title', "базис" );
axisBasis = axes('Parent', tabBasis);
hold on; grid on;
axis equal;
view([150,40]); %azimuth, elevation
title('Инерциальный трекер');
xlabel('x');ylabel('y');zlabel('z');
xlim([-0.125 0.125])
ylim([-0.125 0.125])
zlim([-0.125 0.125])


tabAngle = uitab( tabGroup, 'Title', "угол" );
axisAngle = axes('Parent', tabAngle);
hold on; grid on;
xlabel('время'); ylabel('yгол, °');
ylim([-5 185])
% ylim([-0.125 0.125])


tabSTL = uitab( tabGroup, 'Title', "модель" );
axisSTL = axes('Parent', tabSTL);
axis equal;
hold on;
xlim([-20 25])
zlim([-30 20])
ylim([-15 15])
lighting gouraud;   % Включаем освещение для более плавного отображения
material dull;     % Задаем shiny материал для блеска
camlight infinite;        % Добавляем источник света для улучшения видимости
view([10 30 5])
camproj('perspective');
camva(8); 

gridSize = 100;       % Длина и ширина сетки
gridStep = 5;        % Расстояние между линиями

% Создание координат для сетки
[x, y] = meshgrid(-gridSize:gridStep:gridSize, -gridSize:gridStep:gridSize);
z = zeros(size(x))-28.6;  % Z-координаты сетки (на уровне Z = 0)
surf(x, y, z, 'EdgeColor', 'k', 'FaceColor', 'none', 'Parent', axisSTL);  % Полупрозрачная сетка

modelStand = stlread('C:\Users\barko\Desktop\Daniil\MIET\PID_controller\Matlab\stlModels\stand.stl');
newOrient = (RotX(90)*modelStand.Points')';
modelStand = triangulation(modelStand.ConnectivityList, newOrient);
trisurf(modelStand, 'FaceColor', 'white', 'EdgeColor', 'none', 'Parent', axisSTL);

modelBeam = stlread('C:\Users\barko\Desktop\Daniil\MIET\PID_controller\Matlab\stlModels\beam.stl');
newOrient = (RotX(90)*modelBeam.Points')';
modelBeam = triangulation(modelBeam.ConnectivityList, newOrient);

for i = 0:180*2
   newOrient = (RotY(-i/2)*modelBeam.Points')'; 
   beamArray{i+1} = triangulation(modelBeam.ConnectivityList, newOrient); 
end


basis=[1 0 0; 0 1 0; 0 0 1];
drawBasis(axisBasis, [0 0 0],basis, 0.1);
plot3(axisBasis, 0,0,0,'ob');

IMU = serialport("COM6",115200,"Timeout",15);

coordinate=zeros(3,1);

input('опусти в -90 градусов и нажми Энтер')
basisStep1=getIMUdata(IMU);
input('подними в 0 градусов и нажми Энтер')
basisStep2=getIMUdata(IMU);

transitionMatrix = basisStep2*inv(basisStep1);

[eigVectors, eigValues] = eig(transitionMatrix);
eigValues = diag(eigValues);

[~, vectorIndex] = min(abs(eigValues - 1));
TrMatrixRotAxis = eigVectors(:, vectorIndex);

if vectorIndex == 1
    angleIndex = 2;
else
    angleIndex = 1;
end

fprintf("повернул на %5.1f°\n", rad2deg(angle(eigValues(angleIndex))) )

rotAxis = TrMatrixRotAxis;
rotAxis=rotAxis/norm(rotAxis);

axis2verticalAngle = rad2deg(vectorAngle(rotAxis, [0 0 -1]));
if abs( 90 - axis2verticalAngle ) < 5
    disp("стоим ровно")
else
    disp("стоим криво")
end
    
rotAxisCheck = basisStep1'*rotAxis;

drawVector(axisBasis, [0 0 0], rotAxis, 0.1, 'k');

prewBas=[];
surfBeam = [];

modelDrawInterval = 0.1;
modelDrawPrevTime = tic;

while 1
   basis=getIMUdata(IMU);
    
   delete(prewBas);
   prewBas = drawBasis(axisBasis, coordinate, basis, 0.05);

    rotAxisProjection = basis'*rotAxis;
    if rad2deg(abs(vectorAngle(rotAxisCheck, rotAxisProjection))) > 5
        disp("падаем")       
    end
    
    transitionMatrix = basis*inv(basisStep1);
    [~, eigValues] = eig(transitionMatrix);
    eigValues = diag(eigValues);
    beamAngle = rad2deg(angle(eigValues(angleIndex)));
    
    plot(axisAngle, datetime('now'), beamAngle, '.b')
    
    if (0 <= beamAngle) && (beamAngle <= 180 && (toc(modelDrawPrevTime)>modelDrawInterval))
        arrayIndex = round(beamAngle*2)+1;
        delete(surfBeam)
        surfBeam = trisurf(beamArray{arrayIndex}, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'none', 'Parent', axisSTL); 
        modelDrawPrevTime = tic;
    end

end

function bas=getIMUdata(port)
    flush(port, "input")
    s=read(port,10,'int8');
    
    bas(1,1) = s(1)/100;
    bas(1,2) = s(2)/100;
    bas(1,3) = s(3)/100;
    bas(2,1) = s(4)/100;
    bas(2,2) = s(5)/100;
    bas(2,3) = s(6)/100;
    bas(3,1) = s(7)/100;
    bas(3,2) = s(8)/100;
    bas(3,3) = s(9)/100;
end
%отрисовка базиса
function res = drawBasis(axes, coord, bas, scale)
       res = quiver3(axes, coord(1),coord(2),coord(3), bas(1,1),bas(2,1),bas(3,1), scale,'r');
       res = [res,quiver3(axes, coord(1),coord(2),coord(3), bas(1,2),bas(2,2),bas(3,2), scale,'g')];
       res = [res,quiver3(axes, coord(1),coord(2),coord(3), bas(1,3),bas(2,3),bas(3,3), scale,'b')];
end
function res =  drawVector(axes, cord,vec,scale,style)
       res = quiver3(axes, cord(1),cord(2),cord(3),vec(1),vec(2),vec(3), scale,style);
end
function drawPoint(axes, coord)
        plot3(axes, coord(1),coord(2),coord(3),'.c');
end