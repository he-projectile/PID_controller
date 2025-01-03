% Отображаем модель в 3D
figure(1);
clf(figure(1))
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
surf(x, y, z, 'EdgeColor', 'k', 'FaceColor', 'none');  % Полупрозрачная сетка

modelStand = stlread('C:\Users\barko\Desktop\Daniil\MIET\PID_controller\Matlab\stlModels\stand.stl');
newOrient = (RotX(90)*modelStand.Points')';
modelStand = triangulation(modelStand.ConnectivityList, newOrient);
trisurf(modelStand, 'FaceColor', 'white', 'EdgeColor', 'none');

modelBeam = stlread('C:\Users\barko\Desktop\Daniil\MIET\PID_controller\Matlab\stlModels\beam.stl');
newOrient = (RotX(90)*modelBeam.Points')';
modelBeam = triangulation(modelBeam.ConnectivityList, newOrient);

for i = 0:180*2
   newOrient = (RotY(-i/2)*modelBeam.Points')'; 
   beamArray{i+1} = triangulation(modelBeam.ConnectivityList, newOrient); 
end

tic
surfBeam = [];
for i = 0:360
    delete(surfBeam)
    surfBeam = trisurf(beamArray{i+1}, 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'none');
    
    drawnow
    pause(0.1)
end
toc