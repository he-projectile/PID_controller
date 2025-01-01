clc;
clear;
figure(6)
clf(figure(6))
hold on; grid on;
% writematrix([], 'IMU_DATA.txt', 'WriteMode', 'overwrite');
axis equal;
view([150,40]); %azimuth, elevation
title('Инерциальный трекер');
xlabel('x');ylabel('y');zlabel('z');

basis=[1 0 0; 0 1 0; 0 0 1];
drawBasis([0 0 0],basis, 0.1);
plot3(0,0,0,'ob');
xlim([-0.125 0.125])
ylim([-0.125 0.125])
zlim([-0.125 0.125])
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])

IMU = serialport("COM6",115200,"Timeout",15);

coordinate=zeros(3,1);

input('опусти в -90 градусов и нажми что-нибудь')
flush(IMU, "input")
basisStep1=getIMUdata(IMU);
input('подними в 0 градусов и нажми что-нибудь')
flush(IMU, "input")
basisStep2=getIMUdata(IMU);

transitionMatrix = basisStep2*inv(basisStep1);

[eigVectors, eigValues] = eig(transitionMatrix);
eigValues = diag(eigValues);
tolerance = 1e-2;
real_index = find(abs(eigValues - 1) < tolerance);
TrMatrixRotAxis = eigVectors(:, real_index);

rotAxis = TrMatrixRotAxis;
rotAxis=rotAxis/norm(rotAxis);

axis2verticalAngle = rad2deg(angle(rotAxis, [0 0 -1]));
if abs( 90 - axis2verticalAngle ) < 5
    disp("стоим ровно")
else
    disp("стоим криво")
end
    
rotAxisCheck = basisStep1'*rotAxis;

drawVector([0 0 0], rotAxis, 0.1, 'k');

prewBas=[];
prewVec=[];

flush(IMU, "input")
while 1
   basis=getIMUdata(IMU);
    
   delete(prewBas); delete(prewVec);
   prewBas = drawBasis(coordinate, basis, 0.05);
%    currentAxis = basis*eigenVector;
%    prewVec = drawVector([0 0 0], currentAxis, 0.05, 'y');
%     angleX=rad2deg(abs(angle(rotAxis, basis(:,1))));
%     angleY=rad2deg(abs(angle(rotAxis, basis(:,2))));
%     angleZ=rad2deg(abs(angle(rotAxis, basis(:,3))));
%   disp([angleX angleY angleZ])
    rotAxisProjection = basis'*rotAxis;
    if rad2deg(abs(angle(rotAxisCheck, rotAxisProjection))) > 5
        disp("падаем")       
    end
%     disp( rad2deg(abs(angle(rotAxisCheck, rotAxisProjection))) )

end

function bas=getIMUdata(port)
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
function res = drawBasis(coord, bas, scale)
       res = quiver3(coord(1),coord(2),coord(3), bas(1,1),bas(2,1),bas(3,1), scale,'r');
       res = [res,quiver3(coord(1),coord(2),coord(3), bas(1,2),bas(2,2),bas(3,2), scale,'g')];
       res = [res,quiver3(coord(1),coord(2),coord(3), bas(1,3),bas(2,3),bas(3,3), scale,'b')];
end
function res =  drawVector(cord,vec,scale,style)
       res = quiver3(cord(1),cord(2),cord(3),vec(1),vec(2),vec(3), scale,style);
end
function drawPoint(coord)
        plot3(coord(1),coord(2),coord(3),'.c');
end