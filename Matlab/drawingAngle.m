clc;
clear;

figure(1)
clf(figure(1))
hold on; grid on;
xlabel('время'); ylabel('yгол, °');
ylim([-5 185])
% ylim([-0.125 0.125])

IMU = serialport("COM13",115200,"Timeout",15);

coordinate=zeros(3,1);

start = tic;
prevTimeSample = toc(start);
times=zeros(1,60/0.05);
angles=zeros(1,60/0.05);
cnt = 1;
while 1
    angle=getIMUdata(IMU);
    
    timeSample = toc(start);
    
    times(cnt) = timeSample;
    angles(cnt) = angle;
    if cnt == length(times)+1
        break
    end
    
   xlim([timeSample-5 timeSample])    
   plot(timeSample, angle, '.b')
    
end
plot(times, angles, 'r');


function angles=getIMUdata(port)
    flush(port, "input")
    angles=read(port,1,'uint16');
    angles = angles/100;
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