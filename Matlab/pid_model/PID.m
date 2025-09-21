% Параметры
l_m = norm([138.760 -4.145 ])/1000*1; % Расстояние до центра масс, м
m = 0.181;    % Масса, кг
psi = atan2(-4.145, 138.760); % Угол между балкой и вектором до центра масс, радианы
J = 2036.662 /10^6+m*l_m^2; % Момент инерции, кг*м^2
g = 9.81; % м/с^2
kv = 0.0001850; % Сопротивление смазки, попугаи
ka = 0.0001058; % Сопротивление воздуха, попугаи
l_T = 0.254; % Плечо пропеллера, м
F_T=0.148*g; % Максимальная тяга пропеллера, Н

% Начальные условия
O0 = deg2rad(13.5);    % Начальный угол 9.5
omega0 = 0;   % Начальная угловая скорость
integral0 = 0;
Y0 = [O0; omega0; integral0];

% Время моделирования
tspan = [0: 0.001: 10];    

Kp = 0.6; %1.3887;
Ki = 1.5; %1;  
Kd = 0.2; %0.5;  
Ks = 00; %0.00
% Решение диффура
alpha = deg2rad(90); % Цель

global logData;
logData = [];
global delayedAngle;
delayedAngle = [O0 O0 0];
[t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l_m, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks), tspan, Y0, odeset('RelTol',1e-6,'AbsTol',1e-9));

% Графики
figure(4); % Создаю график с номером 1 / Обращаюсь к графику 1
clf(figure(4)) % Чищу график 1

% subAxesTop = subplot(2, 1, 1);
xlabel('Время, с');
ylabel('Наклон луча, град');
title('Динамика угла');
ylim([0 130])
hold on; grid on;

% plot(subAxesTop, logData(:,1), rad2deg(logData(:,2)), 'b', 'LineWidth', 1.5);
plot( t, rad2deg(Y(:,1)), 'b', 'LineWidth', 3);

yline(rad2deg(alpha),'r', 'y_{ЗЗ}', 'LabelHorizontalAlignment','left', 'LineWidth', 2, 'FontSize', 20)
yline(rad2deg(max(Y(:,1))),'k', 'y_{max}', 'LabelHorizontalAlignment','left', 'LineWidth', 2, 'FontSize', 20)

delta_r = 1-80/90;
T_r = t(end);
for i = flip(1 : length(t))
    if abs(Y(i,1)-alpha)/alpha > delta_r
       T_r = t(i);
       break
    end
end

yline(rad2deg(alpha)*(1-delta_r), 'r', 'y_{ЗЗ}-\Delta', 'LabelHorizontalAlignment','left', 'LineWidth', 2, 'FontSize', 20)
yline(rad2deg(alpha)*(1+delta_r), 'r', 'y_{ЗЗ}+\Delta', 'LabelHorizontalAlignment','left', 'LineWidth', 2, 'FontSize', 20)
xline(T_r, 'k', 'T_{ПП}', 'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'top', 'LineWidth', 2, 'FontSize', 20)

% subAxesBot = subplot(2, 1, 2);
% xlabel('Время, с');
% ylabel('Коэффициенты');
% title('Динамика угла');
% ylim([-2 2])
% hold on; grid on;
% 
% plot(subAxesBot, logData(:,1), logData(:,3), 'b');
% plot(subAxesBot, logData(:,1), logData(:,4), 'r');
% plot(subAxesBot, logData(:,1), logData(:,5), 'g');
% yline(0,'k')
% 
% linkaxes([subAxesTop, subAxesBot], 'x');


% 
% % Решение диффура
% alpha = deg2rad(100); % Цель
% [t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l_m, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks), tspan, Y0);
% plot(t, rad2deg(Y(:,1)), 'g', 'LineWidth', 1.5);
% yline(rad2deg(alpha),'g')
% 
% % Решение диффура
% alpha = deg2rad(45); % Цель
% [t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l_m, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks), tspan, Y0);
% plot(t, rad2deg(Y(:,1)), 'b', 'LineWidth', 1.5);
% yline(rad2deg(alpha),'b')

function dYdt = pendulumODE(t, Y, l, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks)
    if Y(1) >= 0.99*pi
        Y(2) = 0;
        Y(1) = 0.99*pi;
    end  
    O = Y(1);         % Угол O
    omega = Y(2);     % Угловая скорость dO/dt
    
%     global delayedAngle;
%     if delayedAngle(3)+0.0045 <= t
%          delayedAngle = [O delayedAngle(1) t];
%          delayedAngle(3) = t;
%     end
    
%    e = (alpha-delayedAngle(1));
    e = alpha - O;
    
    d_integral=e;
    
    if Y(3)>2
        Y(3) = 2;
    end
    if Y(3)<-2
        Y(3) = -2;  
    end
    integral = Y(3); 
    
%     D = Kd*(delayedAngle(1) - delayedAngle(2))/0.0045;
    D = Kd*omega;
    
    U = Kp*e+Ki*integral-D+Ks*sin(alpha);

    if U>1
        U = 1;
    end
    if U < 0.1
        U = 0.1;
    end
    F = F_T*U;
  
    domegadt = (F*l_T-m*g*l*sin(O-psi)-kv*omega*t-ka*omega^2*sign(omega)*t)/J;
    
%     global logData;
%     logData = [logData; t, O, Kp*e, Ki*integral, -Kd*omega];

    dYdt = [omega; domegadt; d_integral];  % Возвращаем вектор производных
end