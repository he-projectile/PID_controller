clear 

errFunction = @(K) PIDlocal(K(1), K(2), K(3), K(4));

k0=[0.1, 0.1, 0.1, 0];
lb=[0 0 0 0];
ub=[2 2 2 0];
options = optimoptions('fmincon', 'Display', 'iter');
best_params = fmincon(errFunction, k0, [], [], [], [], lb, ub, [], options)

[sse, angles, times] = errFunction(best_params);
sse

% Графики
figure(2); % Создаю график с номером 1 / Обращаюсь к графику 1
clf(figure(2)) % Чищу график 1

xlabel('Время, с');
ylabel('Наклон луча, град)');
title('Динамика угла');
ylim([0 120])
hold on; grid on;
plot(times, rad2deg(angles), 'r', 'LineWidth', 1);
yline(90,'r')



function dYdt = pendulumODE(t, Y, l, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks)
    if Y(1) >= 0.99*pi
        Y(2) = 0;
        Y(1) = 0.99*pi;
    end  
    O = Y(1);         % Угол O
    omega = Y(2);     % Угловая скорость dO/dt
    e = (alpha-O);
    
    d_integral=e;
    
    if Y(3)>10
        Y(3) = 10;
    end
    if Y(3)<-10
        Y(3) = -10;
    end
    integral = Y(3); 
    
    U = Kp*e+Ki*integral-Kd*omega+Ks*sin(alpha);

    if U>1
        U = 1;
    end
    if U < 0.1/0.6
        U = 0.1/0.6;
    end
    F = F_T*U;
  
    domegadt = (F*l_T-m*g*l*sin(O-psi)-kv*omega*t-ka*omega^2*sign(omega)*t)/J;

    dYdt = [omega; domegadt; d_integral];  % Возвращаем вектор производных
end

function [sse, angles, times] = PIDlocal(Kp, Ki, Kd, Ks)
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
alpha = deg2rad(90); % Цель

% Начальные условия
O0 = deg2rad(9.5);    % Начальный угол
omega0 = 0;   % Начальная угловая скорость
integral0 = 0;
Y0 = [O0; omega0; integral0];

% Время моделирования
dt=0.005;
tspan = [0: dt: 10]; 

[t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l_m, m, g, J, psi, kv, ka, F_T, l_T, alpha, Kp, Ki, Kd, Ks), tspan, Y0);

sse = sum((abs(alpha-Y(:,1))).*t.^0)*dt;

times = t;
angles = Y(:,1);
end