clear

load("колебания луча эксперимент.mat")
times = times(83:end)-times(83);
angles = angles(83:end);


errFunction = @(k) SSE(k(1), k(2), times, angles);
k0=[0, 0];
lb=[0, 0];
ub=[0.01 0.01];
options = optimoptions('fmincon', 'Display', 'iter');
best_params = fmincon(errFunction, k0, [], [], [], [], lb, ub, [], options)

function dYdt = pendulumODE(t, Y, l, m, g, J, psi, kv, ka)
    O = Y(1);         % Угол O
    dOdt = Y(2);     % Угловая скорость dO/dt
  
    domegadt = (-m*g*l*sin(O-psi)-kv*dOdt*t-ka*dOdt^2*sign(dOdt)*t)/J;
    dYdt = [dOdt; domegadt];  % Возвращаем вектор производных
end


function sse = SSE(kv, ka, times, angles)
% Параметры
l = norm([138.760 -4.145 ])/1000*1;    % Длина маятника (м)
m = 0.181;    % Масса (кг)
psi = atan2(-4.145, 138.760); % рад
J = 2036.662 /10^6+m*l^2; % кг м^2
g = 9.81; % Ускорение свободного падения (м/с²)

angles = angles+rad2deg(psi);


% Начальные условия
O0 = deg2rad(angles(1));    % Начальный угол (30°)
omega0 = 0;   % Начальная угловая скорость
Y0 = [O0; omega0];

% Временной интервал
tspan = [0 : 0.01 : times(end)];

% Решение уравнения
[t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l, m, g, J, psi, kv, ka), tspan, Y0);

[pks_model, locs_model] = findpeaks(rad2deg(Y(:,1)), t);
[pks_exp,   locs_exp]   = findpeaks(angles', times');
pks_model_interp = interp1(locs_model, pks_model, locs_exp, 'spline', 'extrap');

sse = sum((pks_model_interp-pks_exp).^2);
end