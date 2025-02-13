% Параметры
l = 1;    % Длина маятника (м)
m = 1;    % Масса (кг)
g = 9.81; % Ускорение свободного падения (м/с²)
F = 30;  % Постоянная касательная сила (Н)
k = 0.1; % Трение оси
alpha = deg2rad(90);

% Начальные условия
O0 = deg2rad(0);    % Начальный угол (30°)
omega0 = 0;   % Начальная угловая скорость
integral0 = 0;
Y0 = [O0; omega0; integral0];

% Временной интервал
tspan = [0 15];

% Решение уравнения
[t, Y] = ode45(@(t, Y) pendulumODE(t, Y, l, m, g, F, k, alpha, 1, 0.5, 0.2), tspan, Y0);

% Графики
figure(1);
clf(figure(1))

subplot(2,1,1);
plot(t, rad2deg(Y(:,1)), 'r', 'LineWidth', 1.5);
grid on
xlabel('Время, с');
ylabel('Угол \theta, град');
title('Динамика угла');
yline(rad2deg(alpha), 'b--', 'Цель')
xline(5, 'b--', 'Отпал груз')
ylim([0 1.1*max(rad2deg(Y(:,1)))])

subplot(2,1,2);
plot(t, Y(:,2), 'b', 'LineWidth', 1.5);
grid on
xlabel('Время, с');
ylabel('Угловая скорость ω, рад/с');
title('Динамика угловой скорости');