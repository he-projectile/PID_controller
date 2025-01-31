function dYdt = pendulumODE(t, Y, l, m, g, F0, kt, alpha, kp, ki, kd)
    O = Y(1);         % Угол O
    omega = Y(2);     % Угловая скорость dO/dt
    integral = Y(3);
    
    e = (alpha-O);
    
    d_integral=e;
    
    K = kp*e+ki*integral-kd*omega;
    
    if K>1
        K = 1;
    end
    if K < 0
        K = 0;
    end
    F = F0*K;
    dOdt = omega;
    domegadt = ((F-kt*t*omega) / (l * m)) - (g / l) * sin(O);

    dYdt = [dOdt; domegadt; d_integral];  % Возвращаем вектор производных
end