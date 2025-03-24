function dYdt = pendulumODE(t, Y, l, m0, g, F0, kt, alpha, kp, ki, kd, ks)
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
    
    K = kp*e+ki*integral-kd*omega+ks*sin(alpha);
    
%     if t < 5
%         m = m0;
%     else
%         m = 0.2*m0;
%     end
    
    m = m0;

    if K>1
        K = 1;
    end
    if K < 0.1
        K = 0.1;
    end
    F = F0*K;
    dOdt = omega;
    domegadt = ((F-kt*t*omega) / (l * m)) - (g / l) * sin(O);

    dYdt = [dOdt; domegadt; d_integral];  % Возвращаем вектор производных
end