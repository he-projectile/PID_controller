function TrMatrix=RotL(angle, axis)
    angle = deg2rad(angle);
    axis=axis/norm(axis);
    c=cos(angle);
    s=sin(angle);
    lx=axis(1); ly=axis(2); lz=axis(3);
    TrMatrix=[ c+(1-c)*lx^2 (1-c)*lx*ly-s*lz (1-c)*lz*lx+s*ly; (1-c)*lx*ly+s*lz c+(1-c)*ly^2 (1-c)*lz*ly-s*lx; (1-c)*lx*lz-s*ly (1-c)*ly*lz+s*lx c+(1-c)*lz^2];
end

