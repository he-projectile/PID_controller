function alpha = angle(x, y)
    thirdVec=cross(x,y);
    if (thirdVec(3) ~= 0)
        alpha=acos( dot(x,y)/( norm(x)*norm(y) ) )* sign(thirdVec(3));
    else 
        alpha=acos( dot(x,y)/( norm(x)*norm(y) ) );
    end
end

