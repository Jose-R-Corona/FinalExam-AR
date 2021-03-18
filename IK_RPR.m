function q=IK_RPR(p,phi)
x = p(1);
y=p(2);

q1=sqrt(x^2+y^2);
q2=sqrt((x+cos(phi)-1)^2 + (y+sin(phi))^2);
q3=sqrt((x+cos(phi+pi/3)-1/2)^2 + (y+sin(phi+pi/3)-sqrt(3)/2)^2);

q=[q1;q2;q3];
end