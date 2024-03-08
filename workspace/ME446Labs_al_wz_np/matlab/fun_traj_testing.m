zmax = 0.5334;
zmin = 0.127;
ymax = 0.0762;
ymin = -0.0762;
x = 0.3937;

t = 0:0.001:7;
y = (ymax - ymin).*sin(2*pi/3.5.*t)./2 + (ymin+ymax)/2;
z = (zmax - zmin).*cos(t.*pi/3.5)./2 + (zmin+zmax)/2;

theta_desired1 = atan2(y,x)
theta_desired2 = atan2((0.254-z),sqrt(pow(x,2) + pow(y,2)))-acos( (pow(y,2)+pow(x,2)+pow((0.254-z),2)) / (2*(sqrt(pow(x,2)+pow(y,2)+pow((0.254-z),2))*(0.254))))
theta_desired3 = pi - acos((-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) )
(-(pow((0.254-z),2)+pow(x,2)+pow(y,2)) + pow((0.254),2) + pow((0.254),2)) / (2*.254*.254) 
plot(y,z)

