function [] = plotCircle(cg,R,c)
%PLOTCIRCLE plot a circle in the currently active figure window
%   The circle is centered at cg (a vector with the x and y coordinates)
%   The radius is R
theta = linspace(0,2*pi,100);
if nargin<3
    plot(cg(1)+R*cos(theta),cg(2)+R*sin(theta),'k');
else
    fill(cg(1)+R*cos(theta),cg(2)+R*sin(theta),c);
end 
end

