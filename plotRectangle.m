function [] = plotRectangle(leftTop,rightBottom,c)
%PLOTRECTANGLE plot a rectangle in the currently active figure window
%   The rectangle is defined by two opposite corners, each characterized by
%   their x and y coordinates
%   If a third argument is given, the rectangle is filled with the
%   appropriate color 
if nargin<3
    plot([leftTop(1), rightBottom(1), rightBottom(1), leftTop(1)],...
         [leftTop(2), leftTop(2), rightBottom(2), rightBottom(2)],'k');
else
    fill([leftTop(1), rightBottom(1), rightBottom(1), leftTop(1)],...
         [leftTop(2), leftTop(2), rightBottom(2), rightBottom(2)],c);
end    
end

