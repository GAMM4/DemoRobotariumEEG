function [xcurve,ycurve]=butterflyCurve(numberOfPoints,frecuency,xlength,ylength,xoffset,yoffset)
    t=0:frecuency:numberOfPoints*frecuency;
    xcurve=xlength*(sin(t) .* (exp(cos(t)) - 2 .* cos(4*t) - sin(t/12).^5))+xoffset;
    ycurve=ylength*(cos(t) .* (exp(cos(t)) - 2 .* cos(4*t) - sin(t/12).^5))+yoffset;
end