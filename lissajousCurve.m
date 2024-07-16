%% Funtion to generate diferent lissajous curves.
function [xcurve,ycurve]=LissajousCurve(numberOfPoints,frecuency,xlength,ylength,xoffset,yoffset,constantfrecuency1,constantfrecuency2,angularPhase)
    t=0:frecuency:numberOfPoints*frecuency;
    xcurve=xlength*sin(constantfrecuency1*t)+xoffset;
    ycurve=ylength*sin(constantfrecuency2*t+angularPhase)+yoffset;
end