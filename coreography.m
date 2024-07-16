function[x,y]=coreography(ofx, ofy,stepIdentifier,numberofPoints)
frecuencyActualization=1/20;
    if stepIdentifier==1 %% forward and backward
        [x, y]=lissajousCurve(numberofPoints,frecuencyActualization,0.2,0.2,ofx,ofy,1,1,0);
        x(1:length(x))=ofx;
    end

    if stepIdentifier==2 %% o
        [x, y]=lissajousCurve(numberofPoints,frecuencyActualization,0.2,0.2,ofx,ofy,1,1,pi/2);
    end

    if stepIdentifier==3 %% u
        [x, y]=lissajousCurve(numberofPoints,frecuencyActualization,0.2,0.2,ofx,ofy,1,2,pi/2);
    end

    if stepIdentifier==4%% -o
        [x, y]=lissajousCurve(numberofPoints,frecuencyActualization,-0.2,0.2,ofx,ofy,1,1,pi/2);
    end


    if stepIdentifier==5%% lazo
        [x, y]=lissajousCurve(numberofPoints,frecuencyActualization,0.2,0.2,ofx,ofy,2,3,pi/2);
    end
end