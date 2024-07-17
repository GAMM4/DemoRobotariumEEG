%Code based on "go_to_point" By Sean Wilson-07/2019
%Modified By:Jhonatan Alvarez-Laura Gama-06/2024

%Initialize global variables
global timeCounter; 
global EEGClasificator;
global eegRecords;

%Initialize robotarium variables
N = 5;
final_goal_points = [0 -1.2 -0.6 0.6 1.2;0 0 0 0 0;0 0 0 0 0];
initial_positions = final_goal_points;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

si_barrier_certificate = create_si_barrier_certificate();
si_to_uni_dynamics = create_si_to_uni_dynamics(); 

args = {'PositionError', 0.12, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

%Define markers variables for plotting characteristics
marker_size_robot = 50;
markerOptions=["o" "+" "*" "x" "square" "diamond" "pentagram" "hexagram"];
markerPointer=1;
marker=markerOptions(markerPointer);

%Color variables for the marker 
redColor=1;
greenColor=1;
blueColor=1;
CM=zeros(5,3);

%Variable for create the virtual point 
virtualPointDistance=0.01;

%Inicialize inout for EEG signal 
eegRecordsload=load("reading2.mat");
eegRecords = eegRecordsload.outputHistory;
EEGClasificator=0;
timeCounter=1;

% Define timer
t = timer;

% Configure the timer for execute the process each second
t.Period = 1; % Intervalo en segundos
t.ExecutionMode = 'fixedRate'; % Modo de ejecución fijo

% Function for being execute
t.TimerFcn = @miFuncion;

% Start timer
start(t);

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

%Predifene the trajectory for the Robots
trajectoryPointsNumber=250;
frecuencyActualization=1/20;
[x1, y1]=lissajousCurve(trajectoryPointsNumber,frecuencyActualization,0.4,0.4,0,0,3,1,pi/2);
[x2, y2]=butterflyCurve(trajectoryPointsNumber,frecuencyActualization,-0.1,0.1,-0.8,0.5);
[x3, y3]=butterflyCurve(trajectoryPointsNumber,frecuencyActualization,0.1,0.1,-0.8,-0.5);
[x4, y4]=butterflyCurve(trajectoryPointsNumber,frecuencyActualization,0.1,0.1,0.8,0.5);
[x5, y5]=butterflyCurve(trajectoryPointsNumber,frecuencyActualization,-0.1,0.1,0.8,-0.5);

startTime = tic;

for i = 1:trajectoryPointsNumber
    final_goal_points(1,1)=x1(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,1)=y1(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,2)=x2(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,2)=y2(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,3)=x3(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,3)=y3(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,4)=x4(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,4)=y4(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,5)=x5(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,5)=y5(i)+virtualPointDistance*sin(x(3));

    while(~init_checker(x, final_goal_points))

        x = r.get_poses();
        dxi = controller(x(1:2, :), final_goal_points(1:2, :));

        dxi = si_barrier_certificate(dxi, x(1:2, :));      
        dxu = si_to_uni_dynamics(dxi, x);
        
        r.set_velocities(1:N, dxu);
        r.step(); 

    end
    
    if EEGClasificator==0
        markerPointer=markerPointer+1;
        if (markerPointer==length(markerOptions)-1)
            markerPointer=1;
        end
        marker=markerOptions(markerPointer);
    end
    if EEGClasificator==-1
        virtualPointDistance=virtualPointDistance+0.001;
        if virtualPointDistance>=0.03
           virtualPointDistance=0.03;
        end
        redColor=redColor-0.1;
        greenColor=greenColor-0.1;
        blueColor=blueColor-0.1;
        if redColor<=0.4
            redColor=0.4;
        end
        if greenColor<=0.4
            greenColor=0.4;
        end
        if blueColor<=0.4
            blueColor=0.4;
        end
    end
    if EEGClasificator==1
        virtualPointDistance=virtualPointDistance-0.001;
        if virtualPointDistance<=0.01
           virtualPointDistance=0.01;
        end
        redColor=redColor+0.05;
        greenColor=greenColor+0.05;
        blueColor=blueColor+0.05;
        if redColor>=1
            redColor=1;
        end
        if greenColor>=1
            greenColor=1;
        end
        if blueColor>=1
            blueColor=1;
        end
    end
    
    for k = 1:N
        if(x(2,k)<-0.2)

            CM(k, :) = [redColor 0 0];

        elseif(-0.2 <= x(2,k) && x(2,k)<0.2)

            CM(k, :) = [0 0 blueColor];

        elseif(x(2,k)>=0.2)

            CM(k, :) = [redColor greenColor 0];
        end
    end
    
    if(i>1)
       for j = 1:N

           g(j) = plot(x(1,j),x(2,j),marker,'MarkerSize', marker_size_robot,'LineWidth',5,'Color',CM(j,:));

       end
    end
    
end

%% Second Act- Dance exhibition 

si_barrier_certificate = create_si_barrier_certificate();
si_to_uni_dynamics = create_si_to_uni_dynamics(); 

final_goal_points = [0 -1.2 -0.6 0.6 1.2;0 0 0 0 0;0 0 0 0 0];

args = {'PositionError', 0.05, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

x = [-1.6, 1.6, 1.6, -1.6];
y = [1, 1, -1, -1]; 

fill(x, y, [1 1 1]);

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

%Predifene the trajectory for the Robots
trajectoryPointsNumber=300;
move=1;
stepMarker=1;
neutralStateFlag=0;

changeStep=60;
velocityConstant=1;

virtualPointDistance=0.01;
for j = 1:N
    paths{j} = plot([500, 500], [500, 500],'LineWidth',5,'LineStyle','--','Color',[1 0 0]);
    path_data{j} = zeros(3, trajectoryPointsNumber);
end

offset1=[-0.8,0.3];
offset2=[-0.4,0];
offset3=[0,-0.3];
offset4=[0.4,0];
offset5=[0.8,0.3];

[x1,y1]=coreography(offset1(1),offset1(2),move,trajectoryPointsNumber);
[x2,y2]=coreography(offset2(1),offset2(2),move,trajectoryPointsNumber);
[x3,y3]=coreography(offset3(1),offset3(2),move,trajectoryPointsNumber);
[x4,y4]=coreography(offset4(1),offset4(2),move,trajectoryPointsNumber);
[x5,y5]=coreography(offset5(1),offset5(2),move,trajectoryPointsNumber);
i=0;
numberofpoints=0;
while and((i < length(x1)),(i < trajectoryPointsNumber)) 
    i=i+1;
    numberofpoints=numberofpoints+1;
    final_goal_points(1,1)=x1(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,1)=y1(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,2)=x2(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,2)=y2(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,3)=x3(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,3)=y3(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,4)=x4(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,4)=y4(i)+virtualPointDistance*sin(x(3));
    final_goal_points(1,5)=x5(i)+virtualPointDistance*cos(x(3));
    final_goal_points(2,5)=y5(i)+virtualPointDistance*sin(x(3));

    while(~init_checker(x, final_goal_points))

        x = r.get_poses();

        dxi = controller(x(1:2, :), final_goal_points(1:2, :));
        dxi = si_barrier_certificate(dxi, x(1:2, :));      
        dxu = si_to_uni_dynamics(dxi, x);
        
        r.set_velocities(1:N, dxu);
        r.step(); 
    end
         
    for k=1:N
            path_data{k}(:, i) = x(:, k);
            paths{k}.Color=[redColor 0 blueColor];
            paths{k}.XData = path_data{k}(1, max(i-30, 1):i);
            paths{k}.YData = path_data{k}(2, max(i-30, 1):i); 
    end
    if and(EEGClasificator==0,numberofpoints>changeStep)
        move=round(4*rand)+1;
        numberofpoints=1;
        switch move
            case 1
                offset1=[-0.8,0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,-0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,0.3];%5
            case 2
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
         
            case 3
                offset1=[-0.5,-0.3];%
                offset2=[-0.5,0.3];%%
                offset3=[0,0];%3
                offset4=[0.5,-0.3];%
                offset5=[0.5,0.3];%
            
            case 4
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
            case 5
                
                offset1=[-0.8,-0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,-0.3];%5
                
        end
        
        [x1(i:i+changeStep),y1(i:i+changeStep)]=coreography(offset1(1),offset1(2),move,changeStep);
        [x2(i:i+changeStep),y2(i:i+changeStep)]=coreography(offset2(1),offset2(2),move,changeStep);
        [x3(i:i+changeStep),y3(i:i+changeStep)]=coreography(offset3(1),offset3(2),move,changeStep);
        [x4(i:i+changeStep),y4(i:i+changeStep)]=coreography(offset4(1),offset4(2),move,changeStep);
        [x5(i:i+changeStep),y5(i:i+changeStep)]=coreography(offset5(1),offset5(2),move,changeStep);
    end
    if and(EEGClasificator==-1,numberofpoints>changeStep)
        move=move-1;
        if move<1
           move=5; 
        end
        numberofpoints=1;
        switch move
            case 1
                offset1=[-0.8,0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,-0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,0.3];%5
            case 2
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
         
            case 3
                offset1=[-0.5,-0.3];%
                offset2=[-0.5,0.3];%%
                offset3=[0,0];%3
                offset4=[0.5,-0.3];%
                offset5=[0.5,0.3];%
            
            case 4
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
            case 5
                
                offset1=[-0.8,-0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,-0.3];%5       
        end
        
        [x1(i:i+changeStep),y1(i:i+changeStep)]=coreography(offset1(1),offset1(2),move,changeStep);
        [x2(i:i+changeStep),y2(i:i+changeStep)]=coreography(offset2(1),offset2(2),move,changeStep);
        [x3(i:i+changeStep),y3(i:i+changeStep)]=coreography(offset3(1),offset3(2),move,changeStep);
        [x4(i:i+changeStep),y4(i:i+changeStep)]=coreography(offset4(1),offset4(2),move,changeStep);
        [x5(i:i+changeStep),y5(i:i+changeStep)]=coreography(offset5(1),offset5(2),move,changeStep);
    end
    if and(EEGClasificator==1,numberofpoints>changeStep)
        move=move+1;
        if move>5
           move=1; 
        end
        numberofpoints=1;
        switch move
            case 1
                offset1=[-0.8,0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,-0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,0.3];%5
            case 2
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
         
            case 3
                offset1=[-0.5,-0.3];%
                offset2=[-0.5,0.3];%%
                offset3=[0,0];%3
                offset4=[0.5,-0.3];%
                offset5=[0.5,0.3];%
            
            case 4
                offset1=[-0.5,0.3];%1
                offset2=[-0.5,-0.3];%2
                offset3=[0,0];%3
                offset4=[0.5,0.3];%4
                offset5=[0.5,-0.3];%5
            case 5
                
                offset1=[-0.8,-0.3];%1
                offset2=[-0.4,0];%2
                offset3=[0,0.3];%3
                offset4=[0.4,0];%4
                offset5=[0.8,-0.3];%5
                
                
        end 
        [x1(i:i+changeStep),y1(i:i+changeStep)]=coreography(offset1(1),offset1(2),move,changeStep);
        [x2(i:i+changeStep),y2(i:i+changeStep)]=coreography(offset2(1),offset2(2),move,changeStep);
        [x3(i:i+changeStep),y3(i:i+changeStep)]=coreography(offset3(1),offset3(2),move,changeStep);
        [x4(i:i+changeStep),y4(i:i+changeStep)]=coreography(offset4(1),offset4(2),move,changeStep);
        [x5(i:i+changeStep),y5(i:i+changeStep)]=coreography(offset5(1),offset5(2),move,changeStep);
        
    end
    if(EEGClasificator==-1)
       virtualPointDistance=virtualPointDistance+0.005;
        if virtualPointDistance>=0.06
           virtualPointDistance=0.06;
        end
        redColor=redColor-0.1;
        if redColor<=0.1
            redColor=0.1;
        end
        blueColor=blueColor-0.05;
        if blueColor<=0.4
            blueColor=0.4;
        end 
    end
    if(EEGClasificator==1)
        virtualPointDistance=virtualPointDistance-0.005;
        if virtualPointDistance<=0.04
           virtualPointDistance=0.04;
        end
        
        redColor=redColor+0.1;
        if redColor>=0.8
            redColor=0.8;
        end
        blueColor=blueColor+0.05;
        if blueColor>=0.8
            blueColor=0.8;
        end 
    end
end

% stop(song);
stop(t);
delete(t);
r.debug();

function miFuncion(~, ~)
    global eegRecords;
    global EEGClasificator;
    global timeCounter;
    global t;

    if timeCounter > length(eegRecords)
        stop(t);
        delete(t);
        return;
    end

    EEGClasificator = eegRecords(timeCounter)
    timeCounter = timeCounter + 1
end
