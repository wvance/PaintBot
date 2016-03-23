%*******************************************************************************
% Name			: Nicholas Warner
% Date			: 28 February 2016
% Subject		: CSCE 452
% Assignment	: Project 1, 2
% Updated		: 22 March 2016
% Description	: A 3 Link Planar Robotic Arm that Paints - Paintbot
%*******************************************************************************

% Modified on Sublime

%*******************************************************************************
%                               Version 1
% All initial functionality added AND modified (not simple default GUIDE code):
% rotateRz      paintbot_OpeningFcn
% rotateRx      oneright_Callback
% rotateRy      oneleft_Callback
% translateX    twoCounterclock_Callback
% translateZ    twoClockwise_Callback
% move01        threeCounter_Callback
% move02        threeClockwise_Callback
% move03        paint_Callback
%
%*******************************************************************************
%                               Version 2
% Functions added AND modified:
% paintBrushSizeSlider_CreateFcn    YPlus_Callback
% paintBrushSizeSlider_Callback     XMinus_Callback
% XPlus_Callback                    YMinus_Callback
%
% Paintbrush size slider
%   Global variables pw and ph added to every drawing function (clock and
%   counter buttons) for paintbrush
%   height and width default value of 0.1 added to ph and pw in slider button
%   init/create function
%
% WorldControl Buttons added
%    X+, Y+, X-, Y-
%    These buttons will change the coordinate (X, Y) of the end effector
%
% TO DO
% Add functions to change variables for arm postions (delta, theta 2 and 3,
%    a1 through a3)
% Implement in each of the four world control buttons
% Uncomment line code in world control buttons to redraw and paint
%
% RECOMMEND
% Adding a max-reach-fence check. Should the desired reach be outside of fence,
%   which is based on current joint positions, either slide to the general area,
%   or forbid the movement
%
%*******************************************************************************


function varargout = paintbot(varargin)
% PAINTBOT MATLAB code for paintbot.fig
%      PAINTBOT, by itself, creates a new PAINTBOT or raises the existing
%      singleton*.
%
%      H = PAINTBOT returns the handle to a new PAINTBOT or the handle to
%      the existing singleton*.
%
%      PAINTBOT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PAINTBOT.M with the given input arguments.
%
%      PAINTBOT('Property','Value',...) creates a new PAINTBOT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before paintbot_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to paintbot_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help paintbot

% Last Modified by GUIDE v2.5 22-Mar-2016 09:11:23

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @paintbot_OpeningFcn, ...
                   'gui_OutputFcn',  @paintbot_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

function [R] = rotateRz(theta,m)
theta = (theta*pi)/180;
Rz = [cos(theta),-sin(theta),0,0;...
        sin(theta),cos(theta),0,0;...
        0,0,1,0;...
        0,0,0,1];
R = Rz * m;

function [R] = rotateRx(theta,m)
theta = (theta*pi)/180;
Rx = [1,0,0,0;...
        0,cos(theta),-sin(theta),0;...
        0,sin(theta),cos(theta),0;...
        0,0,0,1];
R = Rx * m;

function [R] = rotateRy(theta,m)
theta = (theta*pi)/180;
Ry = [cos(theta),0,sin(theta),0;...
        0,1,0,0;...
        -sin(theta),0,cos(theta),0;...
        0,0,0,1];
R = Ry * m;

function [R] = translateX(a,m)
Tx = [1,0,0,a;...
        0,1,0,0;...
        0,0,1,0;...
        0,0,0,1];
R = Tx * m;

function [R] = translateZ(d,m)
Tx = [1,0,0,0;...
        0,1,0,0;...
        0,0,1,d;...
        0,0,0,1];
R = Tz * m;

% function [R] = move01(theta,a,m)
% theta = (theta*pi)/180;
% A = [cos(theta),-sin(theta),0,a*cos(theta);...
%         sin(theta),cos(theta),0,a*sin(theta);...
%         0,0,1,0;...
%         0,0,0,1];
% R = A * m;
% disp(R);

% function [R] = move02(theta1,theta2,a1,a2,m)
% delta = (delta*pi)/180;
% theta2 = (theta2*pi)/180;
% A1 = [cos(theta1),-sin(theta1),0,a1*cos(theta1);...
%         sin(theta1),cos(theta1),0,a1*sin(theta1);...
%         0,0,1,0;...
%         0,0,0,1];
% A2 = [cos(theta2),-sin(theta2),0,a2*cos(theta2);...
%         sin(theta2),cos(theta2),0,a2*sin(theta2);...
%         0,0,1,0;...
%         0,0,0,1];
% R = (A1 * A2) * m;

% function [R] = move03(delta,theta2,theta3,a1,a2,a3,m)
% delta = (delta*pi)/180;
% theta2 = (theta2*pi)/180;
% theta3 = (theta3*pi)/180;
% A1 = [cos(theta1),-sin(theta1),0,a1*cos(theta1);...
%         sin(theta1),cos(theta1),0,a1*sin(theta1);...
%         0,0,1,0;...
%         0,0,0,1];
% A2 = [cos(theta2),-sin(theta2),0,a2*cos(theta2);...
%         sin(theta2),cos(theta2),0,a2*sin(theta2);...
%         0,0,1,0;...
%         0,0,0,1];
% A3 = [cos(theta3),-sin(theta3),0,a3*cos(theta3);...
%         sin(theta3),cos(theta3),0,a3*sin(theta3);...
%         0,0,1,0;...
%         0,0,0,1];
% R = ((A1 * A2) * A3) * m;

function [R] = move01(delta,a,m)
global a1;
a1(3) = delta;
A = [1, 0, 0, a;...
    0, 0, 1, 0;...
    0, -1, 0, delta;...
    0, 0, 0, 1];
m(3) = delta;
R = A * m;

function [R] = move02(delta,theta2,a1,a2,m)
theta2 = (theta2*pi)/180;
A1 = [1, 0, 0, a1;...
    0, 0, 1, 0;...
    0, -1, 0, delta;...
    0, 0, 0, 1];
A2 = [cos(theta2),-sin(theta2),0,a2*cos(theta2);...
        sin(theta2),cos(theta2),0,a2*sin(theta2);...
        0,0,1,0;...
        0,0,0,1];
R = (A1 * A2) * m;

function [R] = move03(delta,theta2,theta3,a1,a2,a3,m)
theta2 = (theta2*pi)/180;
theta3 = (theta3*pi)/180;
A1 = [1, 0, 0, a1;...
    0, 0, 1, 0;...
    0, -1, 0, delta;...
    0, 0, 0, 1];
A2 = [cos(theta2),-sin(theta2),0,a2*cos(theta2);...
        sin(theta2),cos(theta2),0,a2*sin(theta2);...
        0,0,1,0;...
        0,0,0,1];
A3 = [cos(theta3),-sin(theta3),0,a3*cos(theta3);...
        sin(theta3),cos(theta3),0,a3*sin(theta3);...
        0,0,1,0;...
        0,0,0,1];
R = ((A1 * A2) * A3) * m;

% --- Executes just before paintbot is made visible.
function paintbot_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to paintbot (see VARARGIN)
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

delta = 0;
% view = [az,el]
a4(3) = .5142;
a4(1) = 5.9142;

a3Calc = inverseKin(a4(3), a4(1));

% a4(3)
% a4(1)
% theta2
% theta3
% a3Calc(1)
% a3Calc(2)

a1 = [0;0;0+delta;1];
temp = move01(delta,3,a1);

a2(1) = temp(1);
a2(2) = temp(2);
a2(3) = temp(3);
a2(4) = 1;
temp = move02(delta,theta2,3,2,a1);

a3(1) = temp(1);
a3(2) = temp(2);
a3(3) = temp(3);
a3(4) = 1;
temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);
a4(4) = 1;

line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3(3)],[a2(1) a3(1)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

% Choose default command line output for paintbot
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes paintbot wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = paintbot_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in oneright.
function oneright_Callback(hObject, eventdata, handles)
% hObject    handle to oneright (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

delta = delta + (1/36);

temp = move01(delta,3,a1);

a2(1) = temp(1);
a2(2) = temp(2);
a2(3) = temp(3);
a2(4) = 1;
temp = move02(delta,theta2,3,2,a1);

a3(1) = temp(1);
a3(2) = temp(2);
a3(3) = temp(3);
a3(4) = 1;
temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line1);
delete(line2);
delete(line3);
line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3(3)],[a2(1) a3(1)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end

% --- Executes on button press in oneleft.
function oneleft_Callback(hObject, eventdata, handles)
% hObject    handle to oneleft (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

delta = delta - (1/36);

temp = move01(delta,3,a1);

a2(1) = temp(1);
a2(2) = temp(2);
a2(3) = temp(3);
a2(4) = 1;
temp = move02(delta,theta2,3,2,a1);

a3(1) = temp(1);
a3(2) = temp(2);
a3(3) = temp(3);
a3(4) = 1;
temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line1);
delete(line2);
delete(line3);
line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3(3)],[a2(1) a3(1)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end

% --- Executes on button press in twoCounterclock.
function twoCounterclock_Callback(hObject, eventdata, handles)
% hObject    handle to twoCounterclock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

if theta2 == 360
    theta2 = 0;
else
    theta2 = theta2 + 1;
end

temp = move02(delta,theta2,3,2,a1);

a3(1) = temp(1);
a3(2) = temp(2);
a3(3) = temp(3);
a3(4) = 1;
temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line2);
delete(line3);
line2 = line([a2(3) a3(3)],[a2(1) a3(1)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end

% --- Executes on button press in twoCounterclock.
function twoClockwise_Callback(hObject, eventdata, handles)
% hObject    handle to twoCounterclock (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

if theta2 == 0
    theta2 = 360;
else
    theta2 = theta2 - 1;
end

temp = move02(delta,theta2,3,2,a1);

a3(1) = temp(1);
a3(2) = temp(2);
a3(3) = temp(3);
a3(4) = 1;
temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line2);
delete(line3);
line2 = line([a2(3) a3(3)],[a2(1) a3(1)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end

% --- Executes on button press in threeCounter.
function threeCounter_Callback(hObject, eventdata, handles)
% hObject    handle to threeCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

if theta3 == 360
    theta3 = 0;
else
    theta3 = theta3 + 1;
end

temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line3);
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end

% --- Executes on button press in threeCounter.
function threeClockwise_Callback(hObject, eventdata, handles)
% hObject    handle to threeCounter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global toggle;
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height

if theta3 == 0
    theta3 = 360;
else
    theta3 = theta3 - 1;
end

temp = move03(delta,theta2,theta3,3,2,1.5,a1);

a4(1) = temp(1);
a4(2) = temp(2);
a4(3) = temp(3);

delete(line3);
line3 = line([a3(3) a4(3)],[a3(1) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue

if toggle == 1
    h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
end


% --- Executes on button press in paint.
function paint_Callback(hObject, eventdata, handles)
% hObject    handle to paint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of paint
global toggle;
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
    toggle = true;
elseif button_state == get(hObject,'Min')
    toggle = false;
end


% --- Executes on button press in init.
function init_Callback(hObject, eventdata, handles)
% hObject    handle to init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over paint.
function paint_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to paint (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function paintBrushSizeSlider_Callback(hObject, eventdata, handles)
% hObject    handle to paintBrushSizeSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global pw;  %paintbrush width
global ph;  %paintbrush height
pw = get(hObject,'Value') + .1;
ph = get(hObject,'Value') + .1;


% --- Executes during object creation, after setting all properties. Only runs once.
function paintBrushSizeSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to paintBrushSizeSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

global pw;  %paintbrush width
global ph;  %paintbrush height
pw = 0.1;   %default
ph = 0.1;   %default

% --- Computes the inverse kinematics given an end point
function [R] = inverseKin(x0,y0)
global theta2;
global theta3;
global a2;

y0 = y0 - 3;

D = (x0^2 + y0^2 - 1.5^2 - 2^2)/(2*2*1.5);
% D = (6.25 - x0^2 - y0^2)/16;
test = sqrt(1-D^2)
if test > 0
    theta3 = atan2d((sqrt(1-D^2)),D);

    % phi = atan2d((sqrt(1-D^2)),D);
    % phi
    % theta3 = 180-phi;

    abeta = atan2d(y0,x0);
    alpha = atan2d(1.5*sind(theta3),2+1.5*cosd(theta3));

    theta2 = abeta - alpha;

    %theta2
    %theta3
    tempx = 2*cosd(theta2) + a2(3);
    tempy = 2*sind(theta2) + 3;

    R(1) = tempx
    R(2) = tempy
end
% theta3 = mod(theta3,180);

% --- Executes on button press in XPlus.
function XPlus_Callback(hObject, eventdata, handles)
% hObject    handle to XPlus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height
global toggle;
% the end effector (paintbrush) of the robot is a4(3) and a4(1), X and Y

a4(3) = a4(3) + .1;    % arbitrary constant, adds to X

a3Calc = inverseKin((a4(3)+.1), a4(1));

a3

a3(1) = a3Calc(2);
a3(3) = a3Calc(1);

a3

delete(line1);
delete(line2);
delete(line3);

line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3Calc(1)],[a2(1) a3Calc(2)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3Calc(1) a4(3)],[a3Calc(2) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue
%
% Same paint functionality
  if toggle == 1
      h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
  end


% --- Executes on button press in YPlus.
function YPlus_Callback(hObject, eventdata, handles)
% hObject    handle to YPlus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height
global toggle;
% the end effector (paintbrush) of the robot is a4(3) and a4(1), X and Y

a4(1) = a4(1) + 0.1;    % arbitrary constant, adds to Y
a3Calc = inverseKin(a4(3),(a4(1)+.1));

a3

a3(1) = a3Calc(2);
a3(3) = a3Calc(1);

a3
% PSEUDO
% call function(s) to figure out the delta, theta 2 and 3, and a1 through a3 variables
% redraw lines to new variables
%
delete(line1);
delete(line2);
delete(line3);
line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3Calc(1)],[a2(1) a3Calc(2)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3Calc(1) a4(3)],[a3Calc(2) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue
%
% Same paint functionality
 if toggle == 1
     h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
 end

% --- Executes on button press in XMinus.
function XMinus_Callback(hObject, eventdata, handles)
% hObject    handle to XMinus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height
global toggle;
% the end effector (paintbrush) of the robot is a4(3) and a4(1), X and Y

a4(3) = a4(3) - 0.1;    % arbitrary constant, subtracts from X
a3Calc = inverseKin((a4(3)-.1),a4(1));

a3

a3(1) = a3Calc(2);
a3(3) = a3Calc(1);

a3
%
delete(line1);
delete(line2);
delete(line3);
line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3Calc(1)],[a2(1) a3Calc(2)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3Calc(1) a4(3)],[a3Calc(2) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue
%
% Same paint functionality
 if toggle == 1
     h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
 end


% --- Executes on button press in YMinus.
function YMinus_Callback(hObject, eventdata, handles)
% hObject    handle to YMinus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global delta;
global theta2;
global theta3;
global line1;
global line2;
global line3;
global a1;
global a2;
global a3;
global a4;
global pw;  %paintbrush width
global ph;  %paintbrush height
global toggle;
% the end effector (paintbrush) of the robot is a4(3) and a4(1), X and Y

a4(1) = a4(1) - 0.1;    % arbitrary constant, subtracts from Y
a3Calc = inverseKin(a4(3),(a4(1)-.1));

a3

a3(1) = a3Calc(2);
a3(3) = a3Calc(1);

a3
% PSEUDO
% call function(s) to figure out the delta, theta 2 and 3, and a1 through a3 variables
% redraw lines to new variables
%
delete(line1);
delete(line2);
delete(line3);
line1 = line([a1(3) a2(3)],[a1(1) a2(1)],'LineWidth',15,'Color',[1 0 0]);   %red
line2 = line([a2(3) a3Calc(1)],[a2(1) a3Calc(2)],'LineWidth',15,'Color',[0 1 0]);   %green
line3 = line([a3Calc(1) a4(3)],[a3Calc(2) a4(1)],'LineWidth',15,'Color',[0 0 1]);   %blue
%
% Same paint functionality
 if toggle == 1
     h = rectangle('Position',[a4(3) a4(1) pw ph],'Curvature',[1 1],'FaceColor',[0 0 0]);
 end

%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNN                                               NNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNN   Nicholas Warner * warnern@email.tamu.edu    NNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNN                                               NNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNN         NNNNNNNNNN         NNNNNNNNNN         NNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN         NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN         NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNN         NNNNNNNNNN         NNN       DNNNNNNN       NNNNNNNNNND
%NNNNNNNNNNNNNNNNDO      NNNNNNNNNNN         NNNM$       NNNN=       NDNNNNNNNNND
%NNNNNNNNNNNNNNNNN        NNNNNNNNNN         NNNNO       ZNNN        NNNNNNNNNNND
%NNNNNNNNNNNNNNNN          NNNNNNNNN         NNNNO        DD         NNNNNNNNNNND
%NNNNNNNNNNNNNNN7    NN    DNNNNNNNN         NNNNO                   NNNNNNNNNNND
%NNNNNNNNNNNNNNN    7NN:    NNNNNNNN         NNNNO    N        N     NNNNNNNNNNND
%NNNNNNNNNNNNND              NNNNNNN         NNNNO    NN      NN     NNNNNNNNNNND
%NNNNNNNNNNNNN:              $NNNNNN         NNNNO    NNN     NN     NNNNNNNNNNND
%NNNNNNNNNNNND    NNNNNNN7    NNNNNN         NNNNO    NNN    NNN     NNNNNNNNNNND
%NNNNNNNNNDDN     NNNNNNNN     DDNNN         NNND$    NDNN  NNND     DNNNNNNNNNND
%NNNNNNNNNN        NNNNNN        NNN         NNN       ZNNNNNNN       NNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN         NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNDNNNN         NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN                   NNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNN                   NNNNNNNNNNNNNNNNNNNNNNNNNNNNNND
%NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNND

