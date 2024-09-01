% Things to implement:
% Add print statement to each button
% Add labels on the GUI (especially for the sliders)
% Capture camera image
% Add timestamp to any captured images


function varargout = YouBot_GUI(varargin)
% YOUBOT_GUI MATLAB code for YouBot_GUI.fig
%      YOUBOT_GUI, by itself, creates a new YOUBOT_GUI or raises the existing
%      singleton*.
%
%      H = YOUBOT_GUI returns the handle to a new YOUBOT_GUI or the handle to
%      the existing singleton*.
%
%      YOUBOT_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in YOUBOT_GUI.M with the given input arguments.
%
%      YOUBOT_GUI('Property','Value',...) creates a new YOUBOT_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before YouBot_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to YouBot_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help YouBot_GUI

% Last Modified by GUIDE v2.5 27-Nov-2023 16:18:12

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @YouBot_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @YouBot_GUI_OutputFcn, ...
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


% --- Executes just before YouBot_GUI is made visible.
function YouBot_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to YouBot_GUI (see VARARGIN)

% Choose default command line output for YouBot_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes YouBot_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = YouBot_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function [handles] = connection_button_Callback(hObject, eventdata, handles)
    clear all
    clc
    
    % Retrieve the Remote API object
    handles.sim = remApi('remoteApi');

    % Close any outstanding connections
    handles.sim.simxFinish(-1);

    % Set details for localhost IP, port number, blocking mode, threading
    % mode, time until timeout, and number of connection attempts
    handles.id = handles.sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

    % Check connection is successful
    if handles.id > -1
        disp('Successfully connected MATLAB to CoppeliaSim.');

        % % Set initial position of each wheel
        rolling_joint_1_pos = 10;
        rolling_joint_2_pos = 10;
        rolling_joint_3_pos = 10;
        rolling_joint_4_pos = 10;
    
        % Get object handles for each wheel
        [r, handles.rolling_joint_1] = handles.sim.simxGetObjectHandle(handles.id, 'rollingJoint_fl', handles.sim.simx_opmode_blocking);
        [r, handles.rolling_joint_2] = handles.sim.simxGetObjectHandle(handles.id, 'rollingJoint_rl', handles.sim.simx_opmode_blocking);
        [r, handles.rolling_joint_3] = handles.sim.simxGetObjectHandle(handles.id, 'rollingJoint_rr', handles.sim.simx_opmode_blocking);
        [r, handles.rolling_joint_4] = handles.sim.simxGetObjectHandle(handles.id, 'rollingJoint_fr', handles.sim.simx_opmode_blocking);

        % Get object handles for each arm joint
        [r, handles.arm_joint_0] = handles.sim.simxGetObjectHandle(handles.id, 'youBotArmJoint0', handles.sim.simx_opmode_blocking);
        [r, handles.arm_joint_1] = handles.sim.simxGetObjectHandle(handles.id, 'youBotArmJoint1', handles.sim.simx_opmode_blocking);
        [r, handles.arm_joint_2] = handles.sim.simxGetObjectHandle(handles.id, 'youBotArmJoint2', handles.sim.simx_opmode_blocking);
        [r, handles.arm_joint_3] = handles.sim.simxGetObjectHandle(handles.id, 'youBotArmJoint3', handles.sim.simx_opmode_blocking);
        [r, handles.arm_joint_4] = handles.sim.simxGetObjectHandle(handles.id, 'youBotArmJoint4', handles.sim.simx_opmode_blocking);
    
        % Set target position of each joint based on slider value
        [r] = handles.sim.simxSetJointTargetPosition(handles.id, handles.rolling_joint_1, rolling_joint_1_pos, handles.sim.simx_opmode_blocking);
        [r] = handles.sim.simxSetJointTargetPosition(handles.id, handles.rolling_joint_2, rolling_joint_2_pos, handles.sim.simx_opmode_blocking);
        [r] = handles.sim.simxSetJointTargetPosition(handles.id, handles.rolling_joint_3, rolling_joint_3_pos, handles.sim.simx_opmode_blocking);
        [r] = handles.sim.simxSetJointTargetPosition(handles.id, handles.rolling_joint_4, rolling_joint_4_pos, handles.sim.simx_opmode_blocking);
    
    else
        disp('Failed to connect MATLAB to CoppeliaSim. Please check CoppeliaSim (V-Rep) simulation is running and try again.')
    end


% --- Executes on button press for move_forward_button.
function move_forward_button_Callback(hObject, eventdata, handles)
    handles = connection_button_Callback(hObject, eventdata, handles);

    % All the values were 2 before speed slider was added
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_1, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_2, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_3, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_4, 2, handles.sim.simx_opmode_blocking);


% --- Executes on button press for move_left_button.
function move_left_button_Callback(hObject, eventdata, handles)
    handles = connection_button_Callback(hObject, eventdata, handles);

    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_1, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_2, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_3, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_4, 0, handles.sim.simx_opmode_blocking);


% --- Executes on button press for move_right_button.
function move_right_button_Callback(hObject, eventdata, handles)
    handles = connection_button_Callback(hObject, eventdata, handles);

    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_1, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_2, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_3, 2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_4, 2, handles.sim.simx_opmode_blocking);


% --- Executes on button press for move_back_button.
function move_back_button_Callback(hObject, eventdata, handles)
    handles = connection_button_Callback(hObject, eventdata, handles);

    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_1, -2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_2, -2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_3, -2, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_4, -2, handles.sim.simx_opmode_blocking);


% --- Executes on button press for stop_button.
function stop_button_Callback(hObject, eventdata, handles)
    handles = connection_button_Callback(hObject, eventdata, handles);

    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_1, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_2, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_3, 0, handles.sim.simx_opmode_blocking);
    handles.sim.simxSetJointTargetVelocity(handles.id, handles.rolling_joint_4, 0, handles.sim.simx_opmode_blocking);


function slider1_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        
        % Joint angle limits to give increased range of motion (degrees)
        minJointAngle = -169;
        maxJointAngle = 169;
        
        % Slider limits
        handles.slider1.Min = minJointAngle;
        handles.slider1.Max = maxJointAngle;
        
        % Get current slider value
        slider_value = get(hObject, 'Value');
        
        % Slider value mapping to joint angle within specified range
        joint_angle = minJointAngle + (maxJointAngle - minJointAngle) * slider_value;
        
        % Set joint target position based on slider value
        handles.sim.simxSetJointTargetPosition(handles.id, handles.arm_joint_0, deg2rad(joint_angle), handles.sim.simx_opmode_blocking);
    catch exception
        disp('Error occurred when attempting to move Arm Joint 0 (base joint)');
        disp(exception.message);
    end


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        
        % Joint angle limits to give increased range of motion (degrees)
        minJointAngle = -65;
        maxJointAngle = 90;
        
        % Slider limits
        handles.slider1.Min = minJointAngle;
        handles.slider1.Max = maxJointAngle;
        
        % Get current slider value
        slider_value = get(hObject, 'Value');
        
        % Slider value mapping to joint angle within specified range
        joint_angle = minJointAngle + (maxJointAngle - minJointAngle) * slider_value;
        
        % Set joint target position based on slider value
        handles.sim.simxSetJointTargetPosition(handles.id, handles.arm_joint_1, deg2rad(joint_angle), handles.sim.simx_opmode_blocking);
    catch exception
        disp('Error occurred when attempting to move Arm Joint 1');
        disp(exception.message);
    end

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
    
        % Joint angle limits to give increased range of motion (degrees)
        minJointAngle = -151;
        maxJointAngle = 146;
        
        % Slider limits
        handles.slider1.Min = minJointAngle;
        handles.slider1.Max = maxJointAngle;
        
        % Get current slider value
        slider_value = get(hObject, 'Value');
        
        % Slider value mapping to joint angle within specified range
        joint_angle = minJointAngle + (maxJointAngle - minJointAngle) * slider_value;
        
        % Set joint target position based on slider value
        handles.sim.simxSetJointTargetPosition(handles.id, handles.arm_joint_2, deg2rad(joint_angle), handles.sim.simx_opmode_blocking);
    catch exception
        disp('Error occurred when attempting to move Arm Joint 2');
        disp(exception.message);
    end

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        
        % Joint angle limits to give increased range of motion (degrees)
        minJointAngle = -102.5;
        maxJointAngle = 102.5;
        
        % Slider limits
        handles.slider1.Min = minJointAngle;
        handles.slider1.Max = maxJointAngle;
        
        % Get current slider value
        slider_value = get(hObject, 'Value');
        
        % Slider value mapping to joint angle within specified range
        joint_angle = minJointAngle + (maxJointAngle - minJointAngle) * slider_value;
    
        % Set joint target position based on slider value
        handles.sim.simxSetJointTargetPosition(handles.id, handles.arm_joint_3, deg2rad(joint_angle), handles.sim.simx_opmode_blocking);
    catch exception
        disp('Error occurred when attempting to move Arm Joint 3');
        disp(exception.message);
    end

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        
        % Joint angle limits to give increased range of motion (degrees)
        minJointAngle = -167.5;
        maxJointAngle = 167.5;
        
        % Slider limits
        handles.slider1.Min = minJointAngle;
        handles.slider1.Max = maxJointAngle;
        
        % Get current slider value
        slider_value = get(hObject, 'Value');
        
        % Slider value mapping to joint angle within specified range
        joint_angle = minJointAngle + (maxJointAngle - minJointAngle) * slider_value;
        
        % Set joint target position based on slider value
        handles.sim.simxSetJointTargetPosition(handles.id, handles.arm_joint_4, deg2rad(joint_angle), handles.sim.simx_opmode_blocking);
    catch exception
        disp('Error occurred when attempting to move Arm Joint 4');
        disp(exception.message);
    end

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
    if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor',[.9 .9 .9]);
    end


% --- Executes on button press for capture_image_button (front view).
function capture_image_button_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');

        disp('Front vision sensor capture clicked. Capturing image...')
    
        [r, front_vision_sensor] = handles.sim.simxGetObjectHandle(handles.id, 'Vision_sensor', handles.sim.simx_opmode_blocking);
        handles.sim.simxSynchronousTrigger(handles.id);
    
        [r, resolution, image] = handles.sim.simxGetVisionSensorImage2(handles.id, front_vision_sensor, 0, handles.sim.simx_opmode_blocking);
    
        filename = ['Vision_Sensor_Images/Front_Vision_Image_' timestamp '.jpg'];

        imwrite(image, filename);
        disp('Image captured succesfully. Image saved to Vision_Sensor_Images folder.')

    catch exception
        disp('Error occurred during image capture:');
        disp(exception.message);
    end

% --- Executes on button press for capture_image_rear_button.
function capture_image_rear_button_Callback(hObject, eventdata, handles)
    try
        handles = connection_button_Callback(hObject, eventdata, handles);
        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
        disp('Rear vision sensor capture clicked. Capturing image...')
    
        [r, rear_vision_sensor] = handles.sim.simxGetObjectHandle(handles.id, 'Vision_sensor2', handles.sim.simx_opmode_blocking);
        handles.sim.simxSynchronousTrigger(handles.id);
    
        [r, resolution, image] = handles.sim.simxGetVisionSensorImage2(handles.id, rear_vision_sensor, 0, handles.sim.simx_opmode_blocking);
        
        filename = ['Vision_Sensor_Images/Rear_Vision_Image_' timestamp '.jpg'];
    
        imwrite(image, filename);
        disp('Image captured succesfully. Image saved to Vision_Sensor_Images folder.')
    
    catch exception
        disp('Error occurred during image capture:');
        disp(exception.message);
    end
