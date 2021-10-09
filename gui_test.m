function varargout = radarSimulation(varargin)
% RADARSIMULATION M-file for radarSimulation.fig
%      RADARSIMULATION, by itself, creates a new RADARSIMULATION or raises the existing
%      singleton*.
%
%      H = RADARSIMULATION returns the handle to a new RADARSIMULATION or the handle to
%      the existing singleton*.
%
%      RADARSIMULATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RADARSIMULATION.M with the given input
%      arguments.
%
%      RADARSIMULATION('Property','Value',...) creates a new RADARSIMULATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before radarSimulation_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to radarSimulation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
 
% Copyright 2002-2003 The MathWorks, Inc.
 
% Edit the above text to modify the response to help radarSimulation
 
% Last Modified by GUIDE v2.5 15-Mar-2008 11:30:35
 
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @radarSimulation_OpeningFcn, ...
                   'gui_OutputFcn',  @radarSimulation_OutputFcn, ...
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
 
 
% --- Executes just before radarSimulation is made visible.
function radarSimulation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to radarSimulation (see VARARGIN)
 
% Choose default command line output for radarSimulation
handles.output = hObject;
handles.FOV = [];
handles.mountains = [];
handles.IF_Freq = 3e7;    
handles.currentTime = 0;
handles.targetsFigure = [];
handles.Targets = [];
handles.pulseNum = 0;
handles.plotedTargets = [];
plotDistLines(handles.radarDisplay,10);
 
PW_Callback(handles.PW, eventdata, handles);    % updating the current PW value
 
% Update handles structure
guidata(hObject, handles);
reset_Callback(hObject, eventdata, handles)
 
% UIWAIT makes radarSimulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);
 
 
% --- Outputs from this function are returned to the command line.
function varargout = radarSimulation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Get default command line output from handles structure
varargout{1} = handles.output;
 
 
 
function PRI_Callback(hObject, eventdata, handles)
% hObject    handle to PRI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hints: get(hObject,'String') returns contents of PRI as text
%        str2double(get(hObject,'String')) returns contents of PRI as a double
 
 
% --- Executes during object creation, after setting all properties.
function PRI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PRI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end
 
 
 
function ZSA_Callback(hObject, eventdata, handles)
% hObject    handle to ZSA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hints: get(hObject,'String') returns contents of ZSA as text
%        str2double(get(hObject,'String')) returns contents of ZSA as a double
 
 
% --- Executes during object creation, after setting all properties.
function ZSA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ZSA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc
    set(hObject,'BackgroundColor','white');
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end
 
 
% --- Executes on slider movement.
function PW_Callback(hObject, eventdata, handles)
    percent = get(hObject,'value');
    str = ['PW = ' num2str(percent*100) '% of the PRI'];
    set(handles.PWstr,'string',str);
% hObject    handle to PW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 
 
% --- Executes during object creation, after setting all properties.
function PW_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PW (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
% Hint: slider controls usually have a light gray background, change
%       'usewhitebg' to 0 to use default.  See ISPC and COMPUTER.
usewhitebg = 1;
if usewhitebg
    set(hObject,'BackgroundColor',[.9 .9 .9]);
else
    set(hObject,'BackgroundColor',get(0,'defaultUicontrolBackgroundColor'));
end
