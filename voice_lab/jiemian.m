function varargout = jiemian(varargin)
% JIEMIAN MATLAB code for jiemian.fig
%      JIEMIAN, by itself, creates a new JIEMIAN or raises the existing
%      singleton*.
%
%      H = JIEMIAN returns the handle to a new JIEMIAN or the handle to
%      the existing singleton*.
%
%      JIEMIAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JIEMIAN.M with the given input arguments.
%
%      JIEMIAN('Property','Value',...) creates a new JIEMIAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before jiemian_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to jiemian_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help jiemian

% Last Modified by GUIDE v2.5 05-Jan-2013 12:16:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jiemian_OpeningFcn, ...
                   'gui_OutputFcn',  @jiemian_OutputFcn, ...
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


% --- Executes just before jiemian is made visible.
function jiemian_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to jiemian (see VARARGIN)

% Choose default command line output for jiemian
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes jiemian wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = jiemian_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton3.


function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  filename;
if nargin<1;action='initialized';end;
[filename,pname]=uigetfile('*.wav','Open Wave File');
set(handles.edit3, 'string' ,filename); 
set(handles.pushbutton3,'Enable','on');

% --- Executes on button press in radiobutton2.


function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global  filename;
[f,fs,bit]=wavread(filename);
sound(f,fs);
% global  filename;
% f=wavread(filename);
% sound(f,22050);

%result=simple(filename)+amdf1(filename)+amdf2(filename);
i=0;
if(simple(filename)==1)
    i=i+1;
end
if(amdf1(filename)==1)
    i=i+1;
end
if(amdf2(filename)==1)
    i=i+1;
end
if(i>=2)
    msgbox('该音频由男性发出');
else
    msgbox('该音频由女性发出');
end

%msgbox(strcat(char(simple(filename)+48),char(amdf1(filename)+48),char(amdf2(filename)+48)));
% --- Executes during object creation, after setting all properties.

function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
global  filename;
filename=get(hObject,'String');


% --- Executes on button press in pushbutton2.

function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')
    set(handles.edit2,'Enable','on');
    set(handles.radiobutton2,'Value',0);
    set(handles.edit1,'Enable','on');
    set(handles.edit3,'Enable','off');
    set(handles.pushbutton1,'Enable','on');
    set(handles.pushbutton2,'Enable','off');
    set(handles.pushbutton3,'Enable','off');
    set(handles.pushbutton1,'string','开始');
    set(handles.edit3,'string','0');
    
else
end;
% Hint: get(hObject,'Value') returns toggle state of radiobutton1



function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(hObject,'Value')
    set(handles.radiobutton1,'Value',0);
    set(handles.edit1,'Enable','off');
    set(handles.edit2,'Enable','off');
    set(handles.edit3,'Enable','on');
    set(handles.pushbutton1,'Enable','off');
    set(handles.pushbutton2,'Enable','on');
else
end;
% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in radiobutton1.



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 set(hObject,'string','录音中');
% pause(0.4);
Fs=str2double(get(handles.edit2,'String'));
t=str2double(get(handles.edit1,'String'));
% ai=analoginput('winsound',0);%初始化录音麦克
% chanel=addchannel(ai,1);%1表示单声道
% set(ai,'SampleRate',Fs); 
duration=t; %录音时间
% set(ai,'SamplesPerTrigger',duration*Fs);
% start(ai); 
% y=0;time=0;
% [y,time]=getdata(ai);%读出相应的数据
% handles.y=y;
handles.Fs=Fs;
% guidata(hObject,handles);
% ysize=size(handles.y);
% set(hObject,'string','完毕');
% set(hObject,'Enable','off');
fs=handles.Fs;
%      [filename]=uiputfile({'*.wav'},'文件保存');
% wavwrite(handles.y,fs,filename);%data表示声音数据

recObj = audiorecorder(fs, 16, 1);
get(recObj)

% Record your voice for 5 seconds.
recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, duration);
disp('End of Recording.');
 set(hObject,'string','完毕');

% Play back the recording.
%splay(recObj);

% Store data in double-precision array.
myRecording = getaudiodata(recObj);
% fs=handles.Fs;
       [filename]=uiputfile({'*.wav'},'文件保存');
 wavwrite(myRecording,filename);%data表示声音数据

% Plot the waveform.
%plot(myRecording);



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
