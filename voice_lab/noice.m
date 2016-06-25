function varargout = noice(varargin)
% NOICE MATLAB code for noice.fig
%      NOICE, by itself, creates a new NOICE or raises the existing
%      singleton*.
%
%      H = NOICE returns the handle to a new NOICE or the handle to
%      the existing singleton*.
%
%      NOICE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in NOICE.M with the given input arguments.
%
%      NOICE('Property','Value',...) creates a new NOICE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before noice_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to noice_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help noice

% Last Modified by GUIDE v2.5 15-Jun-2016 19:37:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @noice_OpeningFcn, ...
                   'gui_OutputFcn',  @noice_OutputFcn, ...
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


% --- Executes just before noice is made visible.
function noice_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to noice (see VARARGIN)

% Choose default command line output for noice
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes noice wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = noice_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
%打开文件
[file path]=uigetfile('*.wav;*.mat');
file=[path file];
global y
global fs
[s,fs]=wavread(file);       %语音信号的采集
y=s(:,1);
y=y-mean(y);                            % 消除直流分量
y=y/max(abs(y));                        % 幅值归一化
%sound(y,fs);  %语音信号的播放
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
%原始音频文件分析
global y
global fs
global ts
n=length(y) ; 
ts=(0:n-1)/fs;                           % 设置时间
Y=fft(y,n);                          %快速傅里叶变换
sound(y,fs);

axes(handles.axes1);
plot(ts,y,'b');
title('原始信号波形','fontweight','bold');
%axis([0 4 -1 1]);
xlabel(['时间/s' 10 ]); 
ylabel('幅值');
% axis([0 3.5*10^4 -0.25 0.25]);
grid;

axes(handles.axes2);
plot(abs(Y),'r');
title('原始信号频谱','fontweight','bold');
ylabel('幅值');
grid;
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
%预处理
global y
global ty
global fs
s=y;
N=length(s);                            % 求出信号长度
t=(0:N-1)/fs;                           % 设置时间
% 低通滤波保留60~450HZ音频
fp=1500; fs1=1750;                         % 设置滤波器的通带和阻带频率
Fs=8000; Fs2=Fs/2;                      % 采样频率
Wp=fp/Fs2; Ws=fs1/Fs2;                   % 把通带和阻带频率归一化
Rp=3; Rs=50;                            % 通带波纹和阻带衰减
[n,Wn]=cheb2ord(Wp,Ws,Rp,Rs);           % 求取滤波器阶数
[b1,a]=cheby2(n,Rs,Wn);                  % 设计契比雪夫II型低通滤波器系数
ty1=filter(b1,a,s);                        % 把语音信号通过滤波器
% 高通滤波，50HZ工频
As=50;Fs=8000; Fs2=Fs/2;                % 最小衰减和采样频率
fs1=49; fs2=51;                         % 阻带频率
fp1=45; fp2=55;                         % 通带频率
df=min(fs1-fp1,fp2-fs2);                % 求过渡带宽
M0=round((As-7.95)/(14.36*df/Fs))+2;    % 按式(5-5-4)求凯泽窗长
M=M0+mod(M0+1,2);                       % 保证窗长为奇数
wp1=fp1/Fs2*pi; wp2=fp2/Fs2*pi;         % 转换成归一化圆频率
ws1=fs1/Fs2*pi; ws2=fs2/Fs2*pi;
wc1=(wp1+ws1)/2; wc2=(wp2+ws2)/2;       % 求截止频率
beta=0.5842*(As-21)^0.4+0.07886*(As-21);% 按式(5-5-5)求出beta值
fprintf('beta=%5.6f\n',beta);
M=M-1;                                  % 阶次和窗长差1
b=fir1(M,[wc1 wc2]/pi,'stop',kaiser(M+1,beta));  % 计算FIR滤波器系数
[h,w]=freqz(b,1,4000);                  % 求幅值的频率响应
db=20*log10(abs(h));
% ty2=conv(b,ty1);                            % FIR带陷滤波，输出为y
ty2=conv(b,ty1);
ty=ty2(fix(M/2)+1:end-fix(M/2));           % 消除conv带来的滤波器输出延迟的影响

%作图
% ty=ty1;
sound(ty,fs);
axes(handles.axes3);                                
plot(t,ty,'b'); 
title('预处理后的语音信号','fontweight','bold')
%axis([0 4 -1 1]);
xlabel(['时间/s' 10 ]); 
ylabel('幅值');
% axis([0 4.5 -0.4 0.3]);
grid;
m11=fft(ty);  %求滤波后的信号
axes(handles.axes4);
plot(abs(m11),'r');
title('预处理后信号的频谱','fontweight','bold');
ylabel('幅值');
%axis([ 0 150000 0 4000]);
grid;
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
%加噪
global ty
global n
global fs
global time
global r1
global r2
Ss=ty;
n = length (Ss) ; 
time=(0:n-1)/fs;
r2=0.2*randn(n,1);%高斯白燥
r1=Ss+r2; %产生加性噪声
sound(r1,fs); 
s=r1;
axes(handles.axes1);
plot(time,s);
title('加噪语音信号的时域波形','fontweight','bold');
grid;
S=fft(s);            
%subplot(2,1,2);
axes(handles.axes2);
plot(abs(S),'r');
title('加噪语音信号的频域波形','fontweight','bold');
grid;
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
%LMS降噪
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fs
global time
global r1
%  %谱减法降噪
% IS=0.15;                                % 前导无话段长度(s)
% alpha=2.8;                              % 过减因子
% beta=0.001;                             % 增益补偿因子
% %c=0时,用功率谱计算增益矩阵不进行开方运算,c=1时,进行开方运算
% c=1;        
% % N=length(r1);                       % 信号长度
% % time=(0:N-1)/fs;                        % 设置时间
% wlen=200;                               % 设置帧长
% inc=80;                                 % 设置帧移
% NIS=fix((IS*fs-wlen)/inc +1);           % 前导无话段帧数
% output=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% 多窗谱改进谱减法减噪处理
% % snr2=SNR_singlech(x,output);            % 计算谱减后的信噪比
% % snr=snr2-snr1;
% % fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% sound(r1,fs);                     % 从声卡发声比较
% pause(1)
% sound(output,fs);
% %LMS自适应降噪
global r2
M=32;                                   % 设置Ｍ和mu
mu=0.001;  
h = adaptfilt.lms(M,mu);                % LMS滤波
[yy,e] = filter(h,r2,r1);
% [yy,e] = filter(h,r2,output1);
[yyy,e2] = filter(h,r2,e);
output=e2;                               % LMS滤波输出
pause(1)
sound(output,fs);

axes(handles.axes3);
plot(time,output,'b'); ylabel('幅值');
title('LMS滤波输出语音信号','fontweight','bold');
grid;axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('幅值')
title('加噪语音信号的频域波形','fontweight','bold');
grid;
%hObject    handle to pushbutton6 (see GCBO)
%eventdata  reserved - to be defined in a future version of MATLAB
%handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
%录音
%time=str2num(get(handles.edit10,'String'));
time = 10;
global Fs nBits y;
Fs=50000;
nBits=16;
recObj=audiorecorder(Fs,nBits,2,-1);
msgbox('started');
recordblocking(recObj,time);
y=getaudiodata(recObj);
msgbox('completed');
msgbox('play');
sound(y)
guidata(hObject,handles);
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
%录音降噪
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%  %谱减法降噪
% IS=0.15;                                % 前导无话段长度(s)
% alpha=2.8;                              % 过减因子
% beta=0.001;                             % 增益补偿因子
% %c=0时,用功率谱计算增益矩阵不进行开方运算,c=1时,进行开方运算
% c=1;        
% % N=length(r1);                       % 信号长度
% % time=(0:N-1)/fs;                        % 设置时间
% wlen=200;                               % 设置帧长
% inc=80;                                 % 设置帧移
% NIS=fix((IS*fs-wlen)/inc +1);           % 前导无话段帧数
% output1=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% 多窗谱改进谱减法减噪处理
% % snr2=SNR_singlech(x,output);            % 计算谱减后的信噪比
% % snr=snr2-snr1;
% % fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% wavplay(r1,fs);                     % 从声卡发声比较
% pause(1)
% wavplay(output1,fs);


% %LMS自适应降噪
global fs
global ts
global ty
n = length (ty);
r1=ty;
r2=0.8*randn(n,1);
M=32;                                   % 设置Ｍ和mu
mu=0.001;  
h = adaptfilt.lms(M,mu);                % LMS滤波
[yy,e] = filter(h,r2,r1);
[yyy,e2] = filter(h,r2,e);
output=e2;                               % LMS滤波输出
pause(1)
sound(output,fs);

axes(handles.axes3);
plot(ts,output,'b'); ylabel('幅值');
title('LMS滤波输出语音信号','fontweight','bold');
grid;axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('幅值')
title('加噪语音信号的频域波形','fontweight','bold');
grid;
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
%LPC合成
global ty
global n
global fs
global ts
x=ty;
% x=r1;
xl=n;
p=12;                                   % LPC的阶数为12
wlen=200; inc=80;                       % 帧长和帧移
msoverlap = wlen - inc;                 % 每帧重叠部分的长度
y=enframe(x,wlen,inc)';                 % 分帧
fn=size(y,2);                           % 取帧数
% 语音分析:求每一帧的LPC系数和预测误差
for i=1 : fn                            
    u=y(:,i);                           % 取来一帧
    A=lpc(u,p);                         % LPC求得系数
    aCoeff(:,i)=A;                      % 存放在aCoeff数组中
    errSig = filter(A,1,u);             % 计算预测误差序列
    resid(:,i) = errSig;                % 存放在resid数组中
end
% 语音合成:求每一帧的合成语音叠接成连续语音信号
for i=1:fn                              
    A = aCoeff(:,i);                    % 取得该帧的预测系数
    residFrame = resid(:,i);            % 取得该帧的预测误差
    synFrame = filter(1, A', residFrame); % 预测误差激励,合成语音
    
    outspeech((i-1)*inc+1:i*inc)=synFrame(1:inc);  % 重叠存储法存放数据
% 如果是最后一帧,把inc后的数据补上
    if i==fn                            
        outspeech(fn*inc+1:(fn-1)*inc+wlen)=synFrame(inc+1:wlen);
    end

end;
ol=length(outspeech);
if ol<xl                                % 把outspeech补零,使与x等长
    outspeech=[outspeech zeros(1,xl-ol)];
end
xt=max(ol,xl);
Tss=(0:xt-1)/fs;
% 发声
sound(x,fs);
% pause(1)
sound(outspeech,fs);
% 作图
% subplot 211;
axes(handles.axes1);
plot(ts,x,'b');
xlabel(['时间/s' 10 ]); ylabel('幅值');
title('原始语音信号');grid;axis([0 4 -1 1]);
m1=fft(x);  
axes(handles.axes2);
plot(abs(m1),'r');
title('合成的语音信号的频谱','fontweight','bold');
ylabel('幅值');
grid;

% subplot 212;
axes(handles.axes3);
plot(Tss,outspeech,'b');
xlabel(['时间/s' 10 ]); ylabel('幅值');
title('合成的语音信号');grid;axis([0 4 -1 1]);

m11=fft(outspeech);  
axes(handles.axes4);
plot(abs(m11),'r');
title('合成的语音信号的频谱','fontweight','bold');
ylabel('幅值');
grid;
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
%说话人性别识别
Fs=50000;
nBits=16;
recObj=audiorecorder(Fs,nBits,2,-1);
msgbox('Speak now for 3 seconds');
recordblocking(recObj,3);
y=getaudiodata(recObj);
M=size(y,1);
f=(0:1:M-1).'*Fs/M;
yf=abs(fft(y(:,1)));
[C I]=max(yf);
fmax=f(I);
if fmax<1000
msgbox('Male Voice');
else 
msgbox('Female Voice');
end
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
%低通滤波器
% global b1
% global a
fp=1600; fs=1750;                         % 设置滤波器的通带和阻带频率
Fs=8000; Fs2=Fs/2;                      % 采样频率
Wp=fp/Fs2; Ws=fs/Fs2;                   % 把通带和阻带频率归一化
Rp=3; Rs=50;                            % 通带波纹和阻带衰减
[n,Wn]=cheb2ord(Wp,Ws,Rp,Rs);           % 求取滤波器阶数
[b1,a]=cheby2(n,Rs,Wn);                  % 设计契比雪夫II型低通滤波器系数
[db,mag,pha,grd,w]=freqz_m(b1,a);        % 求滤波器的频率响应曲线
axes(handles.axes1);
plot(w/pi*Fs2,db,'k','linewidth',2)
grid; axis([0 4000 -100 5]);
title('cheby2 lowpass幅值响应曲线')
xlabel('频率/Hz'); ylabel('幅值/dB');
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
%高通滤波器
% global b
% global M
As=50;Fs=8000; Fs2=Fs/2;               % 阻带最小衰减和采样频率
fp=75; fs=60;                          % 通带阻带频率
df=fp-fs;                              % 求取过渡带
M0=round((As-7.95)/(14.36*df/Fs))+2;   % 按式(5-5-4)求凯泽窗长
M=M0+mod(M0+1,2);                      % 保证窗长为奇数
wp=fp/Fs2*pi; ws=fs/Fs2*pi;            % 转为圆频率
wc=(wp+ws)/2;                          % 求取截止频率
beta=0.5842*(As-21)^0.4+0.07886*(As-21);% 按式(5-5-5)求出beta值
fprintf('beta=%5.6f\n',beta);          % 显示beta的数值
w_kai=(kaiser(M,beta))';               % 求凯泽窗
hd=ideal_lp(pi,M)-ideal_lp(wc,M);      % 求理想滤波器的脉冲响应(高通滤波器的组合)
b=hd.*w_kai;                           % 理想脉冲响应与窗函数相乘
[h,w]=freqz(b,1,4000);                 % 求频率响应
db=20*log10(abs(h));
axes(handles.axes2);
plot(w/pi*Fs2,db,'k','linewidth',2); grid;
axis([0 150 -100 10]);
title('kaiser highpass幅频响应曲线');
xlabel('频率/Hz');ylabel('幅值/dB');
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
%退出
clc;clear;close all;
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
%谱减法减噪
global fs
global time
global r1
IS=0.15;                                % 前导无话段长度(s)
alpha=2.8;                              % 过减因子
beta=0.001;                             % 增益补偿因子
%c=0时,用功率谱计算增益矩阵不进行开方运算,c=1时,进行开方运算
c=1;        
% N=length(r1);                       % 信号长度
% time=(0:N-1)/fs;                        % 设置时间
wlen=200;                               % 设置帧长
inc=80;                                 % 设置帧移
NIS=fix((IS*fs-wlen)/inc +1);           % 前导无话段帧数
output=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% 多窗谱改进谱减法减噪处理
% snr2=SNR_singlech(x,output);            % 计算谱减后的信噪比
% snr=snr2-snr1;
% fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% sound(r1,fs);                     % 从声卡发声比较
% pause(1)
sound(output,fs);

%作图
axes(handles.axes3);
plot(time,output,'b'); ylabel('幅值');
title('谱减法输出语音信号','fontweight','bold');
grid;
%axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('幅值')
title('加噪语音信号的频域波形','fontweight','bold');
grid;
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
%LPC合成
global ty
global n
global fs
global ts
x=ty;
% x=r1;
xl=n;
p=12;                                   % LPC的阶数为12
wlen=200; inc=80;                       % 帧长和帧移
msoverlap = wlen - inc;                 % 每帧重叠部分的长度
y=enframe(x,wlen,inc)';                 % 分帧
fn=size(y,2);                           % 取帧数
% 语音分析:求每一帧的LPC系数和预测误差
for i=1 : fn                            
    u=y(:,i);                           % 取来一帧
    A=lpc(u,p);                         % LPC求得系数
    aCoeff(:,i)=A;                      % 存放在aCoeff数组中
    errSig = filter(A,1,u);             % 计算预测误差序列
    resid(:,i) = errSig;                % 存放在resid数组中
end
% 语音合成:求每一帧的合成语音叠接成连续语音信号
for i=1:fn                              
    A = aCoeff(:,i);                    % 取得该帧的预测系数
    residFrame = resid(:,i);            % 取得该帧的预测误差
    synFrame = filter(1, A', residFrame); % 预测误差激励,合成语音
    
    outspeech((i-1)*inc+1:i*inc)=synFrame(1:inc);  % 重叠存储法存放数据
% 如果是最后一帧,把inc后的数据补上
    if i==fn                            
        outspeech(fn*inc+1:(fn-1)*inc+wlen)=synFrame(inc+1:wlen);
    end

end;
ol=length(outspeech);
if ol<xl                                % 把outspeech补零,使与x等长
    outspeech=[outspeech zeros(1,xl-ol)];
end
xt=max(ol,xl);
Tss=(0:xt-1)/fs;
% 发声
sound(x,fs);
% pause(1)
sound(outspeech,fs);
% 作图
% subplot 211;
axes(handles.axes1);
plot(ts,x,'b');
xlabel(['时间/s' 10 ]); ylabel('幅值');
title('原始语音信号');grid;axis([0 4 -1 1]);
m1=fft(x);  
axes(handles.axes2);
plot(abs(m1),'r');
title('合成的语音信号的频谱','fontweight','bold');
ylabel('幅值');
grid;

% subplot 212;
axes(handles.axes3);
plot(Tss,outspeech,'b');
xlabel(['时间/s' 10 ]); ylabel('幅值');
title('合成的语音信号');grid;axis([0 4 -1 1]);

m11=fft(outspeech);  
axes(handles.axes4);
plot(abs(m11),'r');
title('合成的语音信号的频谱','fontweight','bold');
ylabel('幅值');
grid;
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
%LPC
global y;
global fs;
%[x,fs]=wavread('speech.wav');%读入数据
x = y;
% 预加重滤波器
xx=double(x);
y=filter([1 -0.9495],1,xx);
N=160;
y1=y(1:N);
w1=hamming(N);
y2=(y1.*w1)';%加窗 取一帧数据
p=30;%预测阶数
%首先求自相关函数
r=zeros(1,p+1);
for k=1:p+1
sum=0;
for m=1:N+1-k
sum=sum+y2(m).*y2(m-1+k)';
end
r(k)=sum;
end
%根据durbin算法求线性预测系数
k=zeros(1,p);
k(1)=r(2)/r(1);
a=zeros(p,p);
a(1,1)=k(1);
e=zeros(1,p);
e(1)=(1-k(1)^2)*r(1);
%递推过程
for i=2:p
c=zeros(1,i);
sum=0;
for j=1:i-1
sum=sum+(a(i-1,j).*r(i+1-j));
end
c(i)=sum;
k(i)=(r(i+1)-c(i))/e(i-1);%求反射系数
if find(abs(k)>1) 
    disp('default')
else
%subplot(413);
axes(handles.axes3);
plot(abs(k));title('|k(i)|<=1')
end
a(i,i)=k(i);
for j=1:i-1
a(i,j)=a(i-1,j)-k(i).*a(i-1,i-j);
end
e(i)=(1-k(i)^2)*e(i-1);%预测器残差能量
%subplot(414);
axes(handles.axes4);
plot(e);title('预测器残差能量E(i)')
end
%递推结束后提取预测系数
d=zeros(1,p); 
for t=1:p
d(t)=a(p,t);
end
z=zeros(1,N);
for i=1:p
z(i)=y2(i);

end
figure(1);
%subplot(411);
axes(handles.axes1);
plot(y2);title('原始数据')
%subplot(412);
axes(handles.axes2);
plot(z);title('durbin算法求线性预测系数')
figure(1);
subplot(411);plot(y2);title('原始数据')
subplot(412);plot(z);title('durbin算法求线性预测系数')
subplot(413);plot(abs(k));title('|k(i)|<=1')
subplot(414);plot(e);title('预测器残差能量E(i)')


% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
jiemian
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
