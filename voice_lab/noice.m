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
%���ļ�
[file path]=uigetfile('*.wav;*.mat');
file=[path file];
global y
global fs
[s,fs]=wavread(file);       %�����źŵĲɼ�
y=s(:,1);
y=y-mean(y);                            % ����ֱ������
y=y/max(abs(y));                        % ��ֵ��һ��
%sound(y,fs);  %�����źŵĲ���
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
%ԭʼ��Ƶ�ļ�����
global y
global fs
global ts
n=length(y) ; 
ts=(0:n-1)/fs;                           % ����ʱ��
Y=fft(y,n);                          %���ٸ���Ҷ�任
sound(y,fs);

axes(handles.axes1);
plot(ts,y,'b');
title('ԭʼ�źŲ���','fontweight','bold');
%axis([0 4 -1 1]);
xlabel(['ʱ��/s' 10 ]); 
ylabel('��ֵ');
% axis([0 3.5*10^4 -0.25 0.25]);
grid;

axes(handles.axes2);
plot(abs(Y),'r');
title('ԭʼ�ź�Ƶ��','fontweight','bold');
ylabel('��ֵ');
grid;
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
%Ԥ����
global y
global ty
global fs
s=y;
N=length(s);                            % ����źų���
t=(0:N-1)/fs;                           % ����ʱ��
% ��ͨ�˲�����60~450HZ��Ƶ
fp=1500; fs1=1750;                         % �����˲�����ͨ�������Ƶ��
Fs=8000; Fs2=Fs/2;                      % ����Ƶ��
Wp=fp/Fs2; Ws=fs1/Fs2;                   % ��ͨ�������Ƶ�ʹ�һ��
Rp=3; Rs=50;                            % ͨ�����ƺ����˥��
[n,Wn]=cheb2ord(Wp,Ws,Rp,Rs);           % ��ȡ�˲�������
[b1,a]=cheby2(n,Rs,Wn);                  % �������ѩ��II�͵�ͨ�˲���ϵ��
ty1=filter(b1,a,s);                        % �������ź�ͨ���˲���
% ��ͨ�˲���50HZ��Ƶ
As=50;Fs=8000; Fs2=Fs/2;                % ��С˥���Ͳ���Ƶ��
fs1=49; fs2=51;                         % ���Ƶ��
fp1=45; fp2=55;                         % ͨ��Ƶ��
df=min(fs1-fp1,fp2-fs2);                % ����ɴ���
M0=round((As-7.95)/(14.36*df/Fs))+2;    % ��ʽ(5-5-4)���󴰳�
M=M0+mod(M0+1,2);                       % ��֤����Ϊ����
wp1=fp1/Fs2*pi; wp2=fp2/Fs2*pi;         % ת���ɹ�һ��ԲƵ��
ws1=fs1/Fs2*pi; ws2=fs2/Fs2*pi;
wc1=(wp1+ws1)/2; wc2=(wp2+ws2)/2;       % ���ֹƵ��
beta=0.5842*(As-21)^0.4+0.07886*(As-21);% ��ʽ(5-5-5)���betaֵ
fprintf('beta=%5.6f\n',beta);
M=M-1;                                  % �״κʹ�����1
b=fir1(M,[wc1 wc2]/pi,'stop',kaiser(M+1,beta));  % ����FIR�˲���ϵ��
[h,w]=freqz(b,1,4000);                  % ���ֵ��Ƶ����Ӧ
db=20*log10(abs(h));
% ty2=conv(b,ty1);                            % FIR�����˲������Ϊy
ty2=conv(b,ty1);
ty=ty2(fix(M/2)+1:end-fix(M/2));           % ����conv�������˲�������ӳٵ�Ӱ��

%��ͼ
% ty=ty1;
sound(ty,fs);
axes(handles.axes3);                                
plot(t,ty,'b'); 
title('Ԥ�����������ź�','fontweight','bold')
%axis([0 4 -1 1]);
xlabel(['ʱ��/s' 10 ]); 
ylabel('��ֵ');
% axis([0 4.5 -0.4 0.3]);
grid;
m11=fft(ty);  %���˲�����ź�
axes(handles.axes4);
plot(abs(m11),'r');
title('Ԥ������źŵ�Ƶ��','fontweight','bold');
ylabel('��ֵ');
%axis([ 0 150000 0 4000]);
grid;
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
%����
global ty
global n
global fs
global time
global r1
global r2
Ss=ty;
n = length (Ss) ; 
time=(0:n-1)/fs;
r2=0.2*randn(n,1);%��˹����
r1=Ss+r2; %������������
sound(r1,fs); 
s=r1;
axes(handles.axes1);
plot(time,s);
title('���������źŵ�ʱ����','fontweight','bold');
grid;
S=fft(s);            
%subplot(2,1,2);
axes(handles.axes2);
plot(abs(S),'r');
title('���������źŵ�Ƶ����','fontweight','bold');
grid;
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
%LMS����
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global fs
global time
global r1
%  %�׼�������
% IS=0.15;                                % ǰ���޻��γ���(s)
% alpha=2.8;                              % ��������
% beta=0.001;                             % ���油������
% %c=0ʱ,�ù����׼���������󲻽��п�������,c=1ʱ,���п�������
% c=1;        
% % N=length(r1);                       % �źų���
% % time=(0:N-1)/fs;                        % ����ʱ��
% wlen=200;                               % ����֡��
% inc=80;                                 % ����֡��
% NIS=fix((IS*fs-wlen)/inc +1);           % ǰ���޻���֡��
% output=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% �ര�׸Ľ��׼������봦��
% % snr2=SNR_singlech(x,output);            % �����׼���������
% % snr=snr2-snr1;
% % fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% sound(r1,fs);                     % �����������Ƚ�
% pause(1)
% sound(output,fs);
% %LMS����Ӧ����
global r2
M=32;                                   % ���ãͺ�mu
mu=0.001;  
h = adaptfilt.lms(M,mu);                % LMS�˲�
[yy,e] = filter(h,r2,r1);
% [yy,e] = filter(h,r2,output1);
[yyy,e2] = filter(h,r2,e);
output=e2;                               % LMS�˲����
pause(1)
sound(output,fs);

axes(handles.axes3);
plot(time,output,'b'); ylabel('��ֵ');
title('LMS�˲���������ź�','fontweight','bold');
grid;axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('��ֵ')
title('���������źŵ�Ƶ����','fontweight','bold');
grid;
%hObject    handle to pushbutton6 (see GCBO)
%eventdata  reserved - to be defined in a future version of MATLAB
%handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
%¼��
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
%¼������
% hObject    handle to pushbutton20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%  %�׼�������
% IS=0.15;                                % ǰ���޻��γ���(s)
% alpha=2.8;                              % ��������
% beta=0.001;                             % ���油������
% %c=0ʱ,�ù����׼���������󲻽��п�������,c=1ʱ,���п�������
% c=1;        
% % N=length(r1);                       % �źų���
% % time=(0:N-1)/fs;                        % ����ʱ��
% wlen=200;                               % ����֡��
% inc=80;                                 % ����֡��
% NIS=fix((IS*fs-wlen)/inc +1);           % ǰ���޻���֡��
% output1=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% �ര�׸Ľ��׼������봦��
% % snr2=SNR_singlech(x,output);            % �����׼���������
% % snr=snr2-snr1;
% % fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% wavplay(r1,fs);                     % �����������Ƚ�
% pause(1)
% wavplay(output1,fs);


% %LMS����Ӧ����
global fs
global ts
global ty
n = length (ty);
r1=ty;
r2=0.8*randn(n,1);
M=32;                                   % ���ãͺ�mu
mu=0.001;  
h = adaptfilt.lms(M,mu);                % LMS�˲�
[yy,e] = filter(h,r2,r1);
[yyy,e2] = filter(h,r2,e);
output=e2;                               % LMS�˲����
pause(1)
sound(output,fs);

axes(handles.axes3);
plot(ts,output,'b'); ylabel('��ֵ');
title('LMS�˲���������ź�','fontweight','bold');
grid;axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('��ֵ')
title('���������źŵ�Ƶ����','fontweight','bold');
grid;
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
%LPC�ϳ�
global ty
global n
global fs
global ts
x=ty;
% x=r1;
xl=n;
p=12;                                   % LPC�Ľ���Ϊ12
wlen=200; inc=80;                       % ֡����֡��
msoverlap = wlen - inc;                 % ÿ֡�ص����ֵĳ���
y=enframe(x,wlen,inc)';                 % ��֡
fn=size(y,2);                           % ȡ֡��
% ��������:��ÿһ֡��LPCϵ����Ԥ�����
for i=1 : fn                            
    u=y(:,i);                           % ȡ��һ֡
    A=lpc(u,p);                         % LPC���ϵ��
    aCoeff(:,i)=A;                      % �����aCoeff������
    errSig = filter(A,1,u);             % ����Ԥ���������
    resid(:,i) = errSig;                % �����resid������
end
% �����ϳ�:��ÿһ֡�ĺϳ��������ӳ����������ź�
for i=1:fn                              
    A = aCoeff(:,i);                    % ȡ�ø�֡��Ԥ��ϵ��
    residFrame = resid(:,i);            % ȡ�ø�֡��Ԥ�����
    synFrame = filter(1, A', residFrame); % Ԥ������,�ϳ�����
    
    outspeech((i-1)*inc+1:i*inc)=synFrame(1:inc);  % �ص��洢���������
% ��������һ֡,��inc������ݲ���
    if i==fn                            
        outspeech(fn*inc+1:(fn-1)*inc+wlen)=synFrame(inc+1:wlen);
    end

end;
ol=length(outspeech);
if ol<xl                                % ��outspeech����,ʹ��x�ȳ�
    outspeech=[outspeech zeros(1,xl-ol)];
end
xt=max(ol,xl);
Tss=(0:xt-1)/fs;
% ����
sound(x,fs);
% pause(1)
sound(outspeech,fs);
% ��ͼ
% subplot 211;
axes(handles.axes1);
plot(ts,x,'b');
xlabel(['ʱ��/s' 10 ]); ylabel('��ֵ');
title('ԭʼ�����ź�');grid;axis([0 4 -1 1]);
m1=fft(x);  
axes(handles.axes2);
plot(abs(m1),'r');
title('�ϳɵ������źŵ�Ƶ��','fontweight','bold');
ylabel('��ֵ');
grid;

% subplot 212;
axes(handles.axes3);
plot(Tss,outspeech,'b');
xlabel(['ʱ��/s' 10 ]); ylabel('��ֵ');
title('�ϳɵ������ź�');grid;axis([0 4 -1 1]);

m11=fft(outspeech);  
axes(handles.axes4);
plot(abs(m11),'r');
title('�ϳɵ������źŵ�Ƶ��','fontweight','bold');
ylabel('��ֵ');
grid;
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
%˵�����Ա�ʶ��
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
%��ͨ�˲���
% global b1
% global a
fp=1600; fs=1750;                         % �����˲�����ͨ�������Ƶ��
Fs=8000; Fs2=Fs/2;                      % ����Ƶ��
Wp=fp/Fs2; Ws=fs/Fs2;                   % ��ͨ�������Ƶ�ʹ�һ��
Rp=3; Rs=50;                            % ͨ�����ƺ����˥��
[n,Wn]=cheb2ord(Wp,Ws,Rp,Rs);           % ��ȡ�˲�������
[b1,a]=cheby2(n,Rs,Wn);                  % �������ѩ��II�͵�ͨ�˲���ϵ��
[db,mag,pha,grd,w]=freqz_m(b1,a);        % ���˲�����Ƶ����Ӧ����
axes(handles.axes1);
plot(w/pi*Fs2,db,'k','linewidth',2)
grid; axis([0 4000 -100 5]);
title('cheby2 lowpass��ֵ��Ӧ����')
xlabel('Ƶ��/Hz'); ylabel('��ֵ/dB');
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
%��ͨ�˲���
% global b
% global M
As=50;Fs=8000; Fs2=Fs/2;               % �����С˥���Ͳ���Ƶ��
fp=75; fs=60;                          % ͨ�����Ƶ��
df=fp-fs;                              % ��ȡ���ɴ�
M0=round((As-7.95)/(14.36*df/Fs))+2;   % ��ʽ(5-5-4)���󴰳�
M=M0+mod(M0+1,2);                      % ��֤����Ϊ����
wp=fp/Fs2*pi; ws=fs/Fs2*pi;            % תΪԲƵ��
wc=(wp+ws)/2;                          % ��ȡ��ֹƵ��
beta=0.5842*(As-21)^0.4+0.07886*(As-21);% ��ʽ(5-5-5)���betaֵ
fprintf('beta=%5.6f\n',beta);          % ��ʾbeta����ֵ
w_kai=(kaiser(M,beta))';               % ����
hd=ideal_lp(pi,M)-ideal_lp(wc,M);      % �������˲�����������Ӧ(��ͨ�˲��������)
b=hd.*w_kai;                           % ����������Ӧ�봰�������
[h,w]=freqz(b,1,4000);                 % ��Ƶ����Ӧ
db=20*log10(abs(h));
axes(handles.axes2);
plot(w/pi*Fs2,db,'k','linewidth',2); grid;
axis([0 150 -100 10]);
title('kaiser highpass��Ƶ��Ӧ����');
xlabel('Ƶ��/Hz');ylabel('��ֵ/dB');
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
%�˳�
clc;clear;close all;
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
%�׼�������
global fs
global time
global r1
IS=0.15;                                % ǰ���޻��γ���(s)
alpha=2.8;                              % ��������
beta=0.001;                             % ���油������
%c=0ʱ,�ù����׼���������󲻽��п�������,c=1ʱ,���п�������
c=1;        
% N=length(r1);                       % �źų���
% time=(0:N-1)/fs;                        % ����ʱ��
wlen=200;                               % ����֡��
inc=80;                                 % ����֡��
NIS=fix((IS*fs-wlen)/inc +1);           % ǰ���޻���֡��
output=Mtmpsd_ssb(r1,wlen,inc,NIS,alpha,beta,c);% �ര�׸Ľ��׼������봦��
% snr2=SNR_singlech(x,output);            % �����׼���������
% snr=snr2-snr1;
% fprintf('snr1=%5.4f   snr2=%5.4f   snr=%5.4f\n',snr1,snr2,snr);
% sound(r1,fs);                     % �����������Ƚ�
% pause(1)
sound(output,fs);

%��ͼ
axes(handles.axes3);
plot(time,output,'b'); ylabel('��ֵ');
title('�׼�����������ź�','fontweight','bold');
grid;
%axis([0 4 -1.5 1.5]);

OUT=fft(output);   
axes(handles.axes4);
plot(abs(OUT),'r'); ylabel('��ֵ')
title('���������źŵ�Ƶ����','fontweight','bold');
grid;
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
%LPC�ϳ�
global ty
global n
global fs
global ts
x=ty;
% x=r1;
xl=n;
p=12;                                   % LPC�Ľ���Ϊ12
wlen=200; inc=80;                       % ֡����֡��
msoverlap = wlen - inc;                 % ÿ֡�ص����ֵĳ���
y=enframe(x,wlen,inc)';                 % ��֡
fn=size(y,2);                           % ȡ֡��
% ��������:��ÿһ֡��LPCϵ����Ԥ�����
for i=1 : fn                            
    u=y(:,i);                           % ȡ��һ֡
    A=lpc(u,p);                         % LPC���ϵ��
    aCoeff(:,i)=A;                      % �����aCoeff������
    errSig = filter(A,1,u);             % ����Ԥ���������
    resid(:,i) = errSig;                % �����resid������
end
% �����ϳ�:��ÿһ֡�ĺϳ��������ӳ����������ź�
for i=1:fn                              
    A = aCoeff(:,i);                    % ȡ�ø�֡��Ԥ��ϵ��
    residFrame = resid(:,i);            % ȡ�ø�֡��Ԥ�����
    synFrame = filter(1, A', residFrame); % Ԥ������,�ϳ�����
    
    outspeech((i-1)*inc+1:i*inc)=synFrame(1:inc);  % �ص��洢���������
% ��������һ֡,��inc������ݲ���
    if i==fn                            
        outspeech(fn*inc+1:(fn-1)*inc+wlen)=synFrame(inc+1:wlen);
    end

end;
ol=length(outspeech);
if ol<xl                                % ��outspeech����,ʹ��x�ȳ�
    outspeech=[outspeech zeros(1,xl-ol)];
end
xt=max(ol,xl);
Tss=(0:xt-1)/fs;
% ����
sound(x,fs);
% pause(1)
sound(outspeech,fs);
% ��ͼ
% subplot 211;
axes(handles.axes1);
plot(ts,x,'b');
xlabel(['ʱ��/s' 10 ]); ylabel('��ֵ');
title('ԭʼ�����ź�');grid;axis([0 4 -1 1]);
m1=fft(x);  
axes(handles.axes2);
plot(abs(m1),'r');
title('�ϳɵ������źŵ�Ƶ��','fontweight','bold');
ylabel('��ֵ');
grid;

% subplot 212;
axes(handles.axes3);
plot(Tss,outspeech,'b');
xlabel(['ʱ��/s' 10 ]); ylabel('��ֵ');
title('�ϳɵ������ź�');grid;axis([0 4 -1 1]);

m11=fft(outspeech);  
axes(handles.axes4);
plot(abs(m11),'r');
title('�ϳɵ������źŵ�Ƶ��','fontweight','bold');
ylabel('��ֵ');
grid;
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
%LPC
global y;
global fs;
%[x,fs]=wavread('speech.wav');%��������
x = y;
% Ԥ�����˲���
xx=double(x);
y=filter([1 -0.9495],1,xx);
N=160;
y1=y(1:N);
w1=hamming(N);
y2=(y1.*w1)';%�Ӵ� ȡһ֡����
p=30;%Ԥ�����
%����������غ���
r=zeros(1,p+1);
for k=1:p+1
sum=0;
for m=1:N+1-k
sum=sum+y2(m).*y2(m-1+k)';
end
r(k)=sum;
end
%����durbin�㷨������Ԥ��ϵ��
k=zeros(1,p);
k(1)=r(2)/r(1);
a=zeros(p,p);
a(1,1)=k(1);
e=zeros(1,p);
e(1)=(1-k(1)^2)*r(1);
%���ƹ���
for i=2:p
c=zeros(1,i);
sum=0;
for j=1:i-1
sum=sum+(a(i-1,j).*r(i+1-j));
end
c(i)=sum;
k(i)=(r(i+1)-c(i))/e(i-1);%����ϵ��
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
e(i)=(1-k(i)^2)*e(i-1);%Ԥ�����в�����
%subplot(414);
axes(handles.axes4);
plot(e);title('Ԥ�����в�����E(i)')
end
%���ƽ�������ȡԤ��ϵ��
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
plot(y2);title('ԭʼ����')
%subplot(412);
axes(handles.axes2);
plot(z);title('durbin�㷨������Ԥ��ϵ��')
figure(1);
subplot(411);plot(y2);title('ԭʼ����')
subplot(412);plot(z);title('durbin�㷨������Ԥ��ϵ��')
subplot(413);plot(abs(k));title('|k(i)|<=1')
subplot(414);plot(e);title('Ԥ�����в�����E(i)')


% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
jiemian
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
