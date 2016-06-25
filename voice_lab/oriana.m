[s,fs,nbits]=wavread('OriSound');       %语音信号的采集
y=s(:,1);%截取单声道数据
n = length (y) ;         %求出语音信号的长度
Noise=0.2*randn(n,1);  %随机函数产生噪声
s=y+Noise;             %语音信号加入噪声
S=fft(s);             
Ft=8000;
Fp=1000;
Fs=1200;
wp=2*pi*Fp/Ft;
ws=2*pi*Fs/Ft;
[n11,wn11]=buttord(wp,ws,1,50,'s');%求低通滤波器的阶数和截止频率
[b11,a11]=butter(n11,wn11,'s');    %求S域的频率响应的参数 
[num11,den11]=bilinear(b11,a11,0.5); %利用双线性变换实现频率响应S域到Z域的变换 
z11=filter(num11,den11,s);
sound(z11);
m11=fft(z11);  %求滤波后的信号
figure;
subplot(2,2,1);
plot(abs(S),'g');
title('滤波前信号的频谱','fontweight','bold');
%axis([ 0 150000 0 4000]);
grid;
subplot(2,2,2);
plot(abs(m11),'r');
title('滤波后信号的频谱','fontweight','bold');
%axis([ 0 150000 0 4000]);
grid;
subplot(2,2,3);
plot(s);
title('滤波前信号的波形','fontweight','bold');
%axis([95000 100000 -1 1]);
grid;
subplot(2,2,4);
plot(z11);
title('滤波后的信号波形','fontweight','bold');
%axis([95000 100000 -1 1]);
grid;