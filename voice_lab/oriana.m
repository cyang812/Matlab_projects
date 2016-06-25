[s,fs,nbits]=wavread('OriSound');       %�����źŵĲɼ�
y=s(:,1);%��ȡ����������
n = length (y) ;         %��������źŵĳ���
Noise=0.2*randn(n,1);  %���������������
s=y+Noise;             %�����źż�������
S=fft(s);             
Ft=8000;
Fp=1000;
Fs=1200;
wp=2*pi*Fp/Ft;
ws=2*pi*Fs/Ft;
[n11,wn11]=buttord(wp,ws,1,50,'s');%���ͨ�˲����Ľ����ͽ�ֹƵ��
[b11,a11]=butter(n11,wn11,'s');    %��S���Ƶ����Ӧ�Ĳ��� 
[num11,den11]=bilinear(b11,a11,0.5); %����˫���Ա任ʵ��Ƶ����ӦS��Z��ı任 
z11=filter(num11,den11,s);
sound(z11);
m11=fft(z11);  %���˲�����ź�
figure;
subplot(2,2,1);
plot(abs(S),'g');
title('�˲�ǰ�źŵ�Ƶ��','fontweight','bold');
%axis([ 0 150000 0 4000]);
grid;
subplot(2,2,2);
plot(abs(m11),'r');
title('�˲����źŵ�Ƶ��','fontweight','bold');
%axis([ 0 150000 0 4000]);
grid;
subplot(2,2,3);
plot(s);
title('�˲�ǰ�źŵĲ���','fontweight','bold');
%axis([95000 100000 -1 1]);
grid;
subplot(2,2,4);
plot(z11);
title('�˲�����źŲ���','fontweight','bold');
%axis([95000 100000 -1 1]);
grid;