clc
%filename = '12.wav'
[f,fs,bit]=wavread('1.wav');
f
fs
bit

out1=simple(filename)
out2=amdf2(filename)
out3=amdf2(filename)