function out2=simple(filename)

function out=voicefilter(v)
%Fs=22050;
Fs=8000;
[b,a] = ellip(4,0.05,80,[40 150]*2/Fs);
[H,w] = freqz(b,a,512);
sf = filter(b,a,v);
out=sf;
end

f=wavread(filename);
len=length(f);
k=0;
for i=1:len
  if abs(f(i))>k  k=abs(f(i));
  end
end
for i=1:len
    l=f(i);
    f(i)=l/k;
end
o=voicefilter(f);
len=length(o);
k=0;
for i=1:len
  if o(i)>0.35 p=k+1;k=p;
    end
end
if k>20 out2=1;
else  out2=0;
end
end