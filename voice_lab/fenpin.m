a=wavread('lh_11');
len=length(a);
b=a(1:22050*6);
wavplay(b,22050);
plot(b);
wavwrite(b,22050,'lh_111.wav');
    