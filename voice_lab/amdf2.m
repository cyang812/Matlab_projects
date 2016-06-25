function out=amdf2(filename)
%短时能量函数
    function out=the(v)
      n=floor((length(v)-1)/900);
        for i=0:(n-1)
            p=0;
            for j=1:900
              ss=p+v(i*900+j)*v(i*900+j);
              p=ss;
            end
            sf(i+1)=p;
        end
        out=sf;        
    end
    function out=voicefilter(v)
        Fs=22050;
        [b,a] = ellip(4,0.05,80,[60 900]*2/Fs);
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
%;AMD法
        len=length(o);
        nn=the(o);
        energy=0;
        for i=1:length(nn)
                re=energy;
                energy=re+nn(i);
        end
        average=energy/length(nn);
        woman=0;
        man=0;
        for i=1:length(nn)-1
                if (nn(i)>average)&&(nn(i+1)>average)
                    p=0;
                    for k=1:900
                        for j=1:900
                            c=p+abs(o(i*900-900+j)-o(k+i*900+j-900));
                            p=c;
                        end
                        r(k)=p;
                        p=0;
                    end
                    rr=1000;
                    
     %               plot(r);
                    
                    for j=45:length(r)
                        if r(j)<rr pl=j;rr=r(j);
                        end
                    end
                    m=man;
                    w=woman;
                    if pl>110 man=m+1;
                    else woman=w+1;
                    end
                end
        end 
        num(1)=man;
        num(2)=woman;
        
%        plot(num);
        if man>woman out=1;
        else out=0;
        end
end