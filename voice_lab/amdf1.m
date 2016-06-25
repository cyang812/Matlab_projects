
%男女声识别程序1
%方法为通过基音频率的估计，判断男女声
%男声：50～200Hz  女声：200～450
%窗口长度为900

function out2=amdf1(filename)

%寻找浊音段,在给定的语音信号段内寻找到合适的浊音部分
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
%;滤波函数
    function out=voicefilter(v)
    Fs=22050;
    [b,a] = ellip(4,0.05,80,[60 900]*2/Fs);
    [H,w] = freqz(b,a,512);
    sf = filter(b,a,v);
    out=sf;
    end
%;中心削波
    function  out=center(v)
        len=length(v);
        k=0;
        for i=1:len
             if abs(v(i))>k  k=abs(v(i));
             end
        end
        p=0.5*k;
        for i=1:len
            if v(i)>p  sf(i)=v(i)-p;
            else if v(i)<-p  sf(i)=v(i)+p;
                else  sf(i)=0;
                end
            end
        end
        out=sf;
    end
%;三电平削波
    function  out=three(v)
        for i=1:length(v)
            sf(i)=sign(v(i));
        end
        out=sf;
    end

% ;主程序
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
                if nn(i)>average
                    x=o((i-1)*900+1:i*900);
                    y=center(x);
                    y1=three(y);       
                    p=0;
                    for k=1:length(y)
                        for i=1:length(y)-k
                            c=p+y(i)*y1(k+i);
                            p=c;
                        end
                        r(k)=p;
                        p=0;
                    end
                    rr=0;
    %                plot(r);
                    
                    
                    pl=0;
                    for j=45:length(r)
                        if r(j)>rr pl=j;rr=r(j);
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
%            plot(num);
        if man>woman out2=1; 
        else out2=0;    
        end

end