
%��Ů��ʶ�����1
%����Ϊͨ������Ƶ�ʵĹ��ƣ��ж���Ů��
%������50��200Hz  Ů����200��450
%���ڳ���Ϊ900

function out2=amdf1(filename)

%Ѱ��������,�ڸ����������źŶ���Ѱ�ҵ����ʵ���������
%��ʱ��������
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
%;�˲�����
    function out=voicefilter(v)
    Fs=22050;
    [b,a] = ellip(4,0.05,80,[60 900]*2/Fs);
    [H,w] = freqz(b,a,512);
    sf = filter(b,a,v);
    out=sf;
    end
%;��������
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
%;����ƽ����
    function  out=three(v)
        for i=1:length(v)
            sf(i)=sign(v(i));
        end
        out=sf;
    end

% ;������
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