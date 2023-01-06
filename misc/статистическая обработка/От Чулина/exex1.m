clear all
while 1
    clc
    fprintf('������� ����\n')
    fprintf('____________\n')
    fprintf('1. ����� ����� ������ ������������\n')
    fprintf('2. ����� ��������� ��� ���������\n')
    fprintf('3. ����� ���������� ��� ��������� (� ��� �)\n')
    fprintf('4. ��������� ������������� ������������\n')
    fprintf('5. ��������� ��������� ������\n')
    fprintf('6. �������������� ������\n')
    fprintf('7. ����� ���������\n')
    nmen=input('������� ����� ������ (enter - �����):');
    if length(nmen)==0,
        return
    end
    switch nmen
        case 1
            [filename, pathname] = uigetfile('*.txt','�������� ���� ������');
            if filename~=0
                fid=fopen([pathname, filename]);
                i = 1;
                j = 1;
                while ~feof(fid)
                    s = fgetl(fid);
                    A = sscanf(s,'%d');
                    if length(A)==7
                        N(i) = A(1);   
                        Xd(i) = A(2);   
                        Yd(i) = A(3);   
                        X(i) = A(4);   
                        Y(i) = A(5);   
                        X0(i) = A(6);   
                        Y0(i) = A(7);  
                        i = i+1;
                    else
                        token = strtok(s,';');
                        if(j==3)|(j==7)|(j==8)
                            B(j) = sscanf(token,'%f');
                        else
                            B(j) = sscanf(token,'%d');
                        end
                        j = j + 1;
                    end
                end
                nfakt=i-1;
                fclose(fid);
                
                %������� �� ����� ���������� �������� n
                n=B(9);
                %n=3200;
                %������ ����� �������� �� ����� n � �� ������ nfakt
                if n > nfakt
        fprintf('����� �������� �� ����� %d ������ ����� ��������� �������� %d\n', n, nfakt)
                    n=nfakt;
        fprintf('������� ����� ��������� �������� %d\n', nfakt)
                    input('������� ENTER');
                end
                %������� �� ����� ������������ ������������ tex
                tex=B(8);
                %tex=60;
                T0=tex/n; %��� �����������
                %������� ������� fG � ��
                fG=B(7)
                %��������� ������ ����.����.� �������� TG
                TG=1/fG;
                %� �������� 
                nnTG=TG/T0;
                nTG=round(nnTG); %����� � ��������!
                
                clc
                fprintf('������� �������� ������ ������ ��� ��������� (�� ��������� - ������)?\n')
                nmGRODi=input('���� ��, ������� 1: ');
                if nmGRODi==1
                    nmGROD=1;
                else
                    nmGROD=2;
                end
    
                %����������� ���������� "�� ���������"
                nbeg=1;
                nend=n;
                nWYB=nend-nbeg+1;
                NWYB=N(nbeg:nend);
                if nmGROD==1
                    XWYB=Xd(nbeg:nend); 
                    YWYB=Yd(nbeg:nend); 
                else
                    XWYB=X(nbeg:nend); 
                    YWYB=Y(nbeg:nend); 
                end
                
                close all
                subplot(2,1,1)
                plot(N,XWYB)
                ylabel('X')
                grid
                subplot(2,1,2)
                plot(N,YWYB)
                xlabel('N')
                ylabel('Y')
                grid
               
                %��� ���������
                AGX=0;
                FIGX = 0;
                FIGGX = 0;
                AGY=0;
                FIGY = 0;
                FIGGY = 0;
                mxX=0;
                m2xX=0;
                sxX=0;
                gamma1X=0;
                gamma2X=0;
                gamma05X=0;
                mxY=0;
                m2xY=0;
                sxY=0;
                gamma1Y=0;
                gamma2Y=0;
                gamma05Y=0;
                kHISTX=1;
                NHISTX=zeros(1,50);
                yHISTX=zeros(1,50);
                kHISTY=1;
                NHISTY=zeros(1,50);
                yHISTY=zeros(1,50);
                hi2X=0;
                hi2Y=0;
            end
        case 2
            clc
            fprintf('  ����� ��������� ��� ���������\n')
            nbeg=input('������ �������:');
            nend=input('����� �������:');
            nWYB=nend-nbeg+1;
            NWYB=N(nbeg:nend);   
            if nmGROD==1
                XWYB=Xd(nbeg:nend); 
                YWYB=Yd(nbeg:nend); 
            else
                XWYB=X(nbeg:nend); 
                YWYB=Y(nbeg:nend); 
            end

            figure;
            subplot(2,1,1)
            plot(NWYB,XWYB)
            ylabel('X')
            grid
            subplot(2,1,2)
            plot(NWYB,YWYB)
            xlabel('N')
            ylabel('Y')
            grid
        case 3
            clc
            fprintf('�������� ���������� ��� ���������\n')
            nmXY=input('(������� x, ��� y): ','s');
            if nmXY=='x'
                RBT=XWYB;
            else 
                RBT=YWYB;
            end
            RBTG=RBT;
            RBTGL=RBT; 
            figure;
            plot(NWYB,RBT)
            xlabel('N')
            if nmXY=='x'
                ylabel('X')
            else 
                ylabel('Y')
            end 
            grid
            fprintf('  ������� ������������ ��������� ���������� �������\n')
            nmYES=input('(������� y, ���� ��, ��� n, ���� ���):','s');
            if nmYES=='y'
                nTFour=nWYB; %������ ��� ��� � ��������
                %(������������� ��������� ������� ������� ������)
                TFour=nTFour*T0; %������ ��� ��� � ���.
                FRBT = fft(RBT); %���
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                %������������� 
                    %FRBT = fft(RBT,n2TFour); 
                    %n2TFour - ��������� ������� � nTFour ������� ������
                    %PRBT = FRBT.* conj(FRBT) / n2TFour;
                    %T2Four=n2TFour*T0;
                    %fFour = (0:nTFour)/T2Four; %������ ������
                fFour = (0:(nTFour-1))/TFour; %������ ������
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
            [mPRBT,ImPRBT] = max(PRBT(5:(fix(nTFour/2)-1)));
            fFourGR=fFour(ImPRBT+4)
            input('������� ENTER');
            end
 
        case 4
            clc
            fprintf('  ��������� ������������� ������������\n')
            
            fprintf('��������� ������� �� �������� �������?\n')
            nmYES=input('(������� y, ���� ��, ��� n, ���� ���):','s');
            if nmYES=='y'
                %�������� ������� �� �������� �������
                nTFour=n; %������ ��� ��� � ��������
                %(������������� ��������� ������� ������� ������)
                TFour=nTFour*T0; %������ ��� ��� � ���.
                nmX0Y0=input('(�� x, ��� y)? ','s');
                if nmX0Y0=='x'
                    FRBT = fft(X0); %���
                else 
                    FRBT = fft(Y0); %���
                end
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                fFour = (0:(nTFour-1))/TFour; %������ ������
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
                fprintf('  �������� ������������ ��������� �������� �������\n')
                [mPRBT,ImPRBT] = max(PRBT(1:(fix(nTFour/2)-1)));
                fFourG=fFour(ImPRBT)
                input('������� ENTER');
            end
            
            fprintf('������������� ������������\n')
            nPTG=fix(nWYB/nTG); %����� ��������
            nWYBG=fix(nPTG*nTG); %����� ��������, ������� ����� ��������
            RBG=RBT(1:nWYBG);
            NWYBG=NWYB(1:nWYBG);
            % ������������ ����� 1 ������� 
            fprintf('������������ ����� 1 �������\n')
            a1f = 2*mean(RBG.*cos(2*pi*fG*(NWYBG)*T0))
            b1f = 2*mean(RBG.*sin(2*pi*fG*(NWYBG)*T0))
            % ��������� � ����
            fprintf('��������� � ����\n')
            AG = sqrt(a1f^2 + b1f^2)
            FIG = atan2(a1f,b1f)
            FIGG = FIG*180/pi
            %��� ���������
            if nmXY=='x'
                AGX=AG;
                FIGX = FIG;
                FIGGX = FIGG;
            else 
                AGY=AG;
                FIGY = FIG;
                FIGGY = FIGG;
            end
            tG=(NWYB)*T0;
            GS=AG*sin(2*pi*fG*tG + FIG);
            figure;
            plot(NWYB,RBT,NWYB,GS)
            grid
            fprintf('  �������� ������������� ������������ ���������� �������\n')
            input('������� ENTER');
            RBTG=RBT-GS;
            figure;
            plot(NWYB,RBTG)
            grid
            nTFour=nWYB; %������ ��� ��� � ��������
            %(������������� ��������� ������� ������� ������)
            TFour=nTFour*T0; %������ ��� ��� � ���.
            FRBT = fft(RBTG); %���
            PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
            fFour = (0:(nTFour-1))/TFour; %������ ������
            figure;
            plot(fFour(1:nTFour),PRBT(1:nTFour))
            title('Power spectrum')
            xlabel('frequency (Hz)')
            fprintf('  ������� ������ ��� ������������� ������������\n')
            fprintf('� ��� ������������ ��������� \n')
            [mPRBT,ImPRBT] = max(PRBT(5:(fix(nTFour/2)-1)));
            fFourGo=fFour(ImPRBT+4)
            input('������� ENTER');
            clc
            
        case 51
            nbeg=1;
            nend=n;
            nDR=nend-nbeg+1;
            NDR=N(nbeg:nend);
            nWIND=fix(nDR/100); %������ ����
            %��������� �� >1
            nzWIND=fix(nWIND/2); %"������������"
            clc
            fprintf('���������� ������\n')
                figure;
            %key51=0;
            while 1
                %key51==0
                k51=menu('���������� ������','��������� ����','��������� ����','�������� �����');
                switch k51;
                case 1
                    if nWIND<nDR/2 
                        nzWIND=nWIND;
                        nWIND=nWIND*2;
                    else
                        input('���� �� ������ (������� ENTER)');
                    end
                case 2
                    if nWIND>=4 
                        nWIND=nWIND/2;
                        nzWIND=fix(nWIND/2);
                    else
                        input('���� �� ������ (������� ENTER)');
                    end
                case 3 
                    %key51=1; 
                    break
                end
                Zd=Yd;
                %���������� "���������� �����" ��������� ������ nWIND
                %Zd=Xd ��� Yd, ZDR - ��������� ����������, ZbDR=Zd-ZDR
                mWIND=0;
                for iDR=1:nzWIND
                    mWIND=(mWIND*(iDR-1)+Zd(iDR))/iDR;
                end
                for iDR=nzWIND+1:nDR+nzWIND
                    if iDR<=nWIND
                        %"�����������" �������
                        mWIND=(mWIND*(iDR-1)+Zd(iDR))/iDR;
                    elseif iDR>nDR
                        %"���������" �������
                        mWIND=(mWIND*(nWIND-(iDR-nDR)+1)-Zd(iDR-nWIND))/(nWIND-(iDR-nDR));
                    else
                        %��������� �������
                        mWIND=mWIND+(Zd(iDR)-Zd(iDR-nWIND))/nWIND;
                    end
                    ZDR(iDR-nzWIND)=mWIND;
                end
                ZbDR=Zd-ZDR;
                plot(N,Zd,N,ZDR,N,ZbDR)
                grid
            end 
            
        case 5
            clc
            fprintf('  ��������� ��������� ������\n')
            RBTGL = detrend(RBTG);
            LT=RBTG-RBTGL;
            figure;
            plot(NWYB,RBTG,NWYB,LT)
            grid
            fprintf('  ������� �������� ����� ���������� �������\n')
            input('������� ENTER');
            figure;
            plot(NWYB,RBTGL)
            grid
            nTFour=nWYB; %������ ��� ��� � ��������
            %(������������� ��������� ������� ������� ������)
            TFour=nTFour*T0; %������ ��� ��� � ���.
            FRBT = fft(RBTGL); %���
            PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
            fFour = (0:(nTFour-1))/TFour; %������ ������
            figure;
            plot(fFour(1:nTFour),PRBT(1:nTFour))
            title('Power spectrum')
            xlabel('frequency (Hz)')
            fprintf('  ������� ������ ��� ��������� ������\n')
            fprintf('� ��� ������������ ��������� \n')
            [mPRBT,ImPRBT] = max(PRBT(1:(fix(nTFour/2)-1)));
            fFourGoo=fFour(ImPRBT)
            input('������� ENTER');

        case 6
            clc
            fprintf('�������������� ������\n')
            fprintf('  ���������� �������\n')
            mx = mean(RBTGL) % ���������� �������
            fprintf('  ���������� ���������\n')
            m2x = mean((RBTGL - mx).^2) % ���������� ���������
            fprintf('  � ���\n')
            s2x=sqrt(m2x) % ���
            sx = std(RBTGL) % ����������� ���������� 
            fprintf('  ���������� ����������� ������ 3 �������\n')
            m3x = mean((RBTGL - mx).^3) % ���������� ����������� ������ 3 �������
            fprintf('  ���������� ����������� ������ 4 �������\n')
            m4x = mean((RBTGL - mx).^4) % ���������� ����������� ������ 4 �������
            fprintf('  ����������\n')
            gamma1 = m3x/(m2x^1.5) % ����������
            fprintf('  ������� (��� ����.������������� 3)\n')
            gamma2 = m4x/(m2x^2)  % �������
            fprintf('  ������������ (��� ����.������������� ~0.577)\n')
            gamma05 = m2x/sqrt(m4x)  % ������������
            input('������� ENTER');
            %��� ���������
            if nmXY=='x'
                mxX=mx;
                m2xX=m2x;
                sxX=sx;
                gamma1X=gamma1;
                gamma2X=gamma2;
                gamma05X=gamma05;
            else 
                mxY=mx;
                m2xY=m2x;
                sxY=sx;
                gamma1Y=gamma1;
                gamma2Y=gamma2;
                gamma05Y=gamma05;
            end
            
            %�����������
            kHIST1=3.31*log10(nWYB)+1
            kHIST2=5*log10(nWYB)
            kHIST=round((kHIST1+kHIST2)/2)
            %fprintf('����� �������� ����������� %d\n', kHIST)
            %nmYES=input('������ ��������(������� y, ���� ��, ��� n, ���� ���):','s');
            %if nmYES=='y'
                %fprintf('������������� �������� %d - %d\n', round(kHIST1), round(kHIST2))
                %kHIST=input('����� �������� �����������?');
            %end
            [NHIST, yHIST] = hist(RBTGL,kHIST);
            NHISTnWYB=NHIST/nWYB; %������������� �������
            hHIST=(yHIST(kHIST)-yHIST(1))/kHIST; %����� ������� �����������
            hHIST1=yHIST(2)-yHIST(1); %����� ������� �����������
            hHISTn=(max(RBTGL)-min(RBTGL))/kHIST; %����� ������� �����������
            %������������� �����������
            fxH = normpdf(yHIST,mx,sx)*hHIST;
            %��-�������
            hi2=nWYB*sum((NHISTnWYB-fxH).^2/fxH)
            
            bar(yHIST, NHIST)
            %�������� "3-�����" � ����� 0.1*CKO
            ksi = (-3.0:0.1:3.0)*sx + mx;
            %������������� �����������
            fx = normpdf(ksi,mx,sx)*hHIST;
            figure;
            plot(ksi,fx, yHIST, NHISTnWYB)
            
            %���� �����������-��������
            %xKSTST = (RBTGL - mx)/sx;
            %HKSTST = kstest(xKSTST);
            
            %��� ���������
            if nmXY=='x'
                kHISTX=kHIST;
                NHISTX(1:kHIST)=NHIST(1:kHIST);
                yHISTX(1:kHIST)=yHIST(1:kHIST);
                hi2X=hi2;
            else 
                kHISTY=kHIST;
                NHISTY(1:kHIST)=NHIST(1:kHIST);
                yHISTY(1:kHIST)=yHIST(1:kHIST);
                hi2Y=hi2;
            end
            
            input('������� ENTER');

        case 7
            %��������
            %�������� � �� (1 �� = 15 ��������)
            %��������� ������������� ������������ (� ��) 
            AGXmm=AGX/15;   
            AGYmm=AGY/15;
            %���������� ������� (� ��)
            mxXmm=mxX/15;   
            mxYmm=mxY/15;
            %���������� ���������                        
            m2xXmm=m2xX/15/15;  
            m2xYmm=m2xY/15/15;
            %��� (c���������� ����������) (� ��)         
            sxXmm=sxX/15;   
            sxYmm=sxY/15;
            
            yHISTXmm=yHISTX/15; 
            yHISTYmm=yHISTY/15;
            %������������ �������� (� ��)                
            DELMXX=AGXmm+mxXmm+3*sxXmm;
            DELMXY=AGYmm+mxYmm+3*sxYmm;
            %������� ��������
            BETATMX=2*pi*B(6)*B(7);
            BETATMD=0.636*BETATMX;
            
            clc
            fprintf('   ��������� ������������\n')
            fprintf('����  %d\n', B(1))
            fprintf('��� ����� (1-��,2-��)  %d\n', B(2))
            fprintf('����� ��� (� ��) %f\n', B(3))
            fprintf('����� ���������  %d\n', B(4))
            fprintf('��� ����������  %d\n', B(5))
            fprintf('��������� ����������� (� ��������)  %d\n', B(6))
            fprintf('������� �������������� ����������� (� ��) %f\n', B(7))
            fprintf('������� �������� ������������ (� ��������/�)  %f\n', BETATMX)
            fprintf('������� �������� ������� (� ��������/�)  %f\n', BETATMD)
            fprintf('������������ ������������ (� ���.)  %f\n', B(8))
            fprintf('����� �������  %d\n', B(9))
            
            fprintf('   ���������� ���������\n')
            fprintf('�������� ���������: ��  %d  ��  %d\n', nbeg, nend)

            fprintf('                                      �� ����������� (�)  �� ��������� (�)\n')
            fprintf('������������ �������� (� ��)                %f         %f\n', DELMXX, DELMXY)
            fprintf('��������� ������������� ������������ (� ��) %f         %f\n', AGXmm, AGYmm)
            fprintf('���� ������������� ������������ (� ���.)    %f         %f\n', FIGX, FIGY)
            fprintf('                                (� ����.)   %f         %f\n', FIGGX, FIGGY)
            fprintf('���������� ������� (� ��)                   %f         %f\n', mxXmm, mxYmm)
            fprintf('���������� ���������                        %f         %f\n', m2xXmm, m2xYmm)
            fprintf('��� (c���������� ����������) (� ��)         %f         %f\n', sxXmm, sxYmm)
            fprintf('����������                                  %f         %f\n', gamma1X, gamma1Y)
            fprintf('�������                                     %f         %f\n', gamma2X, gamma2Y)
            fprintf('������������                                %f         %f\n', gamma05X, gamma05Y)
            fprintf('"��-�������"                                %f         %f\n', hi2X, hi2Y)
          
            fprintf('   ��������� ����������\n')
            fprintf(' N ���.        X          P(X)          Y           P(Y)\n')
            kHIpr=kHISTX;
            if kHISTY > kHIpr, kHIpr=kHISTY, end;
            for ipr=1:kHIpr
fprintf('%d         %f      %f      %f     %f\n',ipr,yHISTXmm(ipr),NHISTX(ipr),yHISTYmm(ipr),NHISTY(ipr))
            end
            fprintf('��������� ����������?\n')
            nmYES=input('(������� y, ���� ��, ��� ����� ������ �������, ���� ���):','s');
            if nmYES=='y'
                indp=strfind(lower(filename),'.');
                if (~isempty(indp))
                    indp=indp(length(indp));
                    filenamepr=[filename(1:(indp-1)), '_pr.txt'];
                    filenamere=[filename(1:(indp-1)), '_1.res'];
                end
                fpr=fopen([pathname, filenamepr],'w');
                
fprintf(fpr,'   ��������� ������������\n');
fprintf(fpr,'����  %d\n', B(1));
fprintf(fpr,'��� ����� (1-��,2-��)  %d\n', B(2));
fprintf(fpr,'����� ��� (� ��) %f\n', B(3));
fprintf(fpr,'����� ���������  %d\n', B(4));
fprintf(fpr,'��� ����������  %d\n', B(5));
fprintf(fpr,'��������� ����������� (� ��������) %d\n', B(6));
fprintf(fpr,'������� �������������� ����������� (� ��) %f\n', B(7));
fprintf(fpr,'������� �������� ������������ (� ��������/�)  %f\n', BETATMX);
fprintf(fpr,'������� �������� ������� (� ��������/�)  %f\n', BETATMD);
fprintf(fpr,'������������ ������������ (� ���.)  %f\n', B(8));
fprintf(fpr,'����� �������  %d\n', B(9));
fprintf(fpr,'   ���������� ���������\n');
fprintf(fpr,'�������� ���������: ��  %d  ��  %d\n', nbeg, nend);
fprintf(fpr,'                                      �� ����������� (�)  �� ��������� (�)\n');
fprintf(fpr,'������������ �������� (� ��)                %f         %f\n', DELMXX, DELMXY);
fprintf(fpr,'��������� ������������� ������������ (� ��) %f         %f\n', AGXmm, AGYmm);
fprintf(fpr,'���� ������������� ������������ (� ���.)    %f         %f (� ���.)\n', FIGX, FIGY);
fprintf(fpr,'                                (� ����.)   %f         %f (� ����.)\n', FIGGX, FIGGY);
fprintf(fpr,'���������� ������� (� ��)                   %f         %f\n', mxXmm, mxYmm);
fprintf(fpr,'���������� ���������                        %f         %f\n', m2xXmm, m2xYmm);
fprintf(fpr,'��� (c���������� ����������) (� ��)         %f         %f\n', sxXmm, sxYmm);
fprintf(fpr,'����������                                  %f         %f\n', gamma1X, gamma1Y);
fprintf(fpr,'�������                                     %f         %f\n', gamma2X, gamma2Y);
fprintf(fpr,'������������                                %f         %f\n', gamma05X, gamma05Y);
fprintf(fpr,'"��-�������"                                %f         %f\n', hi2X, hi2Y);
fprintf(fpr,'   ��������� ����������\n');
fprintf(fpr,' N ���.        X          P(X)          Y           P(Y)\n');
                for ipr=1:kHIpr
fprintf(fpr,'%d         %f     %f      %f    %f\n',ipr,yHISTX(ipr),NHISTX(ipr),yHISTY(ipr),NHISTY(ipr));
                end
                fclose(fpr);
                fprintf('�������� �������� � ����� %s\n', filenamepr)
                
                fre=fopen([pathname, filenamere],'w');
fprintf(fre,'%d\n', B(1));
fprintf(fre,'%d\n', B(2));
fprintf(fre,'%f\n', B(3));
fprintf(fre,'%d\n', B(4));
fprintf(fre,'%d\n', B(5));
fprintf(fre,'%d\n', B(6));
fprintf(fre,'%f\n', B(7));
fprintf(fre,'%f\n', B(8));
fprintf(fre,'%d\n', B(9));
fprintf(fre,'%d\n', nbeg);
fprintf(fre,'%d\n', nend);
fprintf(fre,'%f\n', AGX);
fprintf(fre,'%f\n', FIGX);
fprintf(fre,'%f\n', FIGGX);
fprintf(fre,'%f\n', mxX);
fprintf(fre,'%f\n', m2xX);
fprintf(fre,'%f\n', sxX);
fprintf(fre,'%f\n', gamma1X);
fprintf(fre,'%f\n', gamma2X);
fprintf(fre,'%f\n', gamma05X);
fprintf(fre,'%f\n', AGY);
fprintf(fre,'%f\n', FIGY);
fprintf(fre,'%f\n', FIGGY);
fprintf(fre,'%f\n', mxY);
fprintf(fre,'%f\n', m2xY);
fprintf(fre,'%f\n', sxY);
fprintf(fre,'%f\n', gamma1Y);
fprintf(fre,'%f\n', gamma2Y);
fprintf(fre,'%f\n', gamma05Y);
fprintf(fre,'%f\n', hi2X);
fprintf(fre,'%f\n', hi2Y);
fprintf(fre,'%f\n', nmGROD);

                fclose(fre);
                fprintf('���������� ��� ���������� ��������� ��������� � ����� %s\n', filenamere)
                input('������� ENTER');
            end
            
        case 12
            figure;
            plot(XWYB,YWYB,'.')
            xlabel('X')
            ylabel('Y')
            grid
            
        case 99
            %�������� ������ ������ 2 �������� ����
            clc
            fprintf('����\n')
            SIGMAxTST=input('��� ��� ����� �� �?');
            SIGMAyTST=input('��� ��� ����� �� �?');
            MUxTST=0;
            MUyTST=0;
            AxTST=input('��������� ��� ����� �� �?');
            AyTST=input('��������� ��� ����� �� �?');
            FIGxTST=input('���� ��� ����� �� �?');
            FIGyTST=input('���� ��� ����� �� �?');
            tGTST=N*T0;
            nbeg=1;
            nend=n;
            nWYB=nend-nbeg+1;
            NWYB=N(nbeg:nend);
            %XWYB=AxTST*sin(2*pi*fG*tGTST + FIGxTST)+MUxTST + SIGMAxTST*randn(1,n); 
            %YWYB=AyTST*cos(2*pi*fG*tGTST + FIGyTST)+MUyTST + SIGMAyTST*randn(1,n);   
            XWYB=X0+MUxTST + SIGMAxTST*randn(1,n); 
            YWYB=Y0+MUyTST + SIGMAyTST*randn(1,n);
        case 98
            %�������� ����� ������ 1 �������� ����
            clc
            fprintf('����fGT0\n')
            AxTST=input('��������� ��� ����� �� �?');
            AyTST=input('��������� ��� ����� �� �?');
            FIGGxTST=input('���� ��� ����� �� �?');
            FIGGyTST=input('���� ��� ����� �� �?');
            T0T0=T0;
            delT0T0=T0T0/1000;
                figure;
            while 1
                tGTST=N*T0T0;
                X=AxTST*sin((2*fG*tGTST + FIGGxTST/180)*pi); 
                Y=AyTST*sin((2*fG*tGTST + FIGGxTST/180)*pi);   
                plot(N,Y0,N,Y)
                grid
                k=menu('�������� ���','���������','���������','�� ������','��������');
                switch k;
                case 1
                    T0T0=T0T0+delT0T0;
                case 2
                    T0T0=T0T0-delT0T0;
                case 3 
                    T0=T0T0;
                    %������ ����.����.� �������� 
                    nnTG=TG/T0;
                    nTG=round(nnTG); 
                    break
                case 4 
                    delT0T0=delT0T0/5;
                end
            end 
    end
end