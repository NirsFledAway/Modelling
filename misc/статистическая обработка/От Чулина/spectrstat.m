clear all
% �������������� ��������� ������ ������
% ����� ������ - ������ X(1:n)
% ���� ��� - ��������� ����������, �� ����� ������ ������� �������� ��� T0
% �������������� ������ �������� ����������� ������� ������ �������� �
% ���������� �����������
% ��� ��������� ���������� ������������� �������� ������
% ����� �������� "����������" ������ ������ "���������� �����"

% ������� ������ ������
n=input('����� �����?');
T0=input('��� �� �������?');
N=(0:n-1);
t=N*T0;
fprintf('���������:\n')
%MUx=input('���������� ������������ MU?');
%A1x=input('��������� A1?');
%OMEGA1x=input('������� OMEGA1?');
%FI1x=input('���� FI1?');
%A2x=input('��������� A2?');
%OMEGA2x=input('������� OMEGA2?');
%FI2x=input('���� FI2?');
%A3x=input('��������� A3?');
%OMEGA3x=input('������� OMEGA3?');
%FI3x=input('���� FI3?');
%SIGMAx=input('��� ��������� ������������?');
MUx=0
A1x=1
OMEGA1x=0.25
FI1x=0
A2x=0.5
OMEGA2x=0.5
FI2x=0
A3x=0.1
OMEGA3x=5
FI3x=0
FR1=OMEGA1x/2/pi
FR2=OMEGA2x/2/pi
FR3=OMEGA3x/2/pi
SIGMAx=0

%X=Ax*sin(2*pi*FRx*t + FIGx*pi/180) + MUx + SIGMAx*randn(1,n); 
%X=Ax*sin(OMEGAx*t + FIx) + MUx + SIGMAx*randn(1,n); 
X1=A1x*sin(OMEGA1x*t + FI1x);
X2=A2x*sin(OMEGA2x*t + FI2x);
X3=A3x*sin(OMEGA3x*t + FI3x);
X=X1 + X2 + X3 + MUx + SIGMAx*randn(1,n); 
%nOMEGA=1000
%X=MUx;
%for iOMEGA=1:nOMEGA
%    X=X+A1x*sin(OMEGA1x*iOMEGA*t + FI1x);
%end
figure;
plot(t,X)
xlabel('t')
ylabel('X')
grid

            fprintf('  ������� ������������ ��������� �������\n')
            nmYES=input('(������� y, ���� ��, ��� n, ���� ���):','s');
            if nmYES=='y'
                nTFour=n; %������ ��� ��� � ��������
                %(������������� ��������� ������� ������� ������)
                TFour=nTFour*T0; %������ ��� ��� � ���.
                FRBT = fft(X); %���
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                fFour = (0:(nTFour-1))/TFour; %������ ������
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
            input('������� ENTER');
            end
            
            %clc
            fprintf('�������������� ������\n')
            fprintf('  ���������� �������\n')
            mx = mean(X) % ���������� �������
            fprintf('  ���������� ���������\n')
            m2x = mean((X - mx).^2) % ���������� ���������
            fprintf('  � ���\n')
            s2x=sqrt(m2x) % ���
            sx = std(X) % ����������� ���������� 
            fprintf('  ���������� ����������� ������ 3 �������\n')
            m3x = mean((X - mx).^3) % ���������� ����������� ������ 3 �������
            fprintf('  ���������� ����������� ������ 4 �������\n')
            m4x = mean((X - mx).^4) % ���������� ����������� ������ 4 �������
            fprintf('  ����������\n')
            gamma1 = m3x/(m2x^1.5) % ����������
            fprintf('  ������� (��� ����.������������� 3)\n')
            gamma2 = m4x/(m2x^2)  % �������
            fprintf('  ������������ (��� ����.������������� ~0.577)\n')
            gamma05 = m2x/sqrt(m4x)  % ������������
            input('������� ENTER');
            
            %�����������
            kHIST1=3.31*log10(n)+1
            kHIST2=5*log10(n)
            kHIST=round((kHIST1+kHIST2)/2)
            [NHIST, yHIST] = hist(X,kHIST);
            NHISTnWYB=NHIST/n; %������������� �������
            hHIST=(yHIST(kHIST)-yHIST(1))/kHIST; %����� ������� �����������
            hHIST1=yHIST(2)-yHIST(1); %����� ������� �����������
            hHISTn=(max(X)-min(X))/kHIST; %����� ������� �����������
            %������������� �����������
            fxH = normpdf(yHIST,mx,sx)*hHIST;
            %��-�������
            hi2=n*sum((NHISTnWYB-fxH).^2/fxH)
            
            bar(yHIST, NHIST)
            %�������� "3-�����" � ����� 0.1*CKO
            ksi = (-3.0:0.1:3.0)*sx + mx;
            %������������� �����������
            fx = normpdf(ksi,mx,sx)*hHIST;
            figure;
            plot(ksi,fx, yHIST, NHISTnWYB)
            input('������� ENTER');
            
