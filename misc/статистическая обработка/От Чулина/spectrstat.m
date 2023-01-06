clear all
% Статистическая обработка набора данных
% Набор данных - массив X(1:n)
% Если это - временная реализация, то кроме самого массива задается шаг T0
% Статистический анализ включает определение четырех первых моментов и
% построение гистограммы
% Для временных реализаций дополнительно строится спектр
% Можно провести "осреднение" набора данных "скользящим окном"

% Задание набора данных
n=input('Число точек?');
T0=input('Шаг по времени?');
N=(0:n-1);
t=N*T0;
fprintf('Параметры:\n')
%MUx=input('Постоянная составляющая MU?');
%A1x=input('Амплитуда A1?');
%OMEGA1x=input('Частота OMEGA1?');
%FI1x=input('Фаза FI1?');
%A2x=input('Амплитуда A2?');
%OMEGA2x=input('Частота OMEGA2?');
%FI2x=input('Фаза FI2?');
%A3x=input('Амплитуда A3?');
%OMEGA3x=input('Частота OMEGA3?');
%FI3x=input('Фаза FI3?');
%SIGMAx=input('СКО случайной составляющей?');
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

            fprintf('  Вывести спектральную плотность сигнала\n')
            nmYES=input('(введите y, если ДА, или n, если НЕТ):','s');
            if nmYES=='y'
                nTFour=n; %Период для ДПФ в отсчетах
                %(Рекомендуется ближайшая меньшая степень двойки)
                TFour=nTFour*T0; %Период для ДПФ в сек.
                FRBT = fft(X); %ДПФ
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                fFour = (0:(nTFour-1))/TFour; %Массив частот
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
            input('Нажмите ENTER');
            end
            
            %clc
            fprintf('Статистический анализ\n')
            fprintf('  Выборочное среднее\n')
            mx = mean(X) % Выборочное среднее
            fprintf('  Выборочная дисперсия\n')
            m2x = mean((X - mx).^2) % Выборочная дисперсия
            fprintf('  и СКО\n')
            s2x=sqrt(m2x) % СКО
            sx = std(X) % Стандартное отклонение 
            fprintf('  Выборочный центральный момент 3 порядка\n')
            m3x = mean((X - mx).^3) % Выборочный центральный момент 3 порядка
            fprintf('  Выборочный центральный момент 4 порядка\n')
            m4x = mean((X - mx).^4) % Выборочный центральный момент 4 порядка
            fprintf('  Асимметрия\n')
            gamma1 = m3x/(m2x^1.5) % Асимметрия
            fprintf('  Эксцесс (для норм.распределения 3)\n')
            gamma2 = m4x/(m2x^2)  % Эксцесс
            fprintf('  Контрэксцесс (для норм.распределения ~0.577)\n')
            gamma05 = m2x/sqrt(m4x)  % Контрэксцесс
            input('Нажмите ENTER');
            
            %Гистограмма
            kHIST1=3.31*log10(n)+1
            kHIST2=5*log10(n)
            kHIST=round((kHIST1+kHIST2)/2)
            [NHIST, yHIST] = hist(X,kHIST);
            NHISTnWYB=NHIST/n; %Относительные частоты
            hHIST=(yHIST(kHIST)-yHIST(1))/kHIST; %Длина разряда гистограммы
            hHIST1=yHIST(2)-yHIST(1); %Длина разряда гистограммы
            hHISTn=(max(X)-min(X))/kHIST; %Длина разряда гистограммы
            %Теоретические вероятности
            fxH = normpdf(yHIST,mx,sx)*hHIST;
            %Хи-квадрат
            hi2=n*sum((NHISTnWYB-fxH).^2/fxH)
            
            bar(yHIST, NHIST)
            %Диапазон "3-сигма" с шагом 0.1*CKO
            ksi = (-3.0:0.1:3.0)*sx + mx;
            %Теоретические вероятности
            fx = normpdf(ksi,mx,sx)*hHIST;
            figure;
            plot(ksi,fx, yHIST, NHISTnWYB)
            input('Нажмите ENTER');
            
