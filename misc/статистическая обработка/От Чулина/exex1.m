clear all
while 1
    clc
    fprintf('Главное меню\n')
    fprintf('____________\n')
    fprintf('1. Выбор файла данных эксперимента\n')
    fprintf('2. Выбор диапазона для обработки\n')
    fprintf('3. Выбор координаты для обработки (Х или У)\n')
    fprintf('4. Отделение гармонической составляющей\n')
    fprintf('5. Отделение линейного тренда\n')
    fprintf('6. Статистический анализ\n')
    fprintf('7. Вывод протокола\n')
    nmen=input('Введите номер пункта (enter - выход):');
    if length(nmen)==0,
        return
    end
    switch nmen
        case 1
            [filename, pathname] = uigetfile('*.txt','Выберите файл данных');
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
                
                %Считать из шапки количество отсчетов n
                n=B(9);
                %n=3200;
                %Сверка числа отсчетов из шапки n и из данных nfakt
                if n > nfakt
        fprintf('Число отсчетов из шапки %d больше числа считанных отсчетов %d\n', n, nfakt)
                    n=nfakt;
        fprintf('Принято число считанных отсчетов %d\n', nfakt)
                    input('Нажмите ENTER');
                end
                %Считать из шапки длительность эксперимента tex
                tex=B(8);
                %tex=60;
                T0=tex/n; %Шаг квантования
                %Считать частоту fG в Гц
                fG=B(7)
                %Вычислить период гарм.сост.в секундах TG
                TG=1/fG;
                %и отсчетах 
                nnTG=TG/T0;
                nTG=round(nnTG); %Лучше её задавать!
                
                clc
                fprintf('Желаете изменить группу данных для обработки (по умолчанию - вторая)?\n')
                nmGRODi=input('Если да, введите 1: ');
                if nmGRODi==1
                    nmGROD=1;
                else
                    nmGROD=2;
                end
    
                %Определение параметров "по умолчанию"
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
               
                %Для протокола
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
            fprintf('  Выбор диапазона для обработки\n')
            nbeg=input('Начало выборки:');
            nend=input('Конец выборки:');
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
            fprintf('Выберите координату для обработки\n')
            nmXY=input('(введите x, или y): ','s');
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
            fprintf('  Вывести спектральную плотность выбранного сигнала\n')
            nmYES=input('(введите y, если ДА, или n, если НЕТ):','s');
            if nmYES=='y'
                nTFour=nWYB; %Период для ДПФ в отсчетах
                %(Рекомендуется ближайшая меньшая степень двойки)
                TFour=nTFour*T0; %Период для ДПФ в сек.
                FRBT = fft(RBT); %ДПФ
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                %Рекомендуется 
                    %FRBT = fft(RBT,n2TFour); 
                    %n2TFour - ближайшая меньшая к nTFour степень двойки
                    %PRBT = FRBT.* conj(FRBT) / n2TFour;
                    %T2Four=n2TFour*T0;
                    %fFour = (0:nTFour)/T2Four; %Массив частот
                fFour = (0:(nTFour-1))/TFour; %Массив частот
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
            [mPRBT,ImPRBT] = max(PRBT(5:(fix(nTFour/2)-1)));
            fFourGR=fFour(ImPRBT+4)
            input('Нажмите ENTER');
            end
 
        case 4
            clc
            fprintf('  Отделение гармонической составляющей\n')
            
            fprintf('Проверить частоту по входному сигналу?\n')
            nmYES=input('(введите y, если ДА, или n, если НЕТ):','s');
            if nmYES=='y'
                %Проверка частоты по входному сигналу
                nTFour=n; %Период для ДПФ в отсчетах
                %(Рекомендуется ближайшая меньшая степень двойки)
                TFour=nTFour*T0; %Период для ДПФ в сек.
                nmX0Y0=input('(По x, или y)? ','s');
                if nmX0Y0=='x'
                    FRBT = fft(X0); %ДПФ
                else 
                    FRBT = fft(Y0); %ДПФ
                end
                PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
                fFour = (0:(nTFour-1))/TFour; %Массив частот
                figure;
                plot(fFour(1:nTFour),PRBT(1:nTFour))
                title('Power spectrum')
                xlabel('frequency (Hz)')
                fprintf('  Выведена спектральная плотность входного сигнала\n')
                [mPRBT,ImPRBT] = max(PRBT(1:(fix(nTFour/2)-1)));
                fFourG=fFour(ImPRBT)
                input('Нажмите ENTER');
            end
            
            fprintf('Гармоническая составляющая\n')
            nPTG=fix(nWYB/nTG); %Число периодов
            nWYBG=fix(nPTG*nTG); %Число отсчетов, кратное числу периодов
            RBG=RBT(1:nWYBG);
            NWYBG=NWYB(1:nWYBG);
            % Коэффициенты Фурье 1 порядка 
            fprintf('Коэффициенты Фурье 1 порядка\n')
            a1f = 2*mean(RBG.*cos(2*pi*fG*(NWYBG)*T0))
            b1f = 2*mean(RBG.*sin(2*pi*fG*(NWYBG)*T0))
            % Амплитуда и фаза
            fprintf('Амплитуда и фаза\n')
            AG = sqrt(a1f^2 + b1f^2)
            FIG = atan2(a1f,b1f)
            FIGG = FIG*180/pi
            %Для протокола
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
            fprintf('  Выведена гармоническая составляющая выбранного сигнала\n')
            input('Нажмите ENTER');
            RBTG=RBT-GS;
            figure;
            plot(NWYB,RBTG)
            grid
            nTFour=nWYB; %Период для ДПФ в отсчетах
            %(Рекомендуется ближайшая меньшая степень двойки)
            TFour=nTFour*T0; %Период для ДПФ в сек.
            FRBT = fft(RBTG); %ДПФ
            PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
            fFour = (0:(nTFour-1))/TFour; %Массив частот
            figure;
            plot(fFour(1:nTFour),PRBT(1:nTFour))
            title('Power spectrum')
            xlabel('frequency (Hz)')
            fprintf('  Выведен сигнал без гармонической составляющей\n')
            fprintf('и его спектральная плотность \n')
            [mPRBT,ImPRBT] = max(PRBT(5:(fix(nTFour/2)-1)));
            fFourGo=fFour(ImPRBT+4)
            input('Нажмите ENTER');
            clc
            
        case 51
            nbeg=1;
            nend=n;
            nDR=nend-nbeg+1;
            NDR=N(nbeg:nend);
            nWIND=fix(nDR/100); %Ширина окна
            %Проверить на >1
            nzWIND=fix(nWIND/2); %"Запаздывание"
            clc
            fprintf('Устранение дрейфа\n')
                figure;
            %key51=0;
            while 1
                %key51==0
                k51=menu('Устранение дрейфа','Увеличить окно','Уменьшить окно','Отделить дрейф');
                switch k51;
                case 1
                    if nWIND<nDR/2 
                        nzWIND=nWIND;
                        nWIND=nWIND*2;
                    else
                        input('Куда уж больше (Нажмите ENTER)');
                    end
                case 2
                    if nWIND>=4 
                        nWIND=nWIND/2;
                        nzWIND=fix(nWIND/2);
                    else
                        input('Куда уж меньше (Нажмите ENTER)');
                    end
                case 3 
                    %key51=1; 
                    break
                end
                Zd=Yd;
                %Осреднение "скользящим окном" выбранной ширины nWIND
                %Zd=Xd или Yd, ZDR - результат осреднения, ZbDR=Zd-ZDR
                mWIND=0;
                for iDR=1:nzWIND
                    mWIND=(mWIND*(iDR-1)+Zd(iDR))/iDR;
                end
                for iDR=nzWIND+1:nDR+nzWIND
                    if iDR<=nWIND
                        %"Накапливать" среднее
                        mWIND=(mWIND*(iDR-1)+Zd(iDR))/iDR;
                    elseif iDR>nDR
                        %"Списывать" среднее
                        mWIND=(mWIND*(nWIND-(iDR-nDR)+1)-Zd(iDR-nWIND))/(nWIND-(iDR-nDR));
                    else
                        %Обновлять среднее
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
            fprintf('  Отделение линейного тренда\n')
            RBTGL = detrend(RBTG);
            LT=RBTG-RBTGL;
            figure;
            plot(NWYB,RBTG,NWYB,LT)
            grid
            fprintf('  Выведен линейный тренд выбранного сигнала\n')
            input('Нажмите ENTER');
            figure;
            plot(NWYB,RBTGL)
            grid
            nTFour=nWYB; %Период для ДПФ в отсчетах
            %(Рекомендуется ближайшая меньшая степень двойки)
            TFour=nTFour*T0; %Период для ДПФ в сек.
            FRBT = fft(RBTGL); %ДПФ
            PRBT = FRBT.* conj(FRBT) / nTFour; %The power spectrum
            fFour = (0:(nTFour-1))/TFour; %Массив частот
            figure;
            plot(fFour(1:nTFour),PRBT(1:nTFour))
            title('Power spectrum')
            xlabel('frequency (Hz)')
            fprintf('  Выведен сигнал без линейного тренда\n')
            fprintf('и его спектральная плотность \n')
            [mPRBT,ImPRBT] = max(PRBT(1:(fix(nTFour/2)-1)));
            fFourGoo=fFour(ImPRBT)
            input('Нажмите ENTER');

        case 6
            clc
            fprintf('Статистический анализ\n')
            fprintf('  Выборочное среднее\n')
            mx = mean(RBTGL) % Выборочное среднее
            fprintf('  Выборочная дисперсия\n')
            m2x = mean((RBTGL - mx).^2) % Выборочная дисперсия
            fprintf('  и СКО\n')
            s2x=sqrt(m2x) % СКО
            sx = std(RBTGL) % Стандартное отклонение 
            fprintf('  Выборочный центральный момент 3 порядка\n')
            m3x = mean((RBTGL - mx).^3) % Выборочный центральный момент 3 порядка
            fprintf('  Выборочный центральный момент 4 порядка\n')
            m4x = mean((RBTGL - mx).^4) % Выборочный центральный момент 4 порядка
            fprintf('  Асимметрия\n')
            gamma1 = m3x/(m2x^1.5) % Асимметрия
            fprintf('  Эксцесс (для норм.распределения 3)\n')
            gamma2 = m4x/(m2x^2)  % Эксцесс
            fprintf('  Контрэксцесс (для норм.распределения ~0.577)\n')
            gamma05 = m2x/sqrt(m4x)  % Контрэксцесс
            input('Нажмите ENTER');
            %Для протокола
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
            
            %Гистограмма
            kHIST1=3.31*log10(nWYB)+1
            kHIST2=5*log10(nWYB)
            kHIST=round((kHIST1+kHIST2)/2)
            %fprintf('Число участков гистограммы %d\n', kHIST)
            %nmYES=input('Хотите изменить(введите y, если ДА, или n, если НЕТ):','s');
            %if nmYES=='y'
                %fprintf('Рекомендуемый диапазон %d - %d\n', round(kHIST1), round(kHIST2))
                %kHIST=input('Число участков гистограммы?');
            %end
            [NHIST, yHIST] = hist(RBTGL,kHIST);
            NHISTnWYB=NHIST/nWYB; %Относительные частоты
            hHIST=(yHIST(kHIST)-yHIST(1))/kHIST; %Длина разряда гистограммы
            hHIST1=yHIST(2)-yHIST(1); %Длина разряда гистограммы
            hHISTn=(max(RBTGL)-min(RBTGL))/kHIST; %Длина разряда гистограммы
            %Теоретические вероятности
            fxH = normpdf(yHIST,mx,sx)*hHIST;
            %Хи-квадрат
            hi2=nWYB*sum((NHISTnWYB-fxH).^2/fxH)
            
            bar(yHIST, NHIST)
            %Диапазон "3-сигма" с шагом 0.1*CKO
            ksi = (-3.0:0.1:3.0)*sx + mx;
            %Теоретические вероятности
            fx = normpdf(ksi,mx,sx)*hHIST;
            figure;
            plot(ksi,fx, yHIST, NHISTnWYB)
            
            %Тест Колмогорова-Смирнова
            %xKSTST = (RBTGL - mx)/sx;
            %HKSTST = kstest(xKSTST);
            
            %Для протокола
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
            
            input('Нажмите ENTER');

        case 7
            %Протокол
            %Пересчет в мм (1 мм = 15 пикселям)
            %Амплитуда гармонической составляющей (в мм) 
            AGXmm=AGX/15;   
            AGYmm=AGY/15;
            %Выборочное среднее (в мм)
            mxXmm=mxX/15;   
            mxYmm=mxY/15;
            %Выборочная дисперсия                        
            m2xXmm=m2xX/15/15;  
            m2xYmm=m2xY/15/15;
            %СКО (cтандартное отклонение) (в мм)         
            sxXmm=sxX/15;   
            sxYmm=sxY/15;
            
            yHISTXmm=yHISTX/15; 
            yHISTYmm=yHISTY/15;
            %Максимальное смещение (в мм)                
            DELMXX=AGXmm+mxXmm+3*sxXmm;
            DELMXY=AGYmm+mxYmm+3*sxYmm;
            %Угловые скорости
            BETATMX=2*pi*B(6)*B(7);
            BETATMD=0.636*BETATMX;
            
            clc
            fprintf('   ПАРАМЕТРЫ ЭКСПЕРИМЕНТА\n')
            fprintf('Дата  %d\n', B(1))
            fprintf('Тип шлема (1-ЗШ,2-АШ)  %d\n', B(2))
            fprintf('Масса ИНА (в кг) %f\n', B(3))
            fprintf('Номер оператора  %d\n', B(4))
            fprintf('Тип траектории  %d\n', B(5))
            fprintf('Амплитуда воздействия (в градусах)  %d\n', B(6))
            fprintf('Частота гармонического воздействия (в Гц) %f\n', B(7))
            fprintf('Угловая скорость максимальная (в градусах/с)  %f\n', BETATMX)
            fprintf('Угловая скорость средняя (в градусах/с)  %f\n', BETATMD)
            fprintf('Длительность эксперимента (в сек.)  %f\n', B(8))
            fprintf('Объём выборки  %d\n', B(9))
            
            fprintf('   РЕЗУЛЬТАТЫ ОБРАБОТКИ\n')
            fprintf('Диапазон обработки: от  %d  до  %d\n', nbeg, nend)

            fprintf('                                      По горизонтали (Х)  По вертикали (У)\n')
            fprintf('Максимальное смещение (в мм)                %f         %f\n', DELMXX, DELMXY)
            fprintf('Амплитуда гармонической составляющей (в мм) %f         %f\n', AGXmm, AGYmm)
            fprintf('Фаза гармонической составляющей (в рад.)    %f         %f\n', FIGX, FIGY)
            fprintf('                                (в град.)   %f         %f\n', FIGGX, FIGGY)
            fprintf('Выборочное среднее (в мм)                   %f         %f\n', mxXmm, mxYmm)
            fprintf('Выборочная дисперсия                        %f         %f\n', m2xXmm, m2xYmm)
            fprintf('СКО (cтандартное отклонение) (в мм)         %f         %f\n', sxXmm, sxYmm)
            fprintf('Асимметрия                                  %f         %f\n', gamma1X, gamma1Y)
            fprintf('Эксцесс                                     %f         %f\n', gamma2X, gamma2Y)
            fprintf('Контрэксцесс                                %f         %f\n', gamma05X, gamma05Y)
            fprintf('"Хи-Квадрат"                                %f         %f\n', hi2X, hi2Y)
          
            fprintf('   Параметры гистограмм\n')
            fprintf(' N инт.        X          P(X)          Y           P(Y)\n')
            kHIpr=kHISTX;
            if kHISTY > kHIpr, kHIpr=kHISTY, end;
            for ipr=1:kHIpr
fprintf('%d         %f      %f      %f     %f\n',ipr,yHISTXmm(ipr),NHISTX(ipr),yHISTYmm(ipr),NHISTY(ipr))
            end
            fprintf('Сохранить результаты?\n')
            nmYES=input('(введите y, если ДА, или любую другую клавишу, если НЕТ):','s');
            if nmYES=='y'
                indp=strfind(lower(filename),'.');
                if (~isempty(indp))
                    indp=indp(length(indp));
                    filenamepr=[filename(1:(indp-1)), '_pr.txt'];
                    filenamere=[filename(1:(indp-1)), '_1.res'];
                end
                fpr=fopen([pathname, filenamepr],'w');
                
fprintf(fpr,'   ПАРАМЕТРЫ ЭКСПЕРИМЕНТА\n');
fprintf(fpr,'Дата  %d\n', B(1));
fprintf(fpr,'Тип шлема (1-ЗШ,2-АШ)  %d\n', B(2));
fprintf(fpr,'Масса ИНА (в кг) %f\n', B(3));
fprintf(fpr,'Номер оператора  %d\n', B(4));
fprintf(fpr,'Тип траектории  %d\n', B(5));
fprintf(fpr,'Амплитуда воздействия (в градусах) %d\n', B(6));
fprintf(fpr,'Частота гармонического воздействия (в Гц) %f\n', B(7));
fprintf(fpr,'Угловая скорость максимальная (в градусах/с)  %f\n', BETATMX);
fprintf(fpr,'Угловая скорость средняя (в градусах/с)  %f\n', BETATMD);
fprintf(fpr,'Длительность эксперимента (в сек.)  %f\n', B(8));
fprintf(fpr,'Объём выборки  %d\n', B(9));
fprintf(fpr,'   РЕЗУЛЬТАТЫ ОБРАБОТКИ\n');
fprintf(fpr,'Диапазон обработки: от  %d  до  %d\n', nbeg, nend);
fprintf(fpr,'                                      По горизонтали (Х)  По вертикали (У)\n');
fprintf(fpr,'Максимальное смещение (в мм)                %f         %f\n', DELMXX, DELMXY);
fprintf(fpr,'Амплитуда гармонической составляющей (в мм) %f         %f\n', AGXmm, AGYmm);
fprintf(fpr,'Фаза гармонической составляющей (в рад.)    %f         %f (в рад.)\n', FIGX, FIGY);
fprintf(fpr,'                                (в град.)   %f         %f (в град.)\n', FIGGX, FIGGY);
fprintf(fpr,'Выборочное среднее (в мм)                   %f         %f\n', mxXmm, mxYmm);
fprintf(fpr,'Выборочная дисперсия                        %f         %f\n', m2xXmm, m2xYmm);
fprintf(fpr,'СКО (cтандартное отклонение) (в мм)         %f         %f\n', sxXmm, sxYmm);
fprintf(fpr,'Асимметрия                                  %f         %f\n', gamma1X, gamma1Y);
fprintf(fpr,'Эксцесс                                     %f         %f\n', gamma2X, gamma2Y);
fprintf(fpr,'Контрэксцесс                                %f         %f\n', gamma05X, gamma05Y);
fprintf(fpr,'"Хи-Квадрат"                                %f         %f\n', hi2X, hi2Y);
fprintf(fpr,'   Параметры гистограмм\n');
fprintf(fpr,' N инт.        X          P(X)          Y           P(Y)\n');
                for ipr=1:kHIpr
fprintf(fpr,'%d         %f     %f      %f    %f\n',ipr,yHISTX(ipr),NHISTX(ipr),yHISTY(ipr),NHISTY(ipr));
                end
                fclose(fpr);
                fprintf('Протокол сохранен в файле %s\n', filenamepr)
                
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
                fprintf('Результаты для дальнейшей обработки сохранены в файле %s\n', filenamere)
                input('Нажмите ENTER');
            end
            
        case 12
            figure;
            plot(XWYB,YWYB,'.')
            xlabel('X')
            ylabel('Y')
            grid
            
        case 99
            %Включать вместо пункта 2 главного меню
            clc
            fprintf('Тест\n')
            SIGMAxTST=input('СКО для теста по Х?');
            SIGMAyTST=input('СКО для теста по У?');
            MUxTST=0;
            MUyTST=0;
            AxTST=input('Амплитуда для теста по Х?');
            AyTST=input('Амплитуда для теста по У?');
            FIGxTST=input('Фаза для теста по Х?');
            FIGyTST=input('Фаза для теста по У?');
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
            %Включать после пункта 1 главного меню
            clc
            fprintf('ТестfGT0\n')
            AxTST=input('Амплитуда для теста по Х?');
            AyTST=input('Амплитуда для теста по У?');
            FIGGxTST=input('Фаза для теста по Х?');
            FIGGyTST=input('Фаза для теста по У?');
            T0T0=T0;
            delT0T0=T0T0/1000;
                figure;
            while 1
                tGTST=N*T0T0;
                X=AxTST*sin((2*fG*tGTST + FIGGxTST/180)*pi); 
                Y=AyTST*sin((2*fG*tGTST + FIGGxTST/180)*pi);   
                plot(N,Y0,N,Y)
                grid
                k=menu('Изменить шаг','Увеличить','Уменьшить','Не менять','Уточнить');
                switch k;
                case 1
                    T0T0=T0T0+delT0T0;
                case 2
                    T0T0=T0T0-delT0T0;
                case 3 
                    T0=T0T0;
                    %Период гарм.сост.в отсчетах 
                    nnTG=TG/T0;
                    nTG=round(nnTG); 
                    break
                case 4 
                    delT0T0=delT0T0/5;
                end
            end 
    end
end