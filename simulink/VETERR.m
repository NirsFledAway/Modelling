function Wb = VETERR(u)
% ykr=u(1); ta=u(2); kyr=u(3); %�������� ����, ������, ����. 
% va=u(4); % ��������� �������� ��� �����
% vay=0; vaz=0;
% V=[va;vay;vaz];
% %******************************************* 
% bb11=cos(kyr)*cos(ykr)-sin(kyr)*sin(ta)*sin(ykr);    
% bb12=cos(ta)*sin(ykr);                               
% bb13=cos(kyr)*sin(ykr)+sin(kyr)*sin(ta)*cos(ykr);    
% 
% bb21=sin(kyr)*cos(ykr)+cos(kyr)*sin(ta)*sin(ykr); 
% bb22=cos(ta)*cos(kyr);                           
% bb23=sin(kyr)*sin(ykr)-cos(kyr)*sin(ta)*cos(ykr); 
% %---------------------------------------
% bb31=-cos(ta)*sin(ykr); 
% bb32=sin(ta);                     
% bb33=cos(ta)*cos(ykr);  
% %---------------------------------------
% B3=[bb11 bb12 bb13; bb21 bb22 bb23; bb31 bb32 bb33];
% P3=[0 0 1; 1 0 0;0 1 0];% ��������������� �������
% P3tr=P3';
% Wxyz=(P3tr*B3*P3)*V;
% %*****************************************
% vixod(1)=Wxyz(1);
% vixod(2)=Wxyz(2);
% vixod(3)=Wxyz(3);

phi = u(1); t = u(2); psi = u(3); va = u(4);
R = getRotationMatrix([phi t psi]);
vay=0; vaz=0;
Va = [va; vay; vaz];
Wb = R*Va;

function R = getRotationMatrix(angles)
    phi = 1; t = 2; psi = 3; %indices
    s = sin(angles);
    c = cos(angles);
    R = [
        c(t)*c(psi) s(t)    -c(t)*s(psi);
        -c(phi)*s(t)*c(psi)+s(phi)*s(psi) c(phi)*c(t) c(phi)*s(t)*s(psi)+s(phi)*c(psi);
        s(phi)*s(t)*c(psi)+c(phi)*s(psi)  -s(phi)*c(t)    -s(psi)*s(t)*s(phi)+c(psi)*c(phi);
    ];

