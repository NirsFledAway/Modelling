close all;
rad = [1 2 5 10 20 100];
amp = [0.9979 0.9735 0.8626 0.712 0.395 0.08];
amp_db = 20*log10(amp);
figure; 
semilogx(rad, amp_db);
grid on

%%
close all
rad = [0.2 0.5 0.8 1 2 4];
amp = [0.999 0.8935 0.7614 0.678 0.4135 0.22475];
amp_db = 20*log10(amp);
figure; 
semilogx(rad, amp_db);
grid on