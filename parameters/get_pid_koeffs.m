function Modes = get_pid_koeffs()

Modes = struct()

% Stabilize fast
Reg = struct()
Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    14
    0
    5
]';
Reg.PID3 = [
    85
    85
    25
]';
Modes.Stab_fast.Reg = Reg;
Modes.Stab_fast.N = 0;

% Flight on Beard targeting
Modes.Targeting_1.N = 1;
Reg = struct()
Reg.PID1 = [
    200
    0
    10
]';
Reg.PID2 = [
    0
    0
    5
]';
Reg.PID3 = [
    0
    0
    10
]';
Modes.Beard_targeting.Reg = Reg;
Modes.Beard_targeting.N = 2;

% Stabilize gently
Reg = struct()
% Reg.PID1 = [
%     400
%     0
%     10
% ]';
% Reg.PID2 = [
%     14
%     0
%     5
% ]';
Reg.PID1 = [
    200
    60
    15
]';
Reg.PID2 = [
    12
    6
    7
]';
% Reg.PID1 = [
%     200
%     0
%     10
% ]';
% Reg.PID2 = [
%     14
%     0
%     5
% ]';
Reg.PID3 = [
    85
    0
    60
]';
Reg.PID3_speed = [
    0
    0
    60
]';
Modes.Stab_gently.Reg = Reg;
Modes.Stab_gently.N = 3;

Modes.Soft_falling_1.Falling_speed = -0.3;