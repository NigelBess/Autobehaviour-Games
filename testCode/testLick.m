clc
clear all;
requestInput();
addpath('Common');
addpath('PTB-Game-Engine/GameEngine');
fprintf("connecting...\n");
io = HardwareIOGen5(port);
io.Awake();
fprintf("arduino setup complete\n");
while ~GetKey("ESC")
    clc;
    if io.ReadLick()
        fprintf("LICKMETER ACTUATED\n");
    else
        fprintf("0\n");
    end
    pause(0.1);
end