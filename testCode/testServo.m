clc
clear all;

requestInput;
addpath('Common');
addpath('PTB-Game-Engine/GameEngine');
fprintf("connecting...\n");
io = HardwareIOGen5(port);
io.Awake();
fprintf("arduino setup complete\n");
io.PowerServos(true);
open = false;
while true
in = input("Press enter to make the servos open or close. CTRL-C to exit.");
open = ~open;
if open
    
    io.OpenServos();
    clc;
    disp("servos open");
else
    
    io.CloseServos();
    clc;
    disp("servos closed");
end
io.PrintServoTargets();
pause(0.5);
io.PowerServos(false);
end