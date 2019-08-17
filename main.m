
%clear workspace
clc; 
clear all;

addpath('PTB-Game-Engine/GameEngine');     %game engine
addpath('Common');                         %files used by all the games (main, lickspout only, and joystick only)
addpath('Main Game');                      %files used by this game
addpath('Img');                            %image files

requestInput;%get rig specific data from user via GUI

developerMode = (mouseID == '0');%this allows devmode to be set independent of rig, and ignored by git requests

%if we are in developer mode, give user the option to use keyboard as input
if developerMode
choice = menu('Keyboard or Autobehavior Rig input?','Keyboard','Rig');
usingKeyboard = choice==1;
else
usingKeyboard = false;
end

%what percent of the screen do we want to render to?
%[min x, min y, max x, max y] 0 and 1 are the edges of the screen
% this is used for multiple monitor setups like in the headfixed rigs
rect = [0,0,1,1];



secondarySaveDir = 'C:/Autobehavior Data/';
if usingKeyboard
    io = Keyboard;
else
    choice = menu('Which circuit board are you using?','Gen4', 'Gen5','Headfixed');
    switch choice
        case 1
            io = HardwareIOGen4(port);
        case 2
            io = HardwareIOGen5(port);
        case 3
            io = HardwareHeadfixed(port,str2num(rig));
            % headfixed rigs use a triple monitor setup
            % we can choose to render to only the middle monitor by setting
            % the rect to the middle third of the screen
            rect = [1/3,0,2/3,1];
    end
end

%initialize objects
emailer = Emailer('sender','recipients',developerMode);%doesn't send mail if we are in dev mode
results = Results(mouseID,numTrials,sessionNum,'closedLoopTraining');
results.setSaveDirectory(saveDir, secondarySaveDir);
renderer = Renderer(screenNum,0.5,rect);%(screenNumber,default background color,rect to render to)
grating = GratedCircle;
greenCirc = TargetRing;
%background = RandomizedBackground('backgroundDot.png',40,[2,1]);%(image, quantity, [x range, y range])
%background.SetParent(grating);%make background the child of grating so that the move in unison
grating.RenderAfter(greenCirc);%make grating render behind green circle
%background.RenderAfter(grating);%make background render  last
iescape = EscapeQuit;%object that makes game quit if you press the escape key
sound = SoundMaker;


controller = GratedCircleController(grating,io);

%manager handles game logic. Constructor params are references to objects
%that it interacts with
manager = MainGameManager(grating,greenCirc,[],controller,io,sound,results);
manager.SetMaxTrials(numTrials);
manager.SetAllowIncorrect(reward);


ge = GameEngine;



%game engine has built in error handling, but we want to do email error
%notification outside of that
%the reason why is that built in error handling deals with safety stuff
%like making sure the solenoids are shut off if an error occurs.
% emailing takes a long time, so if that were to happen before the solenoid
% error function, the mouse cage could get flooded
try
    ge.Start(); 
catch e
    msg = char(getReport(e,'extended','hyperlinks','off'));
    subject = "Autobehaviour ERROR: rig "+string(rig)+" mouse "+string(mouseID);
    emailer.Send(subject,msg);
    rethrow(e);
end
if manager.GetNumberOfGamesPlayed()>=numTrials
    msg = char("mouse "+string(mouseID) + " on rig " + string(rig) + " has successfully completed " + string(numTrials) + " trials.");
    subject = "Autobehaviour SUCCESS: rig "+string(rig)+" mouse "+string(mouseID);
    emailer.Send(subject,msg);
end
