classdef MainGameManager < GameObject
    properties (Constant)
        tolerance = 30%the mouse must place the grated circle at most this far from the target ring to win
        leftProportionInterval = 5;
        waterGiveTime = 0.06;
        successPauseTime = 2;%time between success and start of next trial
        failPauseTime = 4;%time between trials on failure
        timeOutTime = 10;%time for mouse to succeed before timeout
        stimPauseTime = 1;%time before circle control is enabled after stimulus is shown
        servoDelay = 0.5;%time after control is enabled before servos open
        servoOpenTime = 0.5;%time it takes for servos to adjust (estimated)
    end
    properties (Access = protected)
        ioDevice
        gratedCircle
        targetCircle
        controller
        results
        waitingForIR
        currentTrialNum
        maxTrials = 1000;
        hasHit
        allowIncorrect
        soundMaker
        background
    end
    methods (Access = public)
        function obj = MainGameManager(gratedCircle,targetCircle,background,controller,ioDevice,soundMaker,results)
            obj.gratedCircle = gratedCircle;
            obj.targetCircle = targetCircle;
            obj.background = background;
            obj.controller = controller;
            obj.ioDevice = ioDevice;
            obj.results = results;
            obj.soundMaker = soundMaker;
        end
        function obj = Awake(obj)
            obj.SetState(false,false);
            obj.WaitForIR();
            obj.currentTrialNum = 0;
        end
        function obj = Update(obj)
            if obj.currentTrialNum>obj.maxTrials
                obj.Game.Quit();
                return;
            end
            if obj.waitingForIR
                if obj.ioDevice.ReadIR()
                    obj.waitingForIR = false;
                    obj.StartTrial();
                end
                return;
            end
            if obj.gratedCircle.Distance(obj.targetCircle,1)<obj.tolerance
                obj.Success();
                return;
            end
            hit = obj.gratedCircle.GetScreenHits(1);
            if abs(hit)
                obj.Hit(hit);
                return;
            end
        end
        function obj = Success(obj)
            obj.controller.enabled = false;
            obj.soundMaker.RewardNoise();
            obj.gratedCircle.SetRootPosition(obj.targetCircle.GetGlobalPosition());
            obj.gratedCircle.SetVelocity([0,0]);
            if ~isempty(obj.results)
                obj.results.LogSuccess(obj.Game.GetTime());
            end
            obj.ioDevice.GiveWater(obj.waterGiveTime);
            obj.ioDevice.CloseServos();
            obj.DisableFor(obj.successPauseTime);
            obj.DelayedCall('EndTrial',obj.successPauseTime);
        end
        function obj = Hit(obj,side)
            if obj.hasHit
                return;
            end
            obj.hasHit = true;
            if ~isempty(obj.results)
                obj.results.LogHit(side);
            end
            if ~obj.allowIncorrect
                obj.EndTrial();
                obj.Failure();
            else
                obj.soundMaker.BadNoise();
            end
        end
        function obj = WaitForIR(obj)
            obj.waitingForIR = true;
        end
        function obj = StartTrial(obj)
            obj.StopAllDelayedCalls();
            side = obj.ChooseSide();
            obj.SetState(true);
            obj.hasHit = false;
            obj.gratedCircle.Reset(side);
            if ~isempty(obj.results)
                obj.results.StartTrial(side,1,obj.Game.GetTime());
            end
            obj.currentTrialNum = obj.currentTrialNum +1;
            obj.DelayedCall('TimeOut',obj.timeOutTime);
        end
        function obj = EndTrial(obj)
            obj.StopAllDelayedCalls();
            obj.SetState(false);
            if obj.currentTrialNum>0
                clc;
                if ~isempty(obj.results)
                    obj.results.shortStats();
                end
            end
            if ~isempty(obj.results)
                obj.results.save();
            end
            obj.WaitForIR();
        end
        function out = ChooseSide(obj)
            %chooses the side that the stimulus will appear on
            %returns -1 (left) or 1 (right)
                choice = rand();%used to decide if grated circle starts on the left or right
                if ~isempty(obj.results)
                    leftBias = obj.results.getLeftProportionOnInterval(5);
                else
                    leftBias = 0.5;
                end
                
                if choice < leftBias
                    out = -1;%start the stimulus on the left (forcing mouse to choose right)
                else
                    out = 1;
                end
        end
        function obj = SetState(obj,running,closeServos)
            if nargin<3
                closeServos = true;
            end
            if running
                obj.controller.DisableFor(obj.stimPauseTime+obj.servoDelay);
                obj.DelayedServoOpen();
            else
                if closeServos
                    obj.ioDevice.CloseServos();
                end
                obj.controller.enabled = false;
                if ~isempty(obj.background)
                    obj.background.RandomizePositions();
                end
            end
            obj.targetCircle.enabled = running;
            obj.gratedCircle.enabled = running;
            if ~isempty(obj.background)
                obj.background.enabled = running;
            end
            obj.ResetBackground();
        end
        function obj = SetMaxTrials(obj,num)
            obj.maxTrials = num;
        end
        function obj = TimeOut(obj)
            obj.EndTrial();
            if ~obj.ioDevice.ReadIR() && ~isempty(obj.results)
                obj.results.cancelTrial();
            else
                obj.Failure();
            end
        end
        function obj = Failure(obj)
            obj.soundMaker.BadNoise();
            obj.DisableFor(obj.failPauseTime);
            obj.Renderer.SetBackgroundColor(0);
            obj.DelayedCall('ResetBackground',obj.failPauseTime);
        end
        function obj = ResetBackground(obj)
             obj.Renderer.ResetBackgroundColor();
        end
        function obj = SetAllowIncorrect(obj,bool)
            obj.allowIncorrect = bool;
        end
        function out = GetNumberOfGamesPlayed(obj)
            out = obj.currentTrialNum;
        end
        function obj = DelayedServoOpen(obj)
            obj.ioDevice.DelayedCall('OpenServos',obj.stimPauseTime+obj.servoDelay);
        end
    end
end