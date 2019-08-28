classdef  Gen5Rig < IODevice
    properties(Constant)
        %pins
        leftServoPin = "D10"
        rightServoPin = "D9"
        servoPowerPin = "D6"
        encoderPinA = "D2"
        encoderPinB = "D3"
        solenoidPin = "D8"
        lickmeterReadPin  = "A2"
        breakBeamPin = "D7"
        
        
        lickVoltageDelta = 1;%expected change in voltage on lickmeter when mouse licks
        lickNominalVoltage = 5;%expected voltage from the lickmeter
        servoAdjustmentTime = 0.65;%expected time it takes the servos to move
        evaporationConstant = .15/3600;%how much water evaporates per time. (unknown units calculated by james)
        maxJoystickValue = 40;%max expected change in joystick reading
        joystickResponseThreshold = 0.2;%ratio of max at which we return a value. this gives the joystick a deadzone
       
        %servo positions in degrees
        leftServoOpenPos = 90;
        rightServoOpenPos = 180;
        leftServoClosedPos = 180;
        rightServoClosedPos = 90; 
    end
    
    properties(Access = protected)
        arduino;
        lastWaterTime;
    end
    
    methods (Access = public)
        
        function obj = Gen5Rig(port)
            obj.arduino = Arduino(port);
        end
        
        function obj = Awake(obj)          
            obj.arduino.connect();  
            obj.ConfigurePins();            
            obj.arduino.encoder(obj.encoderPinA,obj.encoderPinB);
            obj.arduino.attachServo(obj.leftServoPin);
            obj.arduino.attachServo(obj.rightServoPin);
            obj.CloseServos();  
        end
        
         function out = ReadJoystick(obj)
            out = obj.arduino.getEncoderCount(obj.encoderPinA)/obj.maxJoystickValue;
            if abs(out)<obj.joystickResponseThreshold
                out = 0;
            end 
         end
         
        function out = ReadIR(obj)         
               out = ~obj.arduino.digitalRead(obj.breakBeamPin);
        end

        function out = ReadLick(obj)
            val = obj.arduino.analogRead(obj.lickmeterReadPin);
            out = abs(val-obj.lickNominalVoltage)>obj.lickVoltageDelta;
        end
        
        function obj = GiveWater(obj,time)
             obj.arduino.digitalWrite(obj.solenoidPin,1);
             if obj.lastWaterTime>0
                 time = time + obj.evaporationConstant*(obj.Game.GetTime() - obj.lastWaterTime);
             end
             obj.lastWaterTime = obj.Game.GetTime();
             obj.DelayedCall('CloseSolenoid',time);
        end
        
        function obj = CloseSolenoid(obj)
           obj.arduino.digitalWrite(obj.solenoidPin,0);
        end
        
        function obj = CloseServos(obj)
            obj.PositionServos(obj.leftServoClosedPos,obj.rightServoClosedPos);
            obj.DelayedCall('ResetEncoder',obj.servoAdjustmentTime);
        end
        function obj = ResetEncoder(obj)
            obj.arduino.resetEncoder(obj.encoderPinA);
        end
        function obj = OpenServos(obj)
             obj.PositionServos(obj.leftServoOpenPos,obj.rightServoOpenPos);
        end
        function obj = OpenSide(obj,side)
            if side<0
                obj.PositionServos(obj.leftServoOpenPos,obj.rightServoClosedPos);
            else
                obj.PositionServos(obj.leftServoClosedPos,obj.rightServoOpenPos);
            end
        end
        function obj = PositionServos(obj,left,right)
            obj.PowerServos(true);
            obj.arduino.writeServo(obj.leftServoPin,left);
            obj.arduino.writeServo(obj.rightServoPin,right);
            obj.DelayedCall('PowerServos',obj.servoAdjustmentTime,false);

        end
        function obj = PowerServos(obj,state)
            obj.arduino.digitalWrite(obj.servoPowerPin,state);
        end
        function obj = TurnOffEverything(obj)
            obj.CloseSolenoid();
            obj.arduino.detachServo(obj.leftServoPin);
            obj.arduino.detachServo(obj.rightServoPin);
            obj.arduino.detachEncoder(obj.encoderPinA);
            obj.PowerServos(false);
        end
        function obj = ConfigurePins(obj)
            outputPins = [obj.solenoidPin, obj.servoPowerPin];
            inputPins = [obj.lickmeterReadPin];
            pullupPins = [obj.breakBeamPin];
            pinGroups = {outputPins,inputPins,pullupPins};
            types = ["OUTPUT","INPUT","INPUT_PULLUP"];
            for i = 1:numel(pinGroups)
                type = types(i);
                for pin = pinGroups{i}
                    obj.arduino.pinMode(pin,type);
                end
            end
        end

    end
end