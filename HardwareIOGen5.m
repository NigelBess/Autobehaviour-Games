classdef HardwareIOGen5 < Rig
    properties(Constant)
        leftServoPin = "D10"
        rightServoPin = "D9"
        servoPowerPin = "D6"
        encoderPinA = "D2"
        encoderPinB = "D3"
        solenoidPin = "D8"
        lickmeterReadPin  = "A2"
        breakBeamPin = "D7"
        lickVoltageDelta = 1;
        lickNominalVoltage = 5;
        
    end
    methods (Access = public)
        function obj = HardwareIOGen5(port)
            obj.leftServoOpenPos = 0;
            obj.rightServoOpenPos = 1;
            obj.leftServoClosedPos = 1;
            obj.rightServoClosedPos = 0;
            obj.port = port;
            obj.digitalOutputPins = [obj.solenoidPin, obj.servoPowerPin];
            obj.digitalInputPins = [];
            obj.analogInputPins = [obj.lickmeterReadPin];
            obj.pullupPins = [obj.breakBeamPin];
        end
        function obj = Awake(obj)          
            obj.arduinoBoard = arduino(obj.port,'uno','libraries',{'servo','rotaryEncoder'});
            obj.ConfigurePins();            
            obj.encoder = rotaryEncoder(obj.arduinoBoard, obj.encoderPinA,obj.encoderPinB);
            
            obj.leftServo = servo(obj.arduinoBoard,obj.leftServoPin,'MinPulseDuration',obj.minServoPulse,'MaxPulseDuration',obj.maxServoPulse);
            obj.rightServo = servo(obj.arduinoBoard,obj.rightServoPin,'MinPulseDuration',obj.minServoPulse,'MaxPulseDuration',obj.maxServoPulse);
             obj.CloseServos();
             
             
        end
         function out = UnsafeReadJoystick(obj)
            out = readCount(obj.encoder)/obj.maxJoystickValue;
            if abs(out)>1
                out = sign(out);
                obj.ResetEnc(out*obj.maxJoystickValue);
                return;
            end
            if abs(out)<obj.joystickResponseThreshold
                out = 0;
                return;
            end 
         end
        function out = ReadIR(obj)         
               out = obj.Try('UnsafeReadIR');
        end
        function out = UnsafeReadIR(obj)
                out = ~readDigitalPin(obj.arduinoBoard,obj.breakBeamPin);
        end
        function out = ReadLick(obj)
            val = readVoltage(obj.arduinoBoard,obj.lickmeterReadPin);
            out = abs(val-obj.lickNominalVoltage)>obj.lickVoltageDelta;
            disp(val);
        end
        function obj = GiveWater(obj,time)
             writeDigitalPin(obj.arduinoBoard,obj.solenoidPin,1);
             if obj.lastWaterTime>0
                 time = time + obj.evaporationConstant*(obj.Game.GetTime() - obj.lastWaterTime);
             end
             obj.lastWaterTime = obj.Game.GetTime();
             obj.DelayedCall('CloseSolenoid',time);
        end
        function obj = CloseSolenoid(obj)
            writeDigitalPin(obj.arduinoBoard,obj.solenoidPin,0);
        end
        function obj = PositionServos(obj,left,right)
            obj.PowerServos(true);
            obj.PositionServos@Rig(left,right);
            obj.DelayedCall('PowerServos',obj.servoAdjustmentTime,false);

        end
        function obj = PowerServos(obj,state)
            writeDigitalPin(obj.arduinoBoard,obj.servoPowerPin,state);
        end
        function obj = TurnOffEverything(obj)
            obj.TurnOffEverything@Rig();
            obj.PowerServos(false);
        end

    end
end