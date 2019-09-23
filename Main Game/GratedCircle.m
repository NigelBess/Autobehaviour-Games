classdef GratedCircle < PhysicsObject
    properties(Constant)
        radius = 100;
        contrast = 1;
    end
    properties(Access = protected)
        initialOffset;
        opacityPool;
    end
    methods(Access = public)
        function obj = GratedCircle(opacityPool)
            if nargin < 1
                obj.opacityPool = 1;
            else
                obj.opacityPool = opacityPool;
            end
            obj.size = ones(1,2)* 2*obj.radius;
            obj.renderLayer = 1;
            obj.screenBounded = true;
            obj.maxPixelsPerFrame = 10;
        end
        function obj = Awake(obj)
            windowSize = obj.Renderer.WindowSize();
            obj.initialOffset = (windowSize(1)/2-obj.radius)/2;
        end
        function img = GenerateImage(obj)
            img = obj.PngToImg('grating.png');
        end
        function obj = Reset(obj,side)
            obj.SetAlpha(randsample(obj.opacityPool, 1)); 
            obj.SetVelocity(0,1);
            obj.SetRootPosition([obj.initialOffset*side,0]);
        end
        function out = GetAlpha(obj)
            out = obj.globalAlpha;
        end
    end
end
