classdef NaturalBackground < Renderable
    properties(Constant)
    end
    properties(Access = private)
        range;
    end
    methods(Access = public)
        function obj = NaturalBackground(imgFileName, range)
            %range defines [x range, y range] where each element is the
            %percent of the screen (x and y) that can be filled by the
            %instances
            if nargin<3
                range = [1,1];
            end
            obj.range = range;
            obj.image = obj.PngToImg(imgFileName);
            obj.screenBounded = false;
        end
        
        function obj = Awake(obj)
            pos = [0, 0];
            obj.InstantiateNew(pos, obj.Renderer.WindowSize() * 2);  % Render to 2X the window size
            obj.RandomizePositions;
        end
        
        function obj = RandomizePositions(obj)
            obj.position = obj.GenerateRandomPosition;
        end
        
        function out = GenerateRandomPosition(obj)
            windowSize = obj.Renderer.WindowSize();
            heightPool = [-windowSize(2) / 2, windowSize(2) / 2];
            
            
            widthPool = [-500: 100: 500]; %(-(size(obj.image, 2) / 2 - windowSize(1) / 2) : 100 : ...
            %    (size(obj.image, 2) / 2 - windowSize(1) / 2));
            out = [randsample(widthPool, 1), randsample(heightPool, 1)];
        end
        
        
        function img = scaleToScreen(obj, img)
            windowSize = obj.Renderer.WindowSize();
            scaleFactor = windowSize(2) / size(img, 1);
            img = imresize(img, 2 * scaleFactor);
        end
        
        function img = GenerateImage(obj)
            % here we want to subsample a part of he image
            img = obj.scaleToScreen(obj.image);
        end
    end
end
