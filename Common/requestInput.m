function [] = requestInput()
    mouseID = [];rig = [];sessionNum=[];numTrials = [];port = [];screenNum=[];natBackground=[];reward = []; saveDir = 'Z:/Autobehaviour Data';
    if exist('values.mat','file')
        load('values.mat');
    end
    prompt = {"Mouse ID","Rig" ,"Session Number", "Number of Trials", "Port","Output Monitor Number (zero for single monitor)","Naturalistic Background", "Reward On Incorrect (0 or 1)","Save Directory"};
    title = "Settings";
    dims = [1 35];
    
    
    defInput = {mouseID,rig,sessionNum,numTrials,port,screenNum,natBackground,reward,saveDir};
    for i = 1:numel(defInput)
        if ~isstring(defInput{i})
            defInput{i} = num2str(defInput{i});
        end
    end
    
    val = inputdlg(prompt,title,dims,defInput);
    mouseID = val{1};
    rig = val{2};
    sessionNum = str2num(val{3});
    numTrials = str2num(val{4});
    port = val{5};
    screenNum = str2num(val{6});
    natBackground = str2num(val{7});
    reward = str2num(val{8});
    saveDir = val{9};
    saveLocalData;
    evalin('base',"load('values')");
end
    
