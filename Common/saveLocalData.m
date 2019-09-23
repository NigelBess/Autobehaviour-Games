if ~exist('values.mat','file')
    createEmpty;
    fprintf('creatd file: values.mat');
end
    vars = {'mouseID','numTrials','port','psswd','reward','saveDir','rig','screenNum','natBackground','sender','sessionNum','mailRecipient'};
for i = 1:numel(vars)
    try
        save('values',vars{i},'-append');
    catch
    end
end
clear vars;
function createEmpty()
    clear all;
    save('values');
end