classdef Emailer < handle
    properties (Access = public)
        sender
        password
        recipientList
        senderFileName
        recipientFileName
        developerMode
    end
    methods (Access = public)
        function obj = Emailer(senderFileName,recipientFileName,developerMode)
            if nargin<3
                developerMode = false;
            end
            obj.developerMode = developerMode;
            if obj.developerMode
                return;
            end
            obj.senderFileName = senderFileName;
            obj.recipientFileName = recipientFileName;
            try
            senderData = LoadStrings(senderFileName);
            obj.SetSender(senderData{1});
            obj.SetPassword(senderData{2});
            catch 
            end
            try
            obj.recipientList = LoadStrings(recipientFileName);
            catch
            end
            obj.Auth();
        end
        function obj = SetSender(obj,str)
            obj.sender = string(str);
        end
        function obj = SetPassword(obj,str)
            obj.password = string(str);
        end
        function obj = Auth(obj)
            if isempty(obj.sender) || isempty(obj.password)
                obj.AuthError();
            else
                obj.Send("(Ignore) AuthenticationMessage","This is a test message used for authentication.",obj.sender);
            end
        end
        function obj =  AuthError(obj)
            choice = menu("Email error notification has not yes been set up or the username/password is incorrect. Would you like reset the sender email address/password?","Yes","No, continue without emailing errors.");
            if choice == 1
               obj.InputSender();
               obj.Save(obj.senderFileName,{obj.sender,obj.password});
               obj.EditRecipients();
               obj.Auth();
            end
        end
        function obj = EditSender(obj)
             if ~isempty(obj.sender) && ~isempty(obj.password)
                   choice = menu("Current mail sender: "+string(obj.sender),"Ok","Change mail sender");
                   if choice == 2
                    obj.InputSender();
                   end
             else
                 obj.InputSender();
             end
             obj.Save(obj.senderFileName,{obj.sender,Obfuscate(obj.password)});
        end
        function obj = InputSender(obj)
            obj.SetSender(input('\nEmail address to send error messages: ','s'));
            obj.SetPassword(Obfuscate(input('\nPassword of sender email. WARNING THIS WILL BE STORED LOCALLY AS PLAINTEXT: ','s')));
        end
        function obj = EditRecipients(obj)
             if ~isempty(obj.recipientList)
                    str = "Current recipient list: ";
                    for i = 1:numel(obj.recipientList)
                        str = str + obj.recipientList{i};
                        if i ~= numel(obj.recipientList)
                            str = str + ", ";
                        end
                    end
                    choice = menu(str ,"OK","Add recipient","Remove recipient");
                    if choice == 2
                        obj.AddRecipient();
                    end
                    if choice == 3
                        obj.RemoveRecipient();
                    end
             else
                 obj.AddRecipient();
             end
             obj.Save(obj.recipientFileName,obj.recipientList)
        end
        function obj = AddRecipient(obj)
            if isempty(obj.recipientList)
                obj.recipientList = cell(0);
            end
            obj.recipientList{end+1} = input('New Recipient Email: ','s');
            obj.EditRecipients();
        end
        function obj = RemoveRecipient(obj)
            rec = obj.recipientList;
            n = numel(rec);
            rec{end+1} = "Cancel";
            choice = menu("Which recipient would you like to remove?",rec);
            if choice> n
                obj.EditRecipients();
            else
                obj.recipientList(choice) = [];
            end
        end

        function obj = Send(obj,subject,message,recipients)
            if nargin<4
                recipients = obj.recipientList;
            end
            if obj.developerMode
                return;
            end
            try
            if isempty(recipients)
                error('no recipient declared');
            end
            subject = char(subject);
            message = char(message);
            
            sendr= char(obj.sender);
            pwd = char(Obfuscate(obj.password,true));
            setpref('Internet','E_mail',sendr);
            setpref('Internet','SMTP_Server','smtp.gmail.com');
            setpref('Internet','SMTP_Username',sendr);
            setpref('Internet','SMTP_Password',pwd);

            props = java.lang.System.getProperties;
            props.setProperty('mail.smtp.auth','true');
            props.setProperty('mail.smtp.socketFactory.class','javax.net.ssl.SSLSocketFactory');
            props.setProperty('mail.smtp.socketFactory.port','465');
            sendmail(recipients, char(subject), char(message));
            catch e
                obj.AuthError();
                warning(getReport(e));
            end
        end
        function out = GetRecipientList(obj)
            out = obj.recipientList;
        end
        function [] = Save(obj,fileName,strings)
            SaveStrings(fileName,strings);
        end
    end
end