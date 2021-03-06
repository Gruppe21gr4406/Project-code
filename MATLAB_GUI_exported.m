classdef MATLAB_GUI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        ResetButton                   matlab.ui.control.Button
        Image                         matlab.ui.control.Image
        ResultaterLabel               matlab.ui.control.Label
        OplysningerLabel              matlab.ui.control.Label
        CRFvurderingLabel             matlab.ui.control.Label
        Gauge                         matlab.ui.control.SemicircularGauge
        StatusLabel                   matlab.ui.control.Label
        FysiskformEditField           matlab.ui.control.EditField
        DinfysiskeformerLabel         matlab.ui.control.Label
        ACamplitudemodtagetLamp       matlab.ui.control.Lamp
        ACamplitudemodtagetLampLabel  matlab.ui.control.Label
        BluetoothforbundetLamp        matlab.ui.control.Lamp
        BluetoothforbundetLampLabel   matlab.ui.control.Label
        StartoptagelseButton          matlab.ui.control.Button
        CRFEditField                  matlab.ui.control.NumericEditField
        CRFEditFieldLabel             matlab.ui.control.Label
        ACamplitudeEditField          matlab.ui.control.NumericEditField
        ACamplitudeEditFieldLabel     matlab.ui.control.Label
        rLabel                        matlab.ui.control.Label
        MATLABGUILabel                matlab.ui.control.Label
        KnDropDown                    matlab.ui.control.DropDown
        KnDropDownLabel               matlab.ui.control.Label
        NavnEditField                 matlab.ui.control.EditField
        NavnEditFieldLabel            matlab.ui.control.Label
        AlderEditField                matlab.ui.control.NumericEditField
        AlderEditFieldLabel           matlab.ui.control.Label
        ContextMenu                   matlab.ui.container.ContextMenu
        Menu                          matlab.ui.container.Menu
        Menu2                         matlab.ui.container.Menu
    end

    
    properties (Access = private)
        keep_running = true; % Description
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: StartoptagelseButton
        function StartoptagelseButtonPushed(app, event)

%% Tjekker, om patientparametre er indtastet

str = {};
if isempty(app.NavnEditField.Value)
    str{1} = "Navn ikke indtastet";
end
if app.AlderEditField.Value <= 0
    str{2} = "Alder ikke indtastet";
end
if length(str) > 0 %Vi g??r kun ind i dette if-statement, hvis der er manglende informationer/indtastninger
   warndlg(str,'Fejl');
   return; %Afbryder og g??r ikke videre herfra
end

%% Forbinder til Bluetooth
try
    my_BLE_Device = ble("ESP32"); %Fors??ger at oprette Bluetooth-forbindelse til ESP32 
catch e
    disp(e.message);
return
end

%Indl??ser service og characteristic fra ESP32
ServiceUUID = my_BLE_Device.Characteristics.ServiceUUID(5); % Macbook: 1 i stedet for 5 
CharacteristicUUID = my_BLE_Device.Characteristics.CharacteristicUUID(5); % Macbook: 1 i stedet for 5 
c = characteristic(my_BLE_Device,ServiceUUID,CharacteristicUUID);
c.DataAvailableFcn = @displayCharacteristicData;

%MATLAB-funktionen aktiverer notifikationer fra characteristic (c) i BLE-forbindelsen.

subscribe(c)

%G??r 'Bluetooth-lampen' gr??n, hvis computeren er forbundet til ESP32
app.BluetoothforbundetLamp.Color = 'g';

%Sender et 's' til ESP32 for at signalere, at optagelsen er startet
write(c,"s")

while(app.keep_running)
    pause(1); %Tjekker hvert sekund om keep_running har ??ndret sig (true/false).
end
    
%L??ser og printer input fra ESP32

function displayCharacteristicData(src,evt)
[data,timestamp] = read(src,'latest');
AC_amplitude = typecast(uint8(data),'double');
fprintf('data recieved: %f\n', AC_amplitude);
fprintf('%s\n',timestamp);
app.keep_running = false;

AC_amplitude_mg = (AC_amplitude*0.0000038)*1000;

%AC-amplituden printes i tekstboksen og lampen g??res gr??n
app.ACamplitudeEditField.Value = AC_amplitude_mg;
app.ACamplitudemodtagetLamp.Color = 'g';



%SEX = 0 eller 1 (kvinde eller mand)
sex = app.KnDropDown.Value; 
if strcmp(sex,'Kvinde')
    SEX = 0;
elseif strcmp(sex,'Mand')
    SEX = 1;                                     
end

%L??ser og gemmer indtastede informationer
alder = app.AlderEditField.Value;



%Algoritme (Vores egen), som enten runder op eller ned til n??rmeste heltal
CRF = round(0.31644*AC_amplitude_mg+29.5092); 

%Vurdering af fysisk form p?? baggrund af k??n og alder
app.CRFEditField.Value = CRF;

   switch sex
       case 'Kvinde'
        if (alder>=20) && (alder<=29)
            if (CRF<24)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=24) && (CRF<=30)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=31) && (CRF<=37)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=38) && (CRF<=48)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=49)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
                
            
        elseif (alder>=30) && (alder<=39)
            if (CRF<20)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=20) && (CRF<=27)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=28) && (CRF<=33)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=34) && (CRF<=44)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=45)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=40) && (alder<=49)
            if (CRF<17)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=17) && (CRF<=23)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=24) && (CRF<=30)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=31) && (CRF<=41)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=42)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=50) && (alder<=59)
            if (CRF<15)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=15) && (CRF<=20)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=21) && (CRF<=27)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=28) && (CRF<=37)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=38)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=60) && (alder<=69)
            if (CRF<13)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=13) && (CRF<=17)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=18) && (CRF<=23)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=24) && (CRF<=34)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=35)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
        end
        
        case 'Mand'
        if (alder>=20) && (alder<=29)
            if (CRF<25)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=25) && (CRF<=33)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=34) && (CRF<=42)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=43) && (CRF<=52)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=53)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
                
            
        elseif (alder>=30) && (alder<=39)
            if (CRF<23)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=23) && (CRF<=30)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=31) && (CRF<=38)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=39) && (CRF<=48)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=49)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=40) && (alder<=49)
            if (CRF<20)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=20) && (CRF<=26)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=27) && (CRF<=35)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=36) && (CRF<=44)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=45)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=50) && (alder<=59)
            if (CRF<18)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=18) && (CRF<=24)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=25) && (CRF<=33)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=34) && (CRF<=42)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=43)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
            
        elseif (alder>=60) && (alder<=69)
            if (CRF<16)
                app.FysiskformEditField.Value = 'Meget d??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=16) && (CRF<=22)
                app.FysiskformEditField.Value = 'D??rlig';
                app.Gauge.Value = CRF;
            elseif (CRF>=23) && (CRF<=30)
                app.FysiskformEditField.Value = 'Moderat';
                app.Gauge.Value = CRF;
            elseif (CRF>=31) && (CRF<=40)
                app.FysiskformEditField.Value = 'God';
                app.Gauge.Value = CRF;
            elseif (CRF>=41)
                app.FysiskformEditField.Value = 'Meget god';
                app.Gauge.Value = CRF;
            end
        end
   end
end
    

        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            app.ACamplitudemodtagetLamp.Color = 'r';
            app.BluetoothforbundetLamp.Color = 'r';
            app.NavnEditField.Value = ' ';
            app.AlderEditField.Value = 0;
            app.CRFEditField.Value = 0 ;
            app.ACamplitudeEditField.Value = 0 ;
            app.FysiskformEditField.Value = ' ';
            app.Gauge.Value = 0;
            app.keep_running = true;     
            
           
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 623 359];
            app.UIFigure.Name = 'MATLAB App';

            % Create AlderEditFieldLabel
            app.AlderEditFieldLabel = uilabel(app.UIFigure);
            app.AlderEditFieldLabel.HorizontalAlignment = 'right';
            app.AlderEditFieldLabel.Position = [18 185 34 22];
            app.AlderEditFieldLabel.Text = 'Alder';

            % Create AlderEditField
            app.AlderEditField = uieditfield(app.UIFigure, 'numeric');
            app.AlderEditField.Limits = [0 69];
            app.AlderEditField.Position = [78 187 100 22];

            % Create NavnEditFieldLabel
            app.NavnEditFieldLabel = uilabel(app.UIFigure);
            app.NavnEditFieldLabel.HorizontalAlignment = 'right';
            app.NavnEditFieldLabel.Position = [18 250 34 22];
            app.NavnEditFieldLabel.Text = 'Navn';

            % Create NavnEditField
            app.NavnEditField = uieditfield(app.UIFigure, 'text');
            app.NavnEditField.Position = [78 250 100 22];

            % Create KnDropDownLabel
            app.KnDropDownLabel = uilabel(app.UIFigure);
            app.KnDropDownLabel.HorizontalAlignment = 'right';
            app.KnDropDownLabel.Position = [18 218 31 22];
            app.KnDropDownLabel.Text = 'K??n ';

            % Create KnDropDown
            app.KnDropDown = uidropdown(app.UIFigure);
            app.KnDropDown.Items = {'Kvinde', 'Mand'};
            app.KnDropDown.Position = [78 218 100 22];
            app.KnDropDown.Value = 'Kvinde';

            % Create MATLABGUILabel
            app.MATLABGUILabel = uilabel(app.UIFigure);
            app.MATLABGUILabel.BackgroundColor = [0.2627 0.6196 0.6392];
            app.MATLABGUILabel.HorizontalAlignment = 'center';
            app.MATLABGUILabel.FontSize = 18;
            app.MATLABGUILabel.FontWeight = 'bold';
            app.MATLABGUILabel.Position = [1 318 623 42];
            app.MATLABGUILabel.Text = 'MATLAB GUI';

            % Create rLabel
            app.rLabel = uilabel(app.UIFigure);
            app.rLabel.Position = [182 185 25 22];
            app.rLabel.Text = '??r ';

            % Create ACamplitudeEditFieldLabel
            app.ACamplitudeEditFieldLabel = uilabel(app.UIFigure);
            app.ACamplitudeEditFieldLabel.HorizontalAlignment = 'right';
            app.ACamplitudeEditFieldLabel.Position = [433 250 78 22];
            app.ACamplitudeEditFieldLabel.Text = 'AC-amplitude';

            % Create ACamplitudeEditField
            app.ACamplitudeEditField = uieditfield(app.UIFigure, 'numeric');
            app.ACamplitudeEditField.Editable = 'off';
            app.ACamplitudeEditField.Position = [525 250 90 22];

            % Create CRFEditFieldLabel
            app.CRFEditFieldLabel = uilabel(app.UIFigure);
            app.CRFEditFieldLabel.HorizontalAlignment = 'right';
            app.CRFEditFieldLabel.Position = [430 220 30 22];
            app.CRFEditFieldLabel.Text = 'CRF';

            % Create CRFEditField
            app.CRFEditField = uieditfield(app.UIFigure, 'numeric');
            app.CRFEditField.Editable = 'off';
            app.CRFEditField.Position = [526 220 90 22];

            % Create StartoptagelseButton
            app.StartoptagelseButton = uibutton(app.UIFigure, 'push');
            app.StartoptagelseButton.ButtonPushedFcn = createCallbackFcn(app, @StartoptagelseButtonPushed, true);
            app.StartoptagelseButton.BackgroundColor = [1 1 1];
            app.StartoptagelseButton.FontName = 'Euphemia UCAS';
            app.StartoptagelseButton.Position = [42 96 160 49];
            app.StartoptagelseButton.Text = 'Start optagelse';

            % Create BluetoothforbundetLampLabel
            app.BluetoothforbundetLampLabel = uilabel(app.UIFigure);
            app.BluetoothforbundetLampLabel.HorizontalAlignment = 'right';
            app.BluetoothforbundetLampLabel.Position = [238 250 113 22];
            app.BluetoothforbundetLampLabel.Text = 'Bluetooth forbundet';

            % Create BluetoothforbundetLamp
            app.BluetoothforbundetLamp = uilamp(app.UIFigure);
            app.BluetoothforbundetLamp.Position = [359 250 20 20];
            app.BluetoothforbundetLamp.Color = [1 0 0];

            % Create ACamplitudemodtagetLampLabel
            app.ACamplitudemodtagetLampLabel = uilabel(app.UIFigure);
            app.ACamplitudemodtagetLampLabel.HorizontalAlignment = 'right';
            app.ACamplitudemodtagetLampLabel.Position = [217 217 135 22];
            app.ACamplitudemodtagetLampLabel.Text = 'AC-amplitude modtaget';

            % Create ACamplitudemodtagetLamp
            app.ACamplitudemodtagetLamp = uilamp(app.UIFigure);
            app.ACamplitudemodtagetLamp.Position = [359 218 20 20];
            app.ACamplitudemodtagetLamp.Color = [1 0 0];

            % Create DinfysiskeformerLabel
            app.DinfysiskeformerLabel = uilabel(app.UIFigure);
            app.DinfysiskeformerLabel.HorizontalAlignment = 'center';
            app.DinfysiskeformerLabel.Position = [439 46 137 22];
            app.DinfysiskeformerLabel.Text = 'Din fysiske form er:';

            % Create FysiskformEditField
            app.FysiskformEditField = uieditfield(app.UIFigure, 'text');
            app.FysiskformEditField.HorizontalAlignment = 'center';
            app.FysiskformEditField.Position = [438 21 138 22];

            % Create StatusLabel
            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.FontWeight = 'bold';
            app.StatusLabel.Position = [294 286 42 22];
            app.StatusLabel.Text = 'Status';

            % Create Gauge
            app.Gauge = uigauge(app.UIFigure, 'semicircular');
            app.Gauge.Limits = [0 70];
            app.Gauge.MajorTicks = [0 10 20 30 40 50 60 70];
            app.Gauge.ScaleColors = [0.48 0.76 0.65;0.48 0.76 0.65;0.48 0.76 0.65;0.48 0.76 0.65;0.48 0.76 0.65];
            app.Gauge.ScaleColorLimits = [49 70;40 49;33 40;24 33;0 24];
            app.Gauge.Position = [413 81 188 102];

            % Create CRFvurderingLabel
            app.CRFvurderingLabel = uilabel(app.UIFigure);
            app.CRFvurderingLabel.FontWeight = 'bold';
            app.CRFvurderingLabel.Position = [460 188 89 22];
            app.CRFvurderingLabel.Text = 'CRF-vurdering';

            % Create OplysningerLabel
            app.OplysningerLabel = uilabel(app.UIFigure);
            app.OplysningerLabel.FontWeight = 'bold';
            app.OplysningerLabel.Position = [91 286 74 22];
            app.OplysningerLabel.Text = 'Oplysninger';

            % Create ResultaterLabel
            app.ResultaterLabel = uilabel(app.UIFigure);
            app.ResultaterLabel.FontWeight = 'bold';
            app.ResultaterLabel.Position = [471 286 65 22];
            app.ResultaterLabel.Text = 'Resultater';

            % Create Image
            app.Image = uiimage(app.UIFigure);
            app.Image.Position = [234 26 157 135];
            app.Image.ImageSource = 'running_man.png';

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.BackgroundColor = [1 1 1];
            app.ResetButton.FontName = 'Euphemia UCAS';
            app.ResetButton.Position = [72 38 100 30];
            app.ResetButton.Text = 'Reset';

            % Create ContextMenu
            app.ContextMenu = uicontextmenu(app.UIFigure);

            % Create Menu
            app.Menu = uimenu(app.ContextMenu);
            app.Menu.Text = 'Menu';

            % Create Menu2
            app.Menu2 = uimenu(app.ContextMenu);
            app.Menu2.Text = 'Menu2';
            
            % Assign app.ContextMenu
            app.BluetoothforbundetLamp.ContextMenu = app.ContextMenu;
            app.Gauge.ContextMenu = app.ContextMenu;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = MATLAB_GUI_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
