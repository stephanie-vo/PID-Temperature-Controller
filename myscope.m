
%-----------------------------------------------------------------------
% myscope.m
% Written by Kenrick Chin
% Date: 2016 Jan 29
% Modified by Mohammadreza Shahzadeh
%-----------------------------------------------------------------------
function varargout = myscope(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myscope_OpeningFcn, ...
                   'gui_OutputFcn',  @myscope_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
%-----------------------------------------------------------------------
% --- Executes just before myscope is made visible.
function myscope_OpeningFcn(hObject, eventdata, handles, varargin)
global comport;
global RUN;
global NPOINTS;
RUN = 0;
NPOINTS = 400;
comport = serial('COM7','BaudRate',115200);
fopen(comport);
% Choose default command line output for myscope1
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
%-----------------------------------------------------------------------
function myscope_OutputFcn(hObject, eventdata, handles, varargin)
%-----------------------------------------------------------------------
function Quit_Button_Callback(hObject, eventdata, handles)
global comport
global RUN
RUN = 0;
fclose(comport);
clear comport
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
% use close to terminate your program
% use quit to terminate MATLAB
close
%----------------------------------------------------------------------
function Run_Button_Callback(hObject, eventdata, handles)
global comport
global NPOINTS
global RUN
global K_Term
global InputFlag
global TargetTemp
global Max_K_Term

%Definitions
 Vs=3.3;R_divider=[1e3 0]; %Supply voltage and voltage divider resistances -Fixed then Thermistor (Ohms)
  %{
      %Run this for different laptops and NPOINTS values
      %Acquires the average Delta_T for any NPOINTS value
       Delta_tLst=zeros(1,200);
       for i=1:200
           %if comport.BytesAvailable>0
        % send a single character prompt to the MCU
        fprintf(comport,"%s",'A');
   
        % fetch thermistor voltage as single 8-bit bytes
        tic
        fread(comport, NPOINTS, 'uint8')*(Vs/255);
        Delta_tLst(i)=toc;
       end
       delta_T=mean(Delta_tLst)
  %}
 Delta_t=0.059; %sampling period to evaluate a batch of NPOINTS
 InputFlag=zeros(1,4); %Sets the initial value of the flags in the K-terms callback functions
 K_Term=[915,1.5,4];%K-values [Kp,Ki,Kd]
 Max_K_Term=[2000 100 100]; % Maximum K-values [Kp,Ki,Kd]
 PID=[0 0 0 0 0 0]; %P,I,D,PID output,Min PID output, and Max PID output values respectively
 
 TargetTemp=20; SamplesCounter=0; TimeCap=10*fix(60/Delta_t);%TimeCap in mins
 Lag=5;%Time period of integration and differentiation from the current time.
 recall=Lag/Delta_t; %recall=fix(Lag/Delta_t-1);
 InstantTherm=zeros(1,NPOINTS); %Holding variable for converting each bach of NPOINTS voltage samples
 T_therm=zeros(1,TimeCap); %Stores the average of a batch of NPOINTS temperature samples.
 errLst=zeros(1,TimeCap);%list of error terms
 TargetTempLst=zeros(1,TimeCap);
 Timer=zeros(1,TimeCap); %Increments the time-axis by 5s intervals of NPOINTS sampling period, capped at 10 mins.

%Loop
if RUN == 0
  RUN = 1;
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')

  while RUN == 1 % ADD YOUR CODE HERE. 
      
        % send a single character prompt to the MCU
        fprintf(comport,"%s",'A');

        % fetch thermistor voltage as single 8-bit bytes
         V_therm = (fread(comport, NPOINTS, 'uint8'))*(Vs/256);
        
    for i=1:NPOINTS
        % Thermistor resistance (Kilo-Ohms)
        R_divider(2)=V_therm(i)/(Vs-V_therm(i)+1/256)*R_divider(1)*1e-3;
        
        %Polynomial Interpolation equation from thermistor data
        R_therm=@(T)-8.88889e-13*T^8+1.24444e-10*T^7-6e-9*T^6+2.44444e-8*T^5+1.25617e-5*T^4-9.48022e-4*T^3+0.0466097*T^2-1.63812*T+32.33-R_divider(2);
        InstantTherm(i)=fzero(R_therm,20); %Instantanous sample of thermistor temp 
        
    end
    
    %Clears the temperature and error histories after the time limit
    
    if SamplesCounter == TimeCap
        %Takes a batch of the latest values,making those the first batch in the new lists
        FinalValues=zeros(3,recall);
        FinalValues(1,:)=T_therm(end-recall+1:end);
        FinalValues(2,:)=TargetTempLst(end-recall+1:end);
        FinalValues(3,:)=errLst(end-recall+1:end);
        
        %Reset temperature and error histories
        [T_therm,TargetTempLst,errLst]=deal(zeros(1,TimeCap));
        T_therm(1:recall)=FinalValues(1,:);
        TargetTempLst(1:recall)=FinalValues(2,:);
        errLst(1:recall)=FinalValues(3,:);
        %1 less than "Lag" # of samples to account for the current error value.
        SamplesCounter=recall;
    end
    
    SamplesCounter=SamplesCounter+1; %Counts the number of times the MSP took NPOINTS from the thermistor
    T_therm(SamplesCounter)=mean(InstantTherm); %reduce noise fluctuations
    TargetTempLst(SamplesCounter)=TargetTemp;% History of target temperatures.
  
    % plot the Thermistor and Target temperature on the GUI
    Timer(SamplesCounter)=SamplesCounter*Delta_t; 
    plot(Timer(1:SamplesCounter),T_therm(1:SamplesCounter));
    hold on
    plot(Timer(1:SamplesCounter),TargetTempLst(1:SamplesCounter));
    hold off
    
    xlabel('Time (s)');
    ylabel('Temperature (C)');
    ylim ([-5 60]);
    % use drawnow to update the figure
    drawnow  
    
    %PID Section
    currentTemp=T_therm(SamplesCounter);
    set(handles.Current_Temp_Display,'string',currentTemp);%Updates current temp in GUI
    errLst(SamplesCounter)=currentTemp-TargetTemp; %Error term
    set(handles.Kp_Display,'string',K_Term(1));%Updates Kp-term in GUI
    set(handles.Ki_Display,'string',K_Term(2));%Updates Ki-term in GUI
    set(handles.Kd_Display,'string',K_Term(3));%Updates Kd-term in GUI
    
    %P-term
    PID(1)=K_Term(1)*errLst(SamplesCounter);
    set(handles.P_Display,'string',PID(1));%Updates P-term in GUI
    
    %I-term
    if SamplesCounter == 1 %Doesn't integrate on the first iteration 
            continue
    else 
    PID(2)=K_Term(2)*sum(errLst(1:SamplesCounter))*Delta_t;
    %set(handles.I_Display,'string',PID(2));%Updates I-term in GUI
    end
       %Doubles Kd during cooling cycle
    if SamplesCounter > 1 && errLst(SamplesCounter)>0
        PID(2)=2*K_Term(2)*sum(errLst(1:SamplesCounter))*Delta_t;
    end
    set(handles.I_Display,'string',PID(2));%Updates I-term in GUI

    %D-term
    if SamplesCounter == 1 %Doesn't integrate on the first iteration 
            continue
    else 
        PID(3)=K_Term(3)*(errLst(SamplesCounter)-errLst(SamplesCounter-1))/Delta_t;
    end
    set(handles.D_Display,'string',PID(3));%Updates D-term in GUI
    
    PID(4)=sum(PID(1:3));%PID output
    %Only uses local PID max or min for every new target temperature set
    if TargetTempLst(SamplesCounter)-TargetTempLst(SamplesCounter-1)~=0
        if PID(4)>0
            PID(6)=PID(4);
        elseif PID(4)<0
            PID(5)=PID(4);
        end
    end
    
    %For cooling
    if PID(4)>0
        %Ensures max PID output is ATLEAST equal to the current one
        if PID(4)>PID(6)
           PID(6)=PID(4);
        end
        MCU=fix(127-(PID(4)/PID(6)*127)); %0 means largest cooling duty cycle
    end
    %For heating
    if PID(4)<0
        %Ensures min PID output is ATMOST equal to the current one
        if PID(4)<PID(5)
           PID(5)=PID(4);
        end
        MCU=fix(128+(PID(4)/PID(5)*127)); %255 means largest heating duty cycle
    end
    set(handles.MCU_Display,'string',MCU);
    fprintf(comport,'%c',MCU);
    
  end
else
  RUN = 0;
  % change the string on the button to RUN
  set(handles.Run_Button,'string','RUN');

end

function Kp_Input_Callback(hObject, eventdata, handles)
global K_Term;
global InputFlag;
global Max_K_Term;
Kp_Value_hold = str2double(get(hObject,'String'));
%Checks if the input is a number within the required range.
if isnumeric(Kp_Value_hold)
    if all(Kp_Value_hold >=0  & Kp_Value_hold <= Max_K_Term(1))
        InputFlag(1)=1;
    else
        disp("Input Kp term exceeds range of operation");
    end
end
if InputFlag(1)==1 
    K_Term(1) = Kp_Value_hold;
    InputFlag(1)=0;
end

% hObject    handle to Kp_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kp_Input as text
%        str2double(get(hObject,'String')) returns contents of Kp_Input as a double


% --- Executes during object creation, after setting all properties.
function Kp_Input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kp_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ki_Input_Callback(hObject, eventdata, handles)
global K_Term;
global InputFlag;
global Max_K_Term
Ki_Value_hold = str2double(get(hObject,'String'));
%Checks if the input is a number within the required range.
if isnumeric(Ki_Value_hold)
    if all(Ki_Value_hold >=0  & Ki_Value_hold <= Max_K_Term(2))
        InputFlag(2)=1;
    else
        disp("Input Ki term exceeds range of operation");
    end
end
if InputFlag(2)==1 
    K_Term(2) = Ki_Value_hold;
    InputFlag(2)=0;
end
% hObject    handle to Ki_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ki_Input as text
%        str2double(get(hObject,'String')) returns contents of Ki_Input as a double


% --- Executes during object creation, after setting all properties.
function Ki_Input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ki_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Kd_Input_Callback(hObject, eventdata, handles)
global K_Term;
global InputFlag;
global Max_K_Term
Kd_Value_hold = str2double(get(hObject,'String'));
%Checks if the input is a number within the required range.
if isnumeric(Kd_Value_hold)
    if all(Kd_Value_hold >=0  & Kd_Value_hold <=  Max_K_Term(3))
        InputFlag(3)=1;
    else
        disp("Input Kd term exceeds range of operation");
    end
end
if InputFlag(3)==1 
    K_Term(3) = Kd_Value_hold;
    InputFlag(3)=0;
end
% hObject    handle to Kd_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kd_Input as text
%        str2double(get(hObject,'String')) returns contents of Kd_Input as a double


% --- Executes during object creation, after setting all properties.
function Kd_Input_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kd_Input (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Target_Temperature_Callback(hObject, eventdata, handles)
global TargetTemp;
global InputFlag;
HoldTargetTemp=str2double(get(hObject,'String'));
%Checks if the input is a number within the allowed temperature range.
if isnumeric(HoldTargetTemp)
    if all(HoldTargetTemp >= 0 & HoldTargetTemp <= 45)
        InputFlag(4)=1;
    else
        disp("Input temperature exceeds range of operation");
    end
end
if InputFlag(4)==1 
            TargetTemp = HoldTargetTemp;
            InputFlag(4)=0;
end
% hObject    handle to Target_Temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Target_Temperature as text
%        str2double(get(hObject,'String')) returns contents of Target_Temperature as a double


% --- Executes during object creation, after setting all properties.
function Target_Temperature_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Target_Temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
