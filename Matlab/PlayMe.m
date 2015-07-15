function varargout = PlayMe(varargin)
% PLAYME MATLAB code for PlayMe.fig
%      PLAYME, by itself, creates a new PLAYME or raises the existing
%      singleton*.
%
%      H = PLAYME returns the handle to a new PLAYME or the handle to
%      the existing singleton*.
%
%      PLAYME('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLAYME.M with the given input arguments.
%
%      PLAYME('Property','Value',...) creates a new PLAYME or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PlayMe_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PlayMe_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PlayMe

% Last Modified by GUIDE v2.5 19-Jul-2011 15:11:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PlayMe_OpeningFcn, ...
                   'gui_OutputFcn',  @PlayMe_OutputFcn, ...
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

% --- Executes just before PlayMe is made visible.
function PlayMe_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PlayMe (see VARARGIN)

% Initial command and specimen parameters
User_c = [ 0; 0; 0];
Specimen_p  = [ 0; 0; 0];
handles.Data_P = [0 0 0 0];

% set(handles.Table,'Data',num2cell([0;0;0;0;0;0;0;20;1]))
Table = get(handles.Table,'Data'); % Data from table
handles.Data_Angles = Table{7}:Table{9}:Table{8}; % initial set of angles

% Save the arrays for both command and specimen parameters to a common cell
% array "Command" in the handles structure 
handles.Data_Command = num2cell([User_c; Specimen_p]);

% Create a figure to show the test view plot and store it to handles
handles.Plot_TestView = figure('Name','Test View'); 

% Create a figure to show an animation of the simulation
handles.Plot_Movie = figure('Name','Animation');
position = get(gcf,'Position'); position(3:4) = 1.5*position(3:4);
set(gcf,'Position',position)

% Create a figure to show the theta2theta scan and store it to handles 
handles.Plot_Theta2theta = figure('Name','Theta2theta scan');

% Create a figure to show the distance scan and store it to handles 
handles.Plot_Distance = figure('Name','Distance');

% Update figures and movie
handles=lf_SimPlot(handles);

% Choose default command line output for PlayMe
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = PlayMe_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure

varargout{1} = handles.output;


% UI callbacks ____________________________________________________________

% --- Executes on button press in Button_FeatureSpace.
function Button_FeatureSpace_Callback(hObject, eventdata, handles)
% hObject    handle to Button_FeatureSpace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Button_FeatureSpace

% If the user turned the button on and the handle does not exist
if get(hObject,'Value') && ~ishandle(handles.Plot_FeatureSpace)
    handles.Plot_FeatureSpace = figure('Name','Feature Space'); % recreate handle
    handles=lf_SimPlot(handles);
% If the user turned off the button and the handle exists then close the fig
elseif ~get(hObject,'Value') && ishandle(handles.Plot_FeatureSpace)
    close(handles.Plot_FeatureSpace)
end

% --- Executes on button press in Button_Theta2theta.
function Button_Theta2theta_Callback(hObject, eventdata, handles)
% hObject    handle to Button_Theta2theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Button_Theta2theta

% If the user turned the button on and the handle does not exist
if get(hObject,'Value') && ~ishandle(handles.Plot_Theta2theta)
    handles.Plot_Theta2theta = figure('Name','Test View'); % recreate handle
    handles=lf_SimPlot(handles);
% If the user turned off the button and the handle exists then close the fig
elseif ~get(hObject,'Value') && ishandle(handles.Plot_Theta2theta)
    close(handles.Plot_Theta2theta)
end

% --- Executes on button press in Button_Distance.
function Button_Distance_Callback(hObject, eventdata, handles)
% hObject    handle to Button_Distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Button_Distance

% If the user turned the button on and the handle does not exist
if get(hObject,'Value') && ~ishandle(handles.Plot_Distance)
    handles.Plot_Distance = figure('Name','Test View'); % recreate handle
    handles=lf_SimPlot(handles);
% If the user turned off the button and the handle exists then close the fig
elseif ~get(hObject,'Value') && ishandle(handles.Plot_Distance)
    close(handles.Plot_Distance)
end


% --- Executes on button press in Button_TestView.
function Button_TestView_Callback(hObject, eventdata, handles)
% hObject    handle to Button_TestView (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Button_TestView
% If the user turned the button on and the handle does not exist
if get(hObject,'Value') && ~ishandle(handles.Plot_TestView)
    handles.Plot_TestView = figure('Name','Test View'); % recreate handle
    handles=lf_SimPlot(handles);
% If the user turned off the button and the handle exists then close the fig
elseif ~get(hObject,'Value') && ishandle(handles.Plot_TestView)
    close(handles.Plot_TestView)
end

% --- Executes on button press in Button_Movie.
function Button_Movie_Callback(hObject, eventdata, handles)
% hObject    handle to Button_Movie (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Button_Movie

% If the user turned the button on and the handle does not exist
if get(hObject,'Value') && ~ishandle(handles.Plot_Movie)
    handles.Plot_Movie = figure('Name','Test View'); % recreate handle
    position = get(gcf,'Position'); position(3:4) = 1.5*position(3:4);
    set(gcf,'Position',position)
    lf_Movie(handles);
% If the user turned off the button and the handle exists then close the fig
elseif ~get(hObject,'Value') && ishandle(handles.Plot_Movie)
    close(handles.Plot_Movie)
end

% --- Executes on slider movement.
function Cx_Callback(hObject, eventdata, handles)
% hObject    handle to Cx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Cx
Cx = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{1} = Cx;

% Update the table
Table = get(handles.Table,'Data');
Table{1} = Cx;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on slider movement.
function Cz_Callback(hObject, eventdata, handles)
% hObject    handle to Cz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Sx
Cz = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{2} = Cz;

% Update the table
Table = get(handles.Table,'Data');
Table{2} = Cz;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on slider movement.
function Ctheta_Callback(hObject, eventdata, handles)
% hObject    handle to Ctheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Ctheta
Ctheta = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{3} = Ctheta;

% Update the table
Table = get(handles.Table,'Data');
Table{3} = Ctheta;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);


% --- Executes on slider movement.
function Sx_Callback(hObject, eventdata, handles)
% hObject    handle to Sx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Sx
Sx = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{4} = Sx;

% Update the table
Table = get(handles.Table,'Data');
Table{4} = Sx;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on slider movement.
function Sz_Callback(hObject, eventdata, handles)
% hObject    handle to Sz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Sz
Sz = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{5} = Sz;

% Update the table
Table = get(handles.Table,'Data');
Table{5} = Sz;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on slider movement.
function Stheta_Callback(hObject, eventdata, handles)
% hObject    handle to Stheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Get the value of Stheta
Stheta = get(hObject,'Value');

% Change the Command cell 
handles.Data_Command{6} = Stheta;

% Update the table
Table = get(handles.Table,'Data');
Table{6} = Stheta;
set(handles.Table,'Data',Table);

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);


% --- Executes when entered data in editable cell(s) in Table.
function Table_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to Table (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% Find which one changed (only want the row)
ind = eventdata.Indices(1);

% Update Command
if ind < 7
handles.Data_Command{ind} = str2double(eventdata.EditData);
end

% Update the slider
if ind==1
    set(handles.Cx,'Value',eventdata.NewData)
elseif ind==2
    set(handles.Cz,'Value',eventdata.NewData)
elseif ind==3
    set(handles.Ctheta,'Value',eventdata.NewData)
elseif ind==4
    set(handles.Sx,'Value',eventdata.NewData)
elseif ind==5
    set(handles.Sz,'Value',eventdata.NewData)
elseif ind==6
    set(handles.Stheta,'Value',eventdata.NewData)
elseif ind > 6
    Table = get(handles.Table,'Data'); 
    handles.Data_Angles = Table{7}:Table{9}:Table{8}; 
end

% Simulate and plot
handles=lf_SimPlot(handles);

% Update handles structure
guidata(hObject, handles);

% Local functions (that I made) ___________________________________________

% function to simulate and plot
function handles = lf_SimPlot(handles)

% Break apart Command
User_c = cell2mat(handles.Data_Command(1:3));
Specimen_p = cell2mat(handles.Data_Command(4:6));

% Simulate
[T_Cell L_Cell] = f_Simulate(User_c,Specimen_p,2*User_c(3));

% If the button is down and the handle exists
if get(handles.Button_TestView,'Value') && ishandle(handles.Plot_TestView)
    % Render
    set(0,'CurrentFigure',handles.Plot_TestView); clf;
    f_Render(T_Cell,L_Cell,1)
end

% Simulate the scan
[I d] = f_Theta2theta(User_c,Specimen_p,handles.Data_Angles);

% Theta2theta scan
if get(handles.Button_Theta2theta,'Value') && ishandle(handles.Plot_Theta2theta)
    % Plot the theta2theta scan
    set(0,'CurrentFigure',handles.Plot_Theta2theta); cla
    plot(handles.Data_Angles,I,'r.-'); grid on; axis auto
    title('theta2theta scan','FontSize',16);
    xlabel('Specimen angle','FontSize',16);
    ylabel('Intensity','FontSize',16)
end

% Distance 
if get(handles.Button_Distance,'Value') && ishandle(handles.Plot_Distance)
    % Distance plot
    set(0,'CurrentFigure',handles.Plot_Distance); cla
    plot(handles.Data_Angles,d); grid on
    title('Distance from beam to sensor','FontSize',16);
    xlabel('Specimen angle','FontSize',16);
    ylabel('Distance (cm)','FontSize',16);
end

% Animation
if get(handles.Button_Movie,'Value') && ishandle(handles.Plot_Movie)
    lf_Movie(handles)
end


% function to show a movie
function lf_Movie(handles)

% Number of angles to look at
n = length(handles.Data_Angles);

% Create an array of angles that are in radians
angles = handles.Data_Angles;

% set the movie figure to be the current figure (without moving it to the
% front of the other figures)
set(0,'CurrentFigure',handles.Plot_Movie); clf

% Break apart Command
User_c = cell2mat(handles.Data_Command(1:3));
Specimen_p = cell2mat(handles.Data_Command(4:6));

% preallocate
F(n) = struct('cdata',[],'colormap',[]);

for i=1:n
    % Rotate specimen to this angle
    User_c(3) = angles(i);
    
    % Simulate
    [T_Cell L_Cell] = f_Simulate(User_c,Specimen_p,2*User_c(3));
    
    % Render
    f_Render(T_Cell,L_Cell,i); 
    
    % Take a snapshot
    F(i) = getframe(handles.Plot_Movie); cla
end

axes('Position',[0 0 1 1])
movie(F);

movie2avi(F, 'SimMovie.avi', 'compression', 'None','FPS',2);


% Unnecessary CreateFcns __________________________________________________

% --- Executes during object creation, after setting all properties.
function Table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function Cx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Cz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Ctheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ctheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Sx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Sz_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Sz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function Stheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Stheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

close all
