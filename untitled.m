
function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 18-Jun-2021 17:15:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @untitled_OpeningFcn, ...
    'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)         %%界面初始化函数
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA) 包括界面内所有元素的结构体
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled

axes(handles.screen);
t = linspace(0,2*pi,100);
a = size(t);
r = 1e5*ones(a);
polar(t,r,'r');
cname = string();
cname(1) = '俯仰角（°）';cname(2) = '方位角（°）';
rname = '目标';
cname2 = string();
cname2(1) = '直线距离（m）';cname2(2) = 'x方位（m）';cname2(3) = 'y方位（m）';cname2(4) = 'z方位（m）';
set(handles.display,'ColumnName',cname);
set(handles.display,'RowName',rname);
set(handles.dis_display,'RowName',rname);
set(handles.dis_display,'ColumnName',cname2);
set(handles.now_mode,'String','当前工作模式');
set(handles.pit,'String','当前雷达波束俯仰角');
set(handles.yaw,'String','当前雷达波束方位角');
Pos_Aircraft=[0;0;0];
Phi = [8 3 1 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
handles.pos = [ Pos_Aircraft+ 5e4*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 4e4*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e4*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];                   % 目标4
handles.nowmode = 1;        %初始化为STT模式
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);          %更新handles，不更新则上面创建的handles.xxxx不会被保存

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start_button.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%按下开始后的事件
function start_button_Callback(hObject, eventdata, handles)

% hObject    handle to start_button (see GCBO)                      %当前控件句柄
% eventdata  reserved - to be defined in a future version of MATLAB
Beam_pit = [10 5 0 -5 -10]    ;       % 俯仰维扫描波束指向角  %%%4个角度
Bean_azi = [-6:6]*5       ;         % 方位维扫描波束指向角  %%%13个方位角 -60 —— +60
now_mode = handles.nowmode;
V_Aircraft=[1000;3000;2000]    ;	% 载机速度矢量【m/s】
Pos_Aircraft=[0;0;0]        ;   % 载机三维坐标【m】
% 起始目标角度  这里的角度是相对飞机而言的
Phi = [8 3 1 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
%%%%
Pos_target = handles.pos;                 

axes(handles.screen);
t = linspace(0,2*pi,100);
a = size(t);
set(handles.now_mode,'String','当前工作模式：TWS');
tas_mode = 1;
stt_mode = 1;
while(1)
    if(now_mode == 1)
        set(handles.now_mode,'String','当前工作模式：TWS');
        tar_find = 0;
        tar_find_ang = zeros(2,1);  %记录已发现目标的角度
        tar_find_dis = zeros(4,1);  %记录已发现目标的距离
        for pit = 1:5
            for yaw = 1:13
                now_pit = Beam_pit(pit);
                now_yaw = Bean_azi(yaw);
                yaw_string = ['当前雷达波束方位角',num2str(now_yaw),'°'];
                pit_string = ['当前雷达波束俯仰角',num2str(now_pit),'°'];
                set(handles.pit,'String',pit_string);
                set(handles.yaw,'String',yaw_string);
                [tar_num,tar_ang,tar_dis,R_max,Pos_Air,Pos_tar,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode);
                r = R_max/4*ones(a);
                polar(t,r,'r')
                hold on
                polar([(now_yaw - 5)*pi/180,(now_yaw - 5)*pi/180],[0,R_max/4],'b')
                polar([(now_yaw + 5)*pi/180,(now_yaw + 5)*pi/180],[0,R_max/4],'b')
                
                
                Phi = real_ang(2,:);
                Sita = real_ang(1,:);
                tar_find_ang = tar_ang;
                tar_find_dis = tar_dis;
                Pos_Aircraft = Pos_Air;
                Pos_target = Pos_tar;
                %disp('start')
                m = tar_num;
                tar_find = tar_num;
                rname = string();
                for kk = 1:m
                    rname(kk) = ['目标',num2str(kk)];
                end
                set(handles.display,'Data',tar_ang');
                set(handles.display,'RowName',rname)
                set(handles.dis_display,'Data',tar_dis');
                set(handles.dis_display,'RowName',rname)
                for ii = 1:tar_num
                    r = tar_dis(1,ii);
                    polar(tar_ang(2,ii)*pi/180,r,'rh');
                end
                hold off
                pause(1);
                
            end
        end
    elseif(now_mode == 2)
        set(handles.now_mode,'String','当前工作模式：TAS');
        CPI_num = 4;
        if(tas_mode == 1)
            tar_find = 0;
            tar_find_ang = zeros(2,1);  %记录已发现目标的角度
            tar_find_dis = zeros(4,1);  %记录已发现目标的距离;
            for pit = 2:2
                for yaw = 1:13
                    now_pit = Beam_pit(pit);
                    now_yaw = Bean_azi(yaw);
                    [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,2);
                    yaw_string = ['当前雷达波束方位角',num2str(now_yaw),'°'];
                    pit_string = ['当前雷达波束俯仰角',num2str(now_pit),'°'];
                    set(handles.pit,'String',pit_string);
                    set(handles.yaw,'String',yaw_string);
                    r = R_max/4*ones(a);
                    polar(t,r,'r')
                    hold on
                    polar([(now_yaw - 5)*pi/180,(now_yaw - 5)*pi/180],[0,R_max/4],'b')
                    polar([(now_yaw + 5)*pi/180,(now_yaw + 5)*pi/180],[0,R_max/4],'b')
                    tar_find_ang = tar_ang;
                    tar_find_dis = tar_dis;
                    Phi = real_ang(2,:);
                    Sita = real_ang(1,:);
                    m = tar_num;
                    tar_find = tar_num;
                    rname = string();
                    for kk = 1:m
                        rname(kk) = ['目标',num2str(kk)];
                    end
                    set(handles.display,'Data',tar_ang');
                    set(handles.display,'RowName',rname)
                    set(handles.dis_display,'Data',tar_dis');
                    set(handles.dis_display,'RowName',rname)
                    for ii = 1:tar_num
                        r = tar_dis(1,ii);
                        polar(tar_ang(2,ii)*pi/180,r,'rh');
                    end
                    hold off
                    pause(0.1);
                end
            end    
            tas_mode = 2;
            tar_total_num = tar_num;
            
        elseif(tas_mode == 2)
            for tar_c = 1:tar_total_num
                now_pit = tar_find_ang(1,tar_c);
                now_yaw = tar_find_ang(2,tar_c);
                tar_dis_rec = zeros(3,CPI_num);
                total_data = zeros(2,3,CPI_num);
                for kk = 1:CPI_num
                    [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,tar_c);
                    tar_dis_rec(:,kk) = tar_dis(2:4,tar_c);
                    now_pit = tar_ang(1);now_yaw = tar_ang(2);
                    Phi = real_ang(2,:);
                    Sita = real_ang(1,:);
                    tar_find = tar_num;
                    tar_find_dis(:,tar_c) = tar_dis(:,tar_c);
                    tar_find_ang(:,tar_c) = tar_ang;
                    yaw_string = ['当前雷达波束方位角',num2str(now_yaw),'°'];
                    pit_string = ['当前雷达波束俯仰角',num2str(now_pit),'°'];
                    set(handles.pit,'String',pit_string);
                    set(handles.yaw,'String',yaw_string);
                    r = R_max/4*ones(a);
                    polar(t,r,'r')
                    hold on
                    polar([(now_yaw - 5)*pi/180,(now_yaw - 5)*pi/180],[0,R_max/4],'b')
                    polar([(now_yaw + 5)*pi/180,(now_yaw + 5)*pi/180],[0,R_max/4],'b')
                    rname = ['目标',num2str(tar_c)];
                    set(handles.display,'Data',tar_ang');
                    set(handles.display,'RowName',rname)
                    set(handles.dis_display,'Data',tar_dis(:,tar_c)');
                    set(handles.dis_display,'RowName',rname)
                    r = tar_dis(1,tar_c);
                    polar(tar_ang(2)*pi/180,r,'rh');
                    hold off
                    pause(0.1)
                end
                clc
                getd = @(p)path(path,p);
                getd('fire_control_subpro/');
                x_soft = zeros(3,CPI_num);
                for kk = 1:3
                    x_soft(kk,:) = soft_fil(tar_dis_rec(kk,:),8);
                end
                total_data(1,:,:) = x_soft;
                %增加虚警点
                false_a = zeros(1,3,CPI_num);
                for kk = 1:1
                    [x1] = find(abs(total_data(kk,1,:)) == max(max(max(abs(total_data(kk,1,:))))));
                    [x2] = find(abs(total_data(kk,2,:)) == max(max(max(abs(total_data(kk,2,:))))));
                    [x3] = find(abs(total_data(kk,3,:)) == max(max(max(abs(total_data(kk,3,:))))));
                    false_a(kk,1,:) = false_a(kk,1,:) + sign(total_data(kk,1,x1(1)))*(rand(size(false_a(kk,1,:)))+0.1)*max(max(abs(total_data(kk,1,:))))*2;
                    false_a(kk,2,:) = false_a(kk,2,:) + sign(total_data(kk,2,x2(1)))*(rand(size(false_a(kk,2,:)))+0.1)*max(max(abs(total_data(kk,2,:))))*2;
                    false_a(kk,3,:) = false_a(kk,3,:) + sign(total_data(kk,3,x3(1)))*(rand(size(false_a(kk,3,:)))+0.1)*max(max(abs(total_data(kk,3,:))))*2;
                    total_data(2,:,:) = false_a(1,:,:);
                end
                
        %{        
                figure;hold on
                for kk = 1:CPI_num
                    plot3(squeeze(total_data(:,1,kk)),squeeze(total_data(:,2,kk)),squeeze(total_data(:,3,kk)),'.','color',[rand rand rand])
                    drawnow;
                end
                xlabel('x');ylabel('y');zlabel('z');
          %}      
                %% 跟踪滤波
                % 跟踪参数
                dis_door = 10;                           %距离门大小
                if_rock = zeros(1,CPI_num);       %是否跟踪到
                lose_num = 0;                           %跟丢累计数
                pre_num = 5;                           %预测参考数
                
                tra_pro = zeros(3,CPI_num);       %跟踪结果
                start_point = total_data(1,:,1);        %跟踪起点确认
                tra_pro(:,1) = start_point;
                for ii = 2:CPI_num
                    for kk = 1:size(total_data,1)
                        now = total_data(kk,:,ii);                  %当前判断点
                        sigma = norm(abs(now' - tra_pro(:,ii-1)));        %算三个距离总共平方差
                        if(sigma < dis_door)
                            tra_pro(:,ii) = total_data(kk,:,ii);    %确定点
                            if_rock(ii) = 1;
                            lose_num = 0;
                            break
                        end
                    end
                    if(if_rock(ii)==0)
                        if(ii > pre_num)
                            tra_pro(:,ii) = my_predict(tra_pro(:,ii-pre_num:ii-1));
                        elseif(ii > 2 && ii < pre_num)
                            tra_pro(:,ii) = my_predict(tra_pro(:,1:ii-1));
                        else
                            tra_pro(:,ii) = tra_pro(:,ii-1);
                        end
                        lose_num = lose_num + 1;
                    end
                    if(lose_num > 2)
                        disp('lose')
                        break
                    end
                end
              %{ 
                figure;hold on
                for kk = 1:CPI_num
                    plot3(tra_pro(1,kk),tra_pro(2,kk),tra_pro(3,kk),'.')
                    %drawnow
                end
                xlabel('x');ylabel('y');zlabel('z');
                hold off
               %}
            end
            
            tas_mode = 1;
            stt_mode = 2;
        end
    elseif(now_mode == 3)
        set(handles.now_mode,'String','当前工作模式：STT');
        if(stt_mode == 1)
            tar_find = 0;
            tar_find_ang = zeros(2,1);  %记录已发现目标的角度
            tar_find_dis = zeros(4,1);  %记录已发现目标的距离;
            tas_mode = 1;
            for pit = 2:2
                for yaw = 1:13
                    now_pit = Beam_pit(pit);
                    now_yaw = Bean_azi(yaw);
                    [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,2);
                    yaw_string = ['当前雷达波束方位角',num2str(now_yaw),'°'];
                    pit_string = ['当前雷达波束俯仰角',num2str(now_pit),'°'];
                    set(handles.pit,'String',pit_string);
                    set(handles.yaw,'String',yaw_string);
                    r = R_max/2*ones(a);
                    polar(t,r,'r')
                    hold on
                    polar([(now_yaw - 5)*pi/180,(now_yaw - 5)*pi/180],[0,R_max/2],'b')
                    polar([(now_yaw + 5)*pi/180,(now_yaw + 5)*pi/180],[0,R_max/2],'b')
                    tar_find_ang = tar_ang;
                    tar_find_dis = tar_dis;
                    Phi = real_ang(2,:);
                    Sita = real_ang(1,:);
                    m = tar_num;
                    tar_find = tar_num;
                    rname = string();
                    for kk = 1:m
                        rname(kk) = ['目标',num2str(kk)];
                    end
                    set(handles.display,'Data',tar_ang');
                    set(handles.display,'RowName',rname)
                    set(handles.dis_display,'Data',tar_dis');
                    set(handles.dis_display,'RowName',rname)
                    for ii = 1:tar_num
                        r = tar_dis(1,ii);
                        polar(tar_ang(2,ii)*pi/180,r,'rh');
                    end
                    hold off
                    pause(0.1);
                end
            end
            stt_mode = 2;
        elseif(stt_mode == 2)
            CPI_num = 64;
            tas_mode = 2;
            now_pit = tar_find_ang(1,min_index);
            now_yaw = tar_find_ang(2,min_index);
            tar_dis_rec = zeros(3,CPI_num);
            total_data = zeros(2,3,CPI_num);
            for kk = 1:CPI_num
                [tar_num,tar_ang,tar_dis,R_max,Pos_Aircraft,Pos_target,real_ang] = all_process_func(now_mode,now_pit,now_yaw,V_Aircraft,Pos_Aircraft,Pos_target,Phi,Sita,tar_find,tar_find_ang,tar_find_dis,tas_mode,min_index);
                tar_dis_rec(:,kk) = tar_dis(2:4,min_index);
                now_pit = tar_ang(1);now_yaw = tar_ang(2);
                Phi = real_ang(2,:);
                Sita = real_ang(1,:);
                tar_find = tar_num;
                tar_find_dis(:,min_index) = tar_dis(:,min_index);
                tar_find_ang(:,min_index) = tar_ang;
                yaw_string = ['当前雷达波束方位角',num2str(now_yaw),'°'];
                pit_string = ['当前雷达波束俯仰角',num2str(now_pit),'°'];
                set(handles.pit,'String',pit_string);
                set(handles.yaw,'String',yaw_string);
                r = R_max/2*ones(a);
                polar(t,r,'r')
                hold on
                polar([(now_yaw - 5)*pi/180,(now_yaw - 5)*pi/180],[0,R_max/2],'b')
                polar([(now_yaw + 5)*pi/180,(now_yaw + 5)*pi/180],[0,R_max/2],'b')
                rname = ['目标',num2str(min_index)];
                set(handles.display,'Data',tar_ang');
                set(handles.display,'RowName',rname)
                set(handles.dis_display,'Data',tar_dis(:,min_index)');
                set(handles.dis_display,'RowName',rname)
                r = tar_dis(1,min_index);
                polar(tar_ang(2)*pi/180,r,'rh');
                hold off
                pause(0.1)
            end
            clc
            getd = @(p)path(path,p);
            getd('fire_control_subpro/');
            x_soft = zeros(3,CPI_num);
            for kk = 1:3
                x_soft(kk,:) = soft_fil(tar_dis_rec(kk,:),8);
            end
            total_data(1,:,:) = x_soft;
            %增加虚警点
            false_a = zeros(1,3,CPI_num);
            for kk = 1:1
                [x1] = find(abs(total_data(kk,1,:)) == max(max(max(abs(total_data(kk,1,:))))));
                [x2] = find(abs(total_data(kk,2,:)) == max(max(max(abs(total_data(kk,2,:))))));
                [x3] = find(abs(total_data(kk,3,:)) == max(max(max(abs(total_data(kk,3,:))))));
                false_a(kk,1,:) = false_a(kk,1,:) + sign(total_data(kk,1,x1(1)))*(rand(size(false_a(kk,1,:)))+0.1)*max(max(abs(total_data(kk,1,:))))*2;
                false_a(kk,2,:) = false_a(kk,2,:) + sign(total_data(kk,2,x2(1)))*(rand(size(false_a(kk,2,:)))+0.1)*max(max(abs(total_data(kk,2,:))))*2;
                false_a(kk,3,:) = false_a(kk,3,:) + sign(total_data(kk,3,x3(1)))*(rand(size(false_a(kk,3,:)))+0.1)*max(max(abs(total_data(kk,3,:))))*2;
                total_data(2,:,:) = false_a(1,:,:);
            end
            
            %{
                figure;hold on
                for kk = 1:CPI_num
                    plot3(squeeze(total_data(:,1,kk)),squeeze(total_data(:,2,kk)),squeeze(total_data(:,3,kk)),'.','color',[rand rand rand])
                    drawnow;
                end
                xlabel('x');ylabel('y');zlabel('z');
            %}
            %% 跟踪滤波
            % 跟踪参数
            dis_door = 10;                           %距离门大小
            if_rock = zeros(1,CPI_num);       %是否跟踪到
            lose_num = 0;                           %跟丢累计数
            pre_num = 5;                           %预测参考数
            
            tra_pro = zeros(3,CPI_num);       %跟踪结果
            start_point = total_data(1,:,1);        %跟踪起点确认
            tra_pro(:,1) = start_point;
            for ii = 2:CPI_num
                for kk = 1:size(total_data,1)
                    now = total_data(kk,:,ii);                  %当前判断点
                    sigma = norm(abs(now' - tra_pro(:,ii-1)));        %算三个距离总共平方差
                    if(sigma < dis_door)
                        tra_pro(:,ii) = total_data(kk,:,ii);    %确定点
                        if_rock(ii) = 1;
                        lose_num = 0;
                        break
                    end
                end
                if(if_rock(ii)==0)
                    if(ii > pre_num)
                        tra_pro(:,ii) = my_predict(tra_pro(:,ii-pre_num:ii-1));
                    elseif(ii > 2 && ii < pre_num)
                        tra_pro(:,ii) = my_predict(tra_pro(:,1:ii-1));
                    else
                        tra_pro(:,ii) = tra_pro(:,ii-1);
                    end
                    lose_num = lose_num + 1;
                end
                if(lose_num > 2)
                    disp('lose')
                    break
                end
            end
            %{
                figure;hold on
                for kk = 1:CPI_num
                    plot3(tra_pro(1,kk),tra_pro(2,kk),tra_pro(3,kk),'.')
                    %drawnow
                end
                xlabel('x');ylabel('y');zlabel('z');
                hold off
            %}
            
        end
    end
    dis_min = 1e10;
    min_index = 0;
    for kk = 1:tar_find
        if(tar_find_dis(1,kk) < dis_min)
            dis_min = tar_find_dis(1,kk);
            min_index = kk;
        end
    end
    if(dis_min <= 3e3)
        now_mode = 3;
        if(now_mode ~= 3)
            set(handles.now_mode,'String','当前工作模式：STT');
        end
    elseif(dis_min <= 1e4)
        now_mode = 2;
        if(now_mode ~= 2)
            set(handles.now_mode,'String','当前工作模式：TAS');
        end
    else
        now_mode = 1;
        if(now_mode ~= 1)
            set(handles.now_mode,'String','当前工作模式：TWS');
        end
    end
    V = 10000;            %飞机径向速度（m/s）
    V_Aircraft = V*[cos(tar_find_ang(1,min_index)*pi/180)*sin(tar_find_ang(2,min_index)*pi/180); cos(tar_find_ang(1,min_index)*pi/180)*cos(tar_find_ang(2,min_index)*pi/180); sin(tar_find_ang(1,min_index)*pi/180)];    %朝最近的目标飞去
    guidata(hObject, handles);
end

% --- Executes on button press in pause_button.
function pause_button_Callback(hObject, eventdata, handles)
% hObject    handle to pause_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);


% --- Executes on button press in end_button.
function end_button_Callback(hObject, eventdata, handles)
% hObject    handle to end_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp(2);

guidata(hObject, handles);


% --- Executes when entered data in editable cell(s) in display.
function display_CellEditCallback(hObject, eventdata, handles)

% hObject    handle to display (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when selected cell(s) is changed in display.
function display_CellSelectionCallback(hObject, eventdata, handles)
% hObject    handle to display (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) currently selecteds
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function screen_CreateFcn(hObject, eventdata, handles)
% hObject    handle to screen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate screen


% --- Executes during object creation, after setting all properties.
function now_mode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to now_mode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
Pos_Aircraft=[0;0;0];
Phi = [8 3 1 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
handles.pos = [ Pos_Aircraft+ 5e4*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 4e4*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e4*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];                   % 目标4
handles.nowmode = 1;
guidata(hObject, handles);
% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Pos_Aircraft=[0;0;0];
Phi = [8 3 1 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
handles.pos = [ Pos_Aircraft+ 5e3*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 4e3*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e3*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];
% Hint: get(hObject,'Value') returns toggle state of radiobutton2
handles.nowmode = 2;
guidata(hObject, handles);

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, ~, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Pos_Aircraft=[0;0;0];
Phi = [8 3 1 ]./180*pi;                 %俯仰
Sita = [-20 -1 15 ]./180*pi;             %方位
handles.pos = [ Pos_Aircraft+ 5e2*[cos(Phi(1))*sin(Sita(1)); cos(Phi(1))*cos(Sita(1)); sin(Phi(1))] ,...
    Pos_Aircraft+ 4e2*[cos(Phi(2))*sin(Sita(2)); cos(Phi(2))*cos(Sita(2)); sin(Phi(2))],...
    Pos_Aircraft+6e2*[cos(Phi(3))*sin(Sita(3)); cos(Phi(3))*cos(Sita(3)); sin(Phi(3))]];
handles.nowmode = 3;
guidata(hObject, handles);
% Hint: get(hObject,'Value') returns toggle state of radiobutton3
