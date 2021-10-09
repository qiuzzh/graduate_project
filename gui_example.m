%% Initialize
clc;close all;clear all;
%% Create Figure
global hF_start
hF_start=figure('menubar','none','Resize','off','NumberTitle','off','position',...
    [100,200,1200,700],'name','线性拟合','Visible','on');
%% Set Global Parameters
global aa
aa=[1,1,1];
global bb
bb=[0,0,0];
global Path_main
Path_main=pwd;
global edit1
global edit21
global edit22
global edit_fun
global edit3
global axes1
global axes2
global axes3
global edit211
global edit221
%% Set UI Parameters
ui_text_1    ={'温度(℃)','压强(atm)','线吸收强度'};
unit         ='*1e-24cm/molecule';
edit_text1   ={'30','0.0845526','8.273'};
ui_text_20   ={'初始值','计算值'};
ui_text_2    ={'q','z','碰撞偏移 d','碰撞展宽 g','FD多普勒','α','ξ','β','b af(x)+b','a af(x)+b'};
edit_text_2_1={'6','0.01','0','0','0.5','2','0','0','0','1'};
edit_text_2_2={'','','','','','','','','',''};
ui_text_3    ={'气体分压','气体浓度','计算耗时'};
unit_text_3  ={'atm','ppm','s'};
Text_CallBack={'原始数据','计算','拟合函数','结果导出','清除'};
Fun_CallBack ={'Input0','','calculate','Output0','Clearaxes'};
Bcolor='w';
set(hF_start,'Color',Bcolor);
d=100;
%% Set UI Main
Position_ui=[20 635 250 50;280 635 150 50;430 635  100 50;740 635 180 50;930 635 250 50];
for i=1:length(Text_CallBack)
    ui=uicontrol(gcf,'style','push','string','','ForegroundColor',bb,'value',0,'fontsize',15,'position',[0 0 0 0],'visible','on');
    set(ui,'string',Text_CallBack{i});
    set(ui,'BackgroundColor',Bcolor);
    set(ui,'callback',Fun_CallBack{i});
    set(ui,'position',Position_ui(i,:));
end
edit_fun=uicontrol(gcf,'style','edit','fontname','Times New Roman', ...
    'BackgroundColor',aa,'string','VP','value',0,'ForegroundColor',bb,'fontsize',12,'position',[430 635  100 50],'visible','on');
clear Position_ui 
%%  hp1
hp_0       =uipanel('Title','','FontSize',18,'BackgroundColor',Bcolor,'Position',[.015 .44 .97 .45]);
hp_1       =uipanel('Title','基本参数设置','FontSize',18,'BackgroundColor',Bcolor,'Position',[.02 .46 .2 .4]);
hp_11      =uipanel('Title','气体环境参数','FontSize',15,'BackgroundColor',Bcolor,'Position',[.025 .6 .18 .2]);
for i=1:length(ui_text_1)
    if i~=3
       ui=uicontrol(gcf,'style','text','ForegroundColor',bb,'value',0,'fontsize',15,'position',[40 330+(i-1)*50+d 80 30],'visible','on');
       edit1(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[120 330+(i-1)*50+d 50 30],'visible','on');
    else
       ui=uicontrol(gcf,'style','text','ForegroundColor',bb,'value',0,'fontsize',15,'position',[40 270+d  80 30],'visible','on');
       edit1(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[40+80 270+d  50 30],'visible','on');
    end 
    set(ui,'string',ui_text_1{i})
    set(ui,'fontsize',15)
    set(ui,'BackgroundColor',Bcolor);
    set(edit1(i),'fontsize',15)
    set(edit1(i),'string',edit_text1{i})
end
ui=uicontrol(gcf,'style','push','ForegroundColor',bb,'value',0,'fontsize',15,'position',[40 240+d 150 30],'visible','on');
set(ui,'style','text');
set(ui,'BackgroundColor',Bcolor);
set(ui,'string',unit);
set(ui,'fontsize',15);
%% hp2
hp_2 =uipanel('Title','拟合参数','FontSize',18,'BackgroundColor',Bcolor,'Position',[.225 .46 .53 .4]);
l=60;
for i=1:length(ui_text_2)
    if i>0 && i<=4
        ui=uicontrol(gcf,'style','push','ForegroundColor',bb,'value',0,'fontsize',15,'position',[230+l 240+(i-1)*60+d 80 30],'visible','on');
        edit21(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[310+l 240+(i-1)*60+d 50 30],'visible','on');
        edit22(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[310+50+l 240+(i-1)*60+d 50 30],'visible','on');
    elseif i>4 && i<=8
        ui=uicontrol(gcf,'style','push','ForegroundColor',bb,'value',0,'fontsize',15,'position',[430+l 240+(i-1-4)*60+d 80 30],'visible','on');
        edit21(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[510+l 240+(i-1-4)*60+d 50 30],'visible','on');
        edit22(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[510+50+l 240+(i-1-4)*60+d 50 30],'visible','on');
    else
        ui=uicontrol(gcf,'style','push','ForegroundColor',bb,'value',0,'fontsize',15,'position',[630+l 240+(i-1-8)*60+d 80 30],'visible','on');
        edit21(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[710+l 240+(i-1-8)*60+d 50 30],'visible','on');
        edit22(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[710+50+l 240+(i-1-8)*60+d 50 30],'visible','on');
    end
    set(ui,'string',ui_text_2{i})
    set(ui,'style','text');
    set(ui,'BackgroundColor',Bcolor);
    %set(edit21(i),'fontsize',15)
    set(edit21(i),'string',edit_text_2_1{i});
    set(edit22(i),'string',edit_text_2_2{i});
end
%%
hp_21 = uipanel('Title','名字','FontSize',18,'BackgroundColor',Bcolor,'Position',[.57 .63 .18 .2]);
i=length(ui_text_2);
ui=uicontrol(gcf,'style','text','BackgroundColor',Bcolor,'ForegroundColor',bb,'value',0,'fontsize',15,'position',[630+l 240+(i-1-8)*60+d+100 80 30],'visible','on');
edit211=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[710+l 240+(i-1-8)*60+d+100 50 30],'visible','on');
edit221=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','100','value',0,'ForegroundColor',bb,'fontsize',12,'position',[710+50+l 240+(i-1-8)*60+d+100 50 30],'visible','on');
set(ui,'string','参数名字')
set(ui,'style','text');
set(ui,'BackgroundColor',Bcolor);
set(edit211,'string','10');
set(edit221,'string','2');
%% Result Display
hp_3 = uipanel('Title','计算结果','FontSize',18,'BackgroundColor',Bcolor,'Position',[.76 .46 .22 .4]);
l=120;
for i=1:2
    ui=uicontrol(gcf,'style','push','string','振动点','ForegroundColor',bb,'value',0,'fontsize',15,'position',[830+l 360+(i-1)*50+d 80 30],'visible','on');
    edit3(i)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','','value',0,'ForegroundColor',bb,'fontsize',12,'position',[910+l 360+(i-1)*50+d 50 30],'visible','on');
    ui_3=uicontrol(gcf,'style','push','string','振动点','ForegroundColor',bb,'value',0,'fontsize',15,'position',[910+50+l 360+(i-1)*50+d 80 30],'visible','on');
    set(ui,'style','text');
    set(ui,'BackgroundColor',Bcolor);
    set(ui,'string',ui_text_3{i})
    set(ui,'fontsize',15);
    set(ui_3,'style','text');
    set(ui_3,'BackgroundColor',Bcolor);
    set(ui_3,'string',unit_text_3{i})
    set(ui_3,'fontsize',15);
end  
ui=uicontrol(gcf,'style','push','string','计算耗时','ForegroundColor',bb,'value',0,'fontsize',15,'position',[830+l 270+d 80 30],'visible','on');
set(ui,'style','text');
set(ui,'BackgroundColor',Bcolor);
set(ui,'fontsize',15);
edit3(3)=uicontrol(gcf,'style','edit','fontname','Times New Roman','BackgroundColor',aa,'string','','value',0,'ForegroundColor',bb,'fontsize',12,'position',[910+l 270+d 50 30],'visible','on');
ui_3=uicontrol(gcf,'style','push','ForegroundColor',bb,'value',0,'fontsize',15,'position',[910+l+50 270+d 50 30],'visible','on');
set(ui_3,'style','text');
set(ui_3,'BackgroundColor',Bcolor);
set(ui_3,'string',unit_text_3{3})
set(ui_3,'fontsize',15);
%%
%hp_4 = uipanel('FontSize',18,'BackgroundColor',Bcolor,'Position',[.015 .02 .97 .4]);
l=0.02;
axes1=axes('visible','on','units','normalized','position',[0.03+l,0.06,0.27,0.35]);
axes2=axes('visible','on','units','normalized','position',[0.35+l,0.06,0.27,0.35]);
axes3=axes('visible','on','units','normalized','position',[0.67+l,0.06,0.27,0.35]);
