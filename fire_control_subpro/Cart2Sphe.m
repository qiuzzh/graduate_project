function [r,sita,phi] = Cart2Sphe(x,y,z)
%   Cart2Sphe: 笛卡尔坐标系转球坐标系
%   Input：x,y,z: 笛卡尔坐标系坐标
%   Output：r,sita,phi: 球坐标系坐标
%%
sita = atan(x/y);
phi = atan(z/sqrt(x^2+y^2));
r = sqrt(x^2+y^2+z^2);
sita = sita/pi*180;
phi = phi/pi*180;
end