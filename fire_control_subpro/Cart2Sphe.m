function [r,sita,phi] = Cart2Sphe(x,y,z)
%   Cart2Sphe: �ѿ�������ϵת������ϵ
%   Input��x,y,z: �ѿ�������ϵ����
%   Output��r,sita,phi: ������ϵ����
%%
sita = atan(x/y);
phi = atan(z/sqrt(x^2+y^2));
r = sqrt(x^2+y^2+z^2);
sita = sita/pi*180;
phi = phi/pi*180;
end