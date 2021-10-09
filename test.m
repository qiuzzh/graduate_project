theta = [-1.4158,2.2474]*pi/180;
tar_posi = 2.973276147822472e+04*[cos(theta(1))*sin(theta(2)),cos(theta(1))*cos(theta(2)),sin(theta(1))];
r_posi = [145.238434346862,385.501352139530,272.147110194769];
v = 1e4*[cos(theta(1))*sin(theta(2)),cos(theta(1))*cos(theta(2)),sin(theta(1))];
Tr = 2e-3;
vt = [ -500,              1000,           -1000];
while(1)
    r_posi = r_posi+v*Tr;
    tar_posi = tar_posi + vt*Tr;
    cha = abs(tar_posi - r_posi);
    dis = sqrt(cha(1)^2+cha(2)^2+cha(3)^2)
end
