%% Yol Değerleri Güç Hesabı
clc
clear
Cx = 0.19; %Drag coefficient
Cr = 0.0048; %Rolling coefficient
m = 1000; %vehicle mass with 2 person
A = 1.8; %front surface
eff = 0.85; %drivetrain efficiency
g = 9.81; %gravity
alpha = 15; %slope of road
dens = 1.25; %air density
speed = 110; %vehicle desired speed in km/h
r = 0.3; %wheel radius

v = speed*(1000/3600); %km/h to m/s
deltaT = 1/60; %deltaT in second
vel = diff(v); %acceleration
ini = 0; %starting speed
dv = [ini vel]; %first integer is must be = 0

f = m*dv/deltaT; %Newton law of motion
fa = 0.5*dens*A*Cx*v^2; %Aerodynamic Drag Force
fr = m*Cr*g*cosd(alpha); %Rolling Friction
fg = m*g*sind(alpha); %hill climbing force
ft = f+fa+fr+fg; %Total Force
Pv = v*ft; %Instantaneous vehicle power needed in watts
fprintf("Total Force: %f\n", ft);
fprintf("Batt Power: %f kW\n", Pv/1000);
Pm = Pv/eff; %Motor Power
fprintf("Motor Power: %f KW\n", Pm/1000);
%% Güç ile Tork Hesabı
%we take it approximately Pm to 115000
P = 115000;
wheeltra = 2*pi*r; %linear wheel travel
rpm = (v/deltaT)/wheeltra; %RPM
round_rpm = round(rpm/100)*100; %rpmi roundlıyor
RPM = round_rpm;
fprintf("RPM: %f\n", RPM);
%T = P*60/(2*pi*RPM); 
To = ft*(r); %tork
%T1 = ft*r;
roundTo = round(To/100)*100; %torku roundladık
fprintf("Tork: %f\n", To);

% Max torque
%v = u + a*t
v1 = 30*1000/3600; %lets say 30 km/h but in m/s
t = 10; %second
a = (v1 - ini)/deltaT/t; %acc
force = m*a;
maxT = force*(r/2);
fprintf("Max Tork: %f\n", maxT);
%% Sarım ve Oluk değerleri
Np = 8; %Number of poles
frekans = Np*RPM/120; %frekans
ph = 3; %three phase
% = 12;
Ns = Np*ph; %number of slots in single phase
Nspp = Ns/Np/ph; %coil per pole
Nc = Ns/ph; %number of coils per phase
coil_span = Ns/Np; %max coil span in slots
Ncp = Ns/(Np/2); %conductor per pole
Kw = 1 - 16/(27*ph);
%% Volt, Amper, Density Constants
E = 900;
Vrms = E/sqrt(6); %simülasyona yazacağımız Volt değeri
I = P/(eff*3*Vrms); %Akım
K_fill =0.60; %slot fill factor
J = 5; %current density diğer analiz kodundan seçildi
qcu = I/J;
%1.829 mm çapında 10 iletken paralel
Asl = qcu*20/K_fill;
%% Rotor/Stator Çapı
TRV = 48;
D2L = (To/1000)/(pi*TRV/4);
lambda = 1;
D_L = Np/(pi);
Dro = nthroot(D2L*D_L, 3);
Lstk = Dro/D_L;
g1 = 1; %mm
fprintf("airgap: %f\n", g1);
%% Electric Loading
tp = (pi*Dro*1000)/(Np); %pole pitch
B = 0.85;
fb = frekans*1.25;
kba = (2/pi)*B*tp*Lstk/1000;
wa = kba*fb*Kw*Vrms/(2*pi/sqrt(2));
Z = 2*ph*wa;
zo = Z/Ns;
zo_2 = round(zo);
B_new = B*zo_2/zo;
% B = 0.55; % Manyetik yüklenme
% sheerstress=48/2;
% Amy = (sheerstress/B); %%A/m
% Nph = (Amy*1000*pi*Di*10^-3)/(2*ph*Ieff); % Nph faz başına iletken sayısı 
% Z = (Amy*1000*pi*Di*10^-3)/Ieff;
% Zslot = round(Z/Ns);
%% Slot Design
tp = (pi*Dro*1000)/(Np); %pole pitch 
%tp = Ns/Np;
ts = (pi*Dro*1000)/Ns; %slot pitch
wt1 = (B_new*ts)/(0.95*1.7);
Ad = kba/1.6;
bo = Ad/(Lstk*0.95);
%wt2 = ts - bo;
h2 = 2.5*wt1;
%stator boyunduruk kalınlığı
hb = kba*1000/(2*0.95*Lstk*1.5);
Dsi = Dro*1000+2*g1;
Dso = Dsi+2*h2+2*hb;
%rotor boyunduruk
hrb = (tp*B_new)/(2*0.95*1.5);
Dri = Dsi-2*g1-2*hrb;

wt = 0.5*ts; %tooth width
Hs2 = 3*wt;
bso = (ts-wt1)*0.2;
Hs0 = (ts-wt)*0.3;
Hs1 = bso;
fprintf("wt: %f\n", wt);
fprintf("Hs2: %f\n", Hs2);
fprintf("bso: %f\n", bso);
fprintf("Hs0: %f\n", Hs0);
fprintf("Hs1: %f\n", Hs1);
