clc;
clear all; close all;

wd=0.1;
vd=0.5;

Mp=10;
Ts=4;

dampingRatio=-log(Mp/100) / sqrt(pi^2 + log((Mp/100))^2);
wn=4 / (dampingRatio*Ts); % 4:2%, 3:5%

a=-dampingRatio*wn;
b=wn*sqrt(1-(dampingRatio)^2);
p=10*a;

c2=-(2*a+p);
c1=a^2 + b^2 + 2*a*p-2*wd^2;
c0=-p*(a^2+b^2)-12*a*wd^2;

polynomial=[1, -c2, -(2*wd^2 - c1), -(c0-c2*wd^2)];
kxs=roots(polynomial);
kys=zeros(3,1);
kthetas=zeros(3,1);
for i=1:length(kxs)
    %kys(i,1)=(c0-(c2-kxs(i,1))*wd^2)/(kxs(i,1)*vd*vd);
    kys(i,1)=(21*a^2+b^2-wd^2-kxs(i,1)*(-12*a-kxs(i,1)))/(vd^2);
    kthetas(i,1)=c2-kxs(i,1);
end

A=[-kxs(1) wd 0;
    -wd 0 vd;
    0 -kys(1)*vd -kthetas(1)];
eigA=eig(A);

C=eye(3);
sys=ss(A,[],C,[]);
t=0:0.01:120;
e0=[0;-4;0];
[y,t]=initial(sys,e0,t);
figure(1);
plot(t,y(:,2),'color','b','LineWidth',1);
grid on;