clc;
clear all;
close all;
dt=0.01;
ttt=120;
t=0:dt:ttt;
R=0.2275;
r=0.09;
n=51;
La=0.05897;
Ra=2.9;
kb=(0.000254*60*n)/(2*pi);
kT=0.02426*n;
J=0.1;
B=0.1;
T=ttt;
%%
% slope=0.734;
% v=0.5;
% w=0;
% u=[1;slope];
% u=u/norm(u);
% s=v*t;
% xd=u(1)*s;
% yd=u(2)*s;
% thetad=atan2(u(2),u(1))*ones(size(t));
% xdd=xd;
% ydd=yd;
% thetadd=thetad;
%-----------------------------------------------
%============逆時針的圓=================%
% R_c=5;
% xc0=1;
% yc0=1;
% Trev=20;
% ome=2*pi/Trev;
% th=ome*t;
% % 逆時針
% xd=xc0+R_c*cos(th);
% yd=yc0-R_c*sin(th);
% % 解析式直接算
% vd = R_c*ome;            % 0.6283  (常數向量也行：vd = vd*ones(size(t));)
% wd = -ome;               % -0.3142 (同上，可建成常數向量)
% xd_dot = -R_c*ome*sin(th);          % 解析式
% yd_dot = -R_c*ome*cos(th);
% thetad = unwrap(atan2(yd_dot, xd_dot));
% xdd=xd;
% ydd=yd;
% thetadd=thetad;
% v=vd;
% w=wd;
%==========順時針的圓===================%
% Rc = 5;
% xc0 = 1;
% yc0 = 1;
% Trev = 20;
% ome = 2*pi/Trev;
% th = ome*t;
% 
% % 參考路徑
% xd = xc0 + Rc*sin(th);
% yd = yc0 + Rc*cos(th);
% 
% % 對應速度
% vd = Rc*ome;               % 常數
% wd = ome;                  % 常數
% xd_dot = Rc*ome*cos(th);
% yd_dot = -Rc*ome*sin(th);
% 
% % Heading 角
% thetad = unwrap(atan2(yd_dot, xd_dot));
% xdd=xd;
% ydd=yd;
% thetadd=thetad;
% v=vd;
% w=wd;
%-----------------------------------------------
%======= 前面斜線後面圓 =======%
% 路徑參數
slope = 0.734;
v = 0.5;             % 線速度
u = [1; slope];
u = u / norm(u);

% 直線階段長度
T1 = T/5;
s1 = v * T1;
x1_end = u(1)*s1;
y1_end = u(2)*s1;

% 預分配
xd = zeros(size(t));
yd = zeros(size(t));
thetad = zeros(size(t));
w=zeros(size(t));
for k = 1:length(t)
    if t(k) <= T1
        % ===== 第一段：直線 =====
        s = v * t(k);
        xd(k) = u(1) * s;
        yd(k) = u(2) * s;
        thetad(k) = atan2(u(2), u(1));
        w(k)=0;
    else
        % ===== 第二段：圓弧 =====
        % 半徑
        R = 5;             % 可以調整曲率 (R越大越平緩)
        omega = v / R;     % 角速度
        tau = t(k) - T1;   % 從圓弧開始的時間

        % 圓弧中心設定：讓路徑在直線末端連續
        th0 = atan2(u(2), u(1)); % 直線方向角
        % 圓心在左側（左轉），可改成 -R 變成右轉
        xc = x1_end - R * sin(th0);
        yc = y1_end + R * cos(th0);

        % 在圓弧上運動
        th = th0 + omega * tau;
        xd(k) = xc + R * sin(th);
        yd(k) = yc - R * cos(th);
        thetad(k) = th;  % 切線方向
        w(k)=omega;
    end
end
xdd=xd;
ydd=yd;
thetadd=thetad;
%----------------------------------------------%
%============= 向+x軸的參考路徑===================%
% x0=0;
% y0=1;
% vd=1;
% xd=x0+vd*t;
% yd=y0*ones(size(t));
% xd_dot=vd*ones(size(t));
% yd_dot=zeros(size(t));
% thetad=zeros(size(t));
% xdd=xd;
% ydd=yd;
% thetadd=thetad;
% v=vd;
% w=0;
%%

model='one_step.slx';
open_system(model);
assignin('base','r',timeseries(r,t));
assignin('base','R',timeseries(R,t));
assignin('base','La',timeseries(La,t));
assignin('base','Ra',timeseries(Ra,t));
assignin('base','kb',timeseries(kb,t));
assignin('base','kT',timeseries(kT,t));
assignin('base','J',timeseries(J,t));
assignin('base','B',timeseries(B,t));
assignin('base','xd',timeseries(xd,t));
assignin('base','yd',timeseries(yd,t));
assignin('base','thetad',timeseries(thetad,t));
assignin('base','w',timeseries(w,t));
assignin('base','v',timeseries(v,t));
out=sim(model);

figure(1);
subplot(2,2,1);
plot(xdd,ydd,'color','r','LineWidth',2);
title("desired path");
xlabel("m");
ylabel("m");
axis equal;
grid on;
subplot(2,2,2);
plot(out.xc.Data,out.yc.Data,'color','b','LineWidth',1);
title("actual path");
xlabel("m");
ylabel("m");
axis equal;
grid on;

figure(2);
plot(xdd,ydd,'color','r','LineWidth',2);
hold on;
grid on;
plot(out.xc.Data,out.yc.Data,'color','b','LineWidth',1,'LineStyle','--');
title("Reference path and Actual path");
xlabel("m");
ylabel("m");
legend("Reference path","Actual path");
axis equal;

figure(3);
subplot(2,1,1);
plot(out.eL.Time,out.eL.Data,'color','b','LineWidth',1);
title('left motor input voltage');
xlabel('time');
ylabel('V');
grid on;
subplot(2,1,2);
plot(out.eR.Time,out.eR.Data,'color','r','LineWidth',1);
title('right motor input voltage');
xlabel('time');
ylabel('V');
grid on;

figure(4);
plot(out.ex.Time,out.ex.Data,'color','b','LineWidth',1);
hold on;
grid on;
plot(out.ey.Time,out.ey.Data,'color','r','LineWidth',1);
plot(out.etheta.Time,out.etheta.Data,'color','g','LineWidth',1);
legend("error_x","error_y","error_{\theta}");
xlabel("time");
title("error");

% figure(4);
% plot(xdd,ydd,'color','r','LineWidth',1);
% hold on;
% grid on;
% plot(out.xc1.Data,out.yc1.Data,'color','b','LineWidth',1);
% title("Desired path and Actual path (v2)");
% xlabel("m");
% ylabel("m");
% axis equal;
% 
% figure(5);
% plot(out.ex1.Time,out.ex1.Data,'color','b','LineWidth',1);
% hold on;
% grid on;
% plot(out.ey1.Time,out.ey1.Data,'color','r','LineWidth',1);
% plot(out.etheta1.Time,out.etheta.Data,'color','g','LineWidth',1);
% title("error (virsion 2)");
% legend("error_x","error_y","error_{\theta}");
% xlabel("time");
% ylabel("error");
% 
% figure(6);
% plot(xdd,ydd,'color','r','LineWidth',1);
% hold on;
% grid on;
% plot(out.xc3.Data,out.yc3.Data,'color','b','LineWidth',1,'LineStyle','--');
% title("Reference path and Actual path");
% legend("Reference path","Actual path");
% xlabel("m");
% ylabel("m");
% axis equal;
% 
% figure(7);
% plot(out.ex3.Time,out.ex3.Data,'color','b','LineWidth',1);
% hold on;
% grid on;
% plot(out.ey3.Time,out.ey3.Data,'color','r','LineWidth',1);
% title("error of position")
% legend("error_x","error_y");
% xlabel("time");
% ylabel("error");
% 
% figure(8);
% plot(out.delta.Time,out.delta.Data,'color','b','LineWidth',1);
% grid on;
% title("error of orientation (version 3)")
% xlabel("time");
% ylabel("error");
% 
% figure(9);
% plot(out.thetac3.Time,out.thetac3.Data);
% grid on;
% hold on;
% plot(out.thetac_now.Time,out.thetac_now.Data);
% 
% figure(10);
% plot(xdd,ydd,'color','r','LineWidth',1);
% grid on;
% axis equal;
% title("Reference Path");
% xlabel("m");
% ylabel("m");

%% estimate the performance
% % the first way
% performanceTime=out.yc.Time;
% performanceY=out.yc.Data;
% y_final=1;
% info=stepinfo(performanceY,performanceTime,y_final,'SettlingTimeThreshold',0.02);
% Mp=info.Overshoot;
% Ts=info.SettlingTime;
% figure(11);
% subplot(2,1,1);
% plot(out.xc.Data,out.yc.Data,'color','b','LineWidth',1);
% hold on;
% grid on;
% plot(xdd,ydd,'color','r','LineWidth',1,'LineStyle','--');
% title("Reference path and Actual path");
% legend("reference path","actual path")
% xlabel("m");
% ylabel("m");
% axis equal;
% 
% subplot(2,1,2);
% plot(out.yc.Time,out.yc.Data,'color','b','LineWidth',1);
% grid on;
% xlabel('time(sec)');
% ylabel('y-value');
% figure11_title=sprintf('Mp：%.3f %%, Ts：%.3f sec',Mp,Ts);
% title(figure11_title);
% 
% % the second way
% y_0=performanceY(1);
% y_ref=1;
% A=y_ref-y_0;
% 
% y_peak=max(performanceY);
% Mp2=max(0,(y_peak-y_ref)/abs(A))*100;
% 
% tol=0.02*abs(A);
% idx_last_out=find(abs(performanceY-y_ref)>tol,1,'last');
% if isempty(idx_last_out)
%     Ts2=0;
% elseif idx_last_out<numel(t)
%     Ts2=performanceTime(idx_last_out+1);
% else
%     Ts2=NaN;
% end
% figure(12);
% subplot(2,1,1);
% plot(out.xc.Data,out.yc.Data,'color','b','LineWidth',1);
% hold on;
% grid on;
% plot(xdd,ydd,'color','r','LineWidth',1,'LineStyle','--');
% title("Reference path and Actual path");
% legend("reference path","actual path")
% xlabel("m");
% ylabel("m");
% axis equal;
% 
% subplot(2,1,2);
% plot(out.yc.Time,out.yc.Data,'color','b','LineWidth',1);
% grid on;
% xlabel('time(sec)');
% ylabel('y-value');
% figure12_title=sprintf('Mp：%.3f %%, Ts：%.3f sec',Mp2,Ts2);
% title(figure12_title);