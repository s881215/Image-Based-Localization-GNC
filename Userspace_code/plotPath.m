clc;
clear all;
close all;
[x1,y1]=refPts_yml('multiPaths_0.yml');
[x2,y2]=parse_yaml_xy('recordCarPath0.yml');
x2=x2(1:length(x2)-1);
y2=y2(1:length(y2)-1);
figure(1);
scatter(x1,   y1,   100, 'k', 'filled', ...
    'DisplayName', 'desired position');
hold on;
grid on;
%scatter(0,   0,   100, 'k', 'filled', ...
%    'DisplayName', 'desired position');
plot(x2,y2,'Color','b','LineStyle','--','LineWidth',1);
xlabel('m');
ylabel('m');
legend("reference point","tracking path");
title("reference points vs. tracking path");
xlim([-1,1]);
% x1=x1(:);
% y1=y1(:);
% x2=x2(:);
% y2=y2(:);
% n=length(x1);
% errX=[];
% errY=[];
% errDist=zeros(n,1);
% for i=1:n
%     dx=x2-x1(i);
%     dy=y2-y1(i);
%     d=sqrt(dx.^2 + dy.^2);
%     [errDist(i),idx]=min(d);
%     errX=[errX x2(idx)-x1(i)];
%     errY=[errY y2(idx)-y1(i)];
% end
x1 = x1(:);  y1 = y1(:);
x2 = x2(:);  y2 = y2(:);

N = numel(x1);
M = numel(x2);

errDist = zeros(N,1);  % 垂直最近距離
px = zeros(N,1);           % 對應軌跡上的最近點座標 (可選)
py = zeros(N,1);

for i = 1:N
    P = [x1(i); y1(i)];
    dmin = inf;

    for k = 1:M-1
        A = [x2(k);   y2(k)];
        B = [x2(k+1); y2(k+1)];

        v = B - A;
        w = P - A;

        t = dot(w,v) / dot(v,v);   % 投影參數

        if t < 0
            Q = A;                 % 最近點在端點 A
        elseif t > 1
            Q = B;                 % 最近點在端點 B
        else
            Q = A + t*v;           % 最近點在 AB 線段上（此時 PQ 垂直於 AB）
        end

        d = norm(P - Q);

        if d < dmin
            dmin = d;
            Qbest = Q;
        end
    end

    errDist(i) = dmin;
    px(i) = Qbest(1);
    py(i) = Qbest(2);
end
figure(2);
% scatter(errX,   errY,   60, 'red', 'filled', ...
%     'DisplayName', 'Error');
plot(1:N,errDist,'o-','LineWidth',1.5,'Color','r');
hold on;
grid on;
title("Distance Error (m)");
xlabel("reference point index");
ylabel("m");
[errMax,idxMax]=max(errDist);
[errMin,idxMin]=min(errDist);
plot(idxMax,errMax,'bs','MarkerSize',8,'LineWidth',1.5);
plot(idxMin,errMin,'gs','MarkerSize',8,'LineWidth',1.5);
text(idxMax,errMax,sprintf(' max = %.3f',errMax),'VerticalAlignment','bottom','HorizontalAlignment','left');
text(idxMin,errMin,sprintf(' min = %.3f',errMin),'VerticalAlignment','top','HorizontalAlignment','left');
meanErr=mean(errDist);
yline(meanErr,'k--','LineWidth',1.5);
text(N,meanErr,sprintf(' mean = %.3f',meanErr),'VerticalAlignment','bottom','HorizontalAlignment','right');
legend('distance error','max','min','Location','best');

function [A,B] = parse_yaml_xy(fname)
    str = fileread(fname);                     % 讀整個檔
    % 抓出每個 record 裡的 x 與 y（支援科學記號）
    tok = regexp(str, 'x:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*[\r\n]+.*?y:\s*([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)', ...
                  'tokens');
    % 轉成數值矩陣 [x y]
    A = cell2mat(cellfun(@(t) str2double(t{1}), tok, 'UniformOutput', false));
    B = cell2mat(cellfun(@(t) str2double(t{2}), tok, 'UniformOutput', false));
end
function [x_ref,y_ref]=refPts_yml(fname)
    txt=fileread(fname);
    % 只擷取 "data: [ ... ]" 內的內容（包含跨行）
    s = regexp(txt, 'data\s*:\s*\[', 'end', 'once');           % 指到左中括號後一個字元
    e = s - 1 + regexp(txt(s:end), '\]', 'once');               % 尋找對應的右中括號
    arrtxt = txt(s:e-1);                                        % 中括號內純文字
    
    % 從這段文字擷取數字
    m    = regexp(arrtxt, '[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', 'match');
    data = str2double(m).';                                     % 只會是 data 裡面的數字
    
    % 分成 x_ref / y_ref
    assert(mod(numel(data),2)==0, 'data 內的元素數量必須為偶數');
    x_ref = data(1:2:end);
    y_ref = data(2:2:end);
end