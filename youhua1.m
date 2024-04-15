%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 节点编号规则： 
% subsinkNum为subsink节点的数量
% nodeNumber为总的传感器节点数量
% sensorNum = nodeNumber - subsinkNum为感知节点的数量
% 1 - subsinkNum为subsink的编号；
% subsinkNum+1 - nodeNumber为感知节点的编号
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
M = readmatrix('subsinkHops.txt'); % 从文件中读取感知节点到对应的subsink节点的跳数到矩阵
subsinkNum = 12;  % subsink节点数目
nodeNumber = 120; % 总的传感器节点数，包括感知节点和subsink节点
sensorNum = nodeNumber - subsinkNum; % 感知节点数量


% 转化矩阵形式，格式为按每个感知节点到不同subsink节点的跳数值
for i=1:sensorNum
    A(subsinkNum*(i-1)+1:subsinkNum*(i-1)+subsinkNum,:) = M(i:sensorNum:sensorNum*subsinkNum,:);
end

% 将感知节点的编号：（subsinkNum+1）--- nodeNumber 更新为： 1 --- （nodeNumber-subsinkNum)
% 重新编号后的感知节点 i 到 subsink节点 j 的路数值为 ：A(subsinkNum*(i-1)+j,3)
A(:,1)=A(:,1) - subsinkNum;

% 按论文的模型，将矩阵A进一步改造，换成 H(sensorNum * subsinkNum)的形式，
% H(i,j)表示第i个感知节点到第j个subjsink节点的跳数值
for i=1:sensorNum
    for j=1:subsinkNum
        H(i,j) = A(subsinkNum*(i-1)+j, 3);
    end
end    

% % 矩阵（解）S（sensorNum * subsinkNum）：S(i,j)为二进制变量，
% % S(i,j) = 1 表示 感知节点 i 选择subsink节点 j 作为中转节点；
% % S(i,j) = 0 表示 感知节点 i 不是subsink节点 j 的成员节点
% % 创建0/1型决策变量，full表示全参数矩阵

S = binvar(sensorNum, subsinkNum);

C = [];

% 目标值
E = 0;
for i=1:sensorNum
    for j=1:subsinkNum
        E = E + S(i,j)*H(i,j);
    end
end

% 约束条件一：确保每个感知节点必须且仅选择一个subsink节点
for i=1:sensorNum
     C = [C, sum(S(i,:)) == 1];
end

% 读取文件中保存的每个subsink节点的通信时长
fid = fopen('subsinkTransmit.txt', 'r');
timeListData = textscan(fid, '%s %d');
timeList = [];
for i=1:subsinkNum
    timeList(i) = timeListData{2}(i);  % timeList里为每个subsink节点与sink节点的通信时长
end
fclose(fid);

%公式为：0.2kbps * T(采集周期)* (subsink节点管辖感知节点数+1) ----------- 20kbps * T(传输时间)
% 参考方面数据：T(传输时间)为自己仿真的通信时长，而T(采集周期)为300S，这样可保证在通信时长内感知节点无法将全部数据传递给sink节点
minReq = [];
for i=1:subsinkNum
   minReq(i) = floor(timeList(i)/3) - 1;
end

% 约束条件二： 确保每个subsink节点的成员数量要大于最小需求量
for j=1:subsinkNum
    C = [C, sum(S(:, j)) >= minReq(j)];
end
% 约束条件二：可修改为所有subsink节点管辖的成员节点总数小于等于某个值（对应到的是延时容许值）

% 约束条件三：考虑高密度网络，感知节点的总量要大于最小需求量的总和
C = [C, sensorNum >= sum(minReq)]; 

% 参数设置和求解
ops = sdpsettings('verbose', 0, 'solver', 'cplex');
sol = optimize(C, E);

if sol.problem == 0
     value(S)
     value(E)
else
    disp('求解过程出错');
end