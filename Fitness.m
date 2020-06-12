function [Fitness1]=Fitness(x,Fixedcosts,nuitTransCost,coldRate,congesteRate,goodLossRate,openDoorCost,openDoorCostRate)
%% 适应度函数
global D Startdistance Time Demand DemandMax v1 Matrix ServiceTime Cost
step = 0;
flag = 1;

if nargin<2
    flag = 0;
end
%% 信息矩阵Matrix

n=size(D,1);
Matrix=zeros(13,n);
Matrix(1,:)=x;
% n=length(x);
for i=1:n
    Matrix(6,i)=Time(x(i),1);%第六行对应目前最早可接受应该到达时间
    Matrix(7,i)=Time(x(i),2);%第七行对应目前最晚可接受到达时间
    Matrix(8,i)=ServiceTime(x(i));%第十行存储服务时间
end
for i=1:n
    %第一个地点由第一辆车负责，然后开始给1号车分配后续任务
    if i==1
        Matrix(2,i)=1;%第二行存储对应目前配送车辆
        Matrix(3,i)=Demand(x(i));%第三行存储对应目前载货量
        Matrix(4,i)=Matrix(6,i)-Startdistance(x(i))/v1;%第四行对应目前车辆出发时间
        Matrix(5,i)=Matrix(6,i);%第五行对应目前到达时间
        Matrix(9,i)=0;%第9行存储早到时间。
        Matrix(10,i)=0;%第10行存储晚到时间。
        Matrix(11,i)=Matrix(5,i)+ Matrix(8,i);%第11行存储出发时间
        Matrix(12,i)=Startdistance(x(i));%第12行车辆已走路程。
        Matrix(13,i)=Startdistance(x(i))/v1;%第13行车辆已运动时间
    else
        if Matrix(3,i-1)+Demand(x(i))<=DemandMax   %判断载重
            %满足，则依旧使用该车辆运送
            Matrix(2,i)=Matrix(2,i-1);%第二行存储对应目前配送车辆
            Matrix(3,i)=Matrix(3,i-1)+Demand(x(i));%第三行存储对应目前载货量
            Matrix(4,i)=Matrix(11,i-1);%第四行对应目前车辆出发时间
            Matrix(5,i)=Matrix(4,i)+D(x(i-1),x(i))/v1;%第五行对应目前到达时间
            Matrix(9,i)=max(0,Matrix(6,i)-Matrix(5,i));%第9行存储早到时间
            Matrix(10,i)=max(0,Matrix(5,i)-Matrix(7,i));%第10行存储晚到时间
            Matrix(11,i)=Matrix(5,i)+ Matrix(8,i);%第11行存储出发时间
            Matrix(12,i)=Matrix(12,i-1)+D(x(i-1),x(i));%第12行车辆已走路程。
            Matrix(13,i)=Matrix(13,i-1)+Matrix(5,i)-Matrix(4,i);%第13行车辆已运动时间
        else
            %不满足，则安排新车
            Matrix(2,i)=Matrix(2,i-1)+1;%第二行存储对应目前配送车辆
            Matrix(3,i)=Demand(x(i));%第三行存储对应目前载货量
            Matrix(4,i)=Matrix(8,i)-Startdistance(x(i))/v1;%第四行对应目前车辆出发时间
            Matrix(5,i)=Matrix(8,i);%第五行对应目前到达时间
            Matrix(9,i)=0;%第9行存储早到时间。
            Matrix(10,i)=0;%第10行存储晚到时间。
            Matrix(11,i)=Matrix(5,i)+ Matrix(8,i);%第11行存储出发时间
            Matrix(12,i)=Startdistance(x(i));%第12行车辆已走路程。
            Matrix(13,i)=Startdistance(x(i))/v1;%第13行车辆已运动时间
        end
    end
end
%% 目标函数
% （1）车辆固定成本
% 固定成本150元
f=Fixedcosts;
S=max(Matrix(2,:));
Fguding=f*S;
%（2）运输成本

% e=100;  % 冷藏设备平均功率：100kw
% Cf=1.2;%Cf:单位制冷能耗成本5元每公里
h=nuitTransCost;% 单位运输成本：15元/公里
Dist=0;
Q=zeros(1,n);
for i=1:S
    A=find(Matrix(2,:)==i);
    Dist=Dist+Startdistance(x(A(end)))+Matrix(12,A(end));
    Q(A)=Matrix(3,A(end));
end
Fyunshu=h*Dist;

%（3）制冷成本
b=coldRate;%制冷消耗系数

% Fzhileng=e*Cf*Dist/v1;
% Lij:路段的距离
% rij 拥堵系数
Fzhileng=0;
rij=congesteRate;
if step == 0
    for i=1:n
        if i==1
            Fzhileng=Fzhileng+b*Startdistance(x(i))/(v1*(rij-1));
        else
            Fzhileng=Fzhileng+b*D(x(i-1),x(i))/(v1*(rij-1));
        end
    end
else
    Fzhileng = 5*(n-1);
end
for i=1:n
    if i==1
        Fzhileng=Fzhileng+b*Startdistance(x(i))/(v1*(rij-1));
    else
        Fzhileng=Fzhileng+b*D(x(i-1),x(i))/(v1*(rij-1));
    end
end
% （4）货损成本
q=goodLossRate; %货损成本系数
Fhuosun=0;
for i=1:n
    if i==1
        Fhuosun=Fhuosun+q*Startdistance(x(i))/(v1*(rij-1));
    else
        Fhuosun=Fhuosun+q*D(x(i-1),x(i))/(v1*(rij-1));
    end
end
% （5）惩罚成本
[xl,si] =size(x); 
n=si;%表示接驳点数量
w1=20;%早到的惩罚成本 10元/小时
w2=30;%晚到的惩罚成本 12元/小时
% 构建惩罚矩阵FF
FF=zeros(2,n);
FF(1,:)=w1*Matrix(9,:);
FF(2,:)=w2*Matrix(10,:);
Fchengfa=sum(sum(FF))/60;
a=openDoorCost;%一次打开车厢门和卸货产生的制冷成本
p=openDoorCostRate;%表示车厢门开启或卸货造成的货损常数
Fitness1=Fguding+Fyunshu+Fhuosun+Fzhileng+Fchengfa+a*n+p*n;
if flag
    Cost = [Fguding,Fyunshu,Fhuosun+p*n,Fzhileng+a*n,Fchengfa];
end
end