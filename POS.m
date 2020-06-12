%本算法的核心来源于对适应度和位置的转化高纬度转化
%% 1.清空环境变量
clear;
clc;
close all;
global  Startdistance D Demand Time ServiceTime v1 DemandMax Matrix Cost
%% 2.导入数据
citys = load('坐标.txt'); %数据集
Xstart=[40 50];           %配送中心坐标
Demand=load('需求量.txt');
Time=load('时间窗.txt');
ServiceTime=load('卸货时间.txt');
[n,~] = size(citys);
%% 3.初始化参数
c1=0.1;                         %个体学习因子
c2=0.075;                       %社会学习因子
w=0.7;                          %惯性因子
m=50;                           %粒子数量
pop=zeros(m,n);                 %粒子位置
v=zeros(m,n);                   %粒子速度
gen=1;                          %迭代计数器
genmax=500;                     %迭代次数
fitness=zeros(m,1);             %适应度函数值
Pbest=zeros(m,n);               %个体极值路径
Pbest_fitness=zeros(m,1);       %个体极值
Gbest=zeros(genmax,n);          %群体极值路径
Gbest_fitness=zeros(genmax,1);  %群体极值
Length_ave=zeros(genmax,1);     %各代路径的平均长度
ws=0.8;                         %惯性因子最大值
we=0.5;                         %惯性因子最小值
Fixedcosts = 150;                   %固定成本
nuitTransCost =2.4;                 %单位运输成本
coldRate=1.3;                       %制冷率
congesteRate=1.5;                   %拥堵率
goodLossRate = 0.39;                %货损率
openDoorCost = 30;                  %一次开门费用
openDoorCostRate = 0.26;            %开门费率
v1=60;                              %车辆行驶速度60Km/h=1km/min
DemandMax=200;                      %车辆最大载重量2000kg

%% 4.计算城市间相互距离
Startdistance=calculate_Startdistance(Xstart,citys);    %计算出发点到各个点之间的距离
D=calculateD(citys);                                    %计算各个目的地之间的距离

%% 5.产生初始粒子
% 5.1随机产生粒子初始位置和速度
for i=1:m
    pop(i,:)=randperm(n);
    v(i,:)=randperm(n);
end

% 5.2计算粒子适应度函数值
for i=1:m

      fitness(i) = Fitness(pop(i,:),Fixedcosts,nuitTransCost,coldRate,congesteRate,goodLossRate,openDoorCost,openDoorCostRate);
      
end

% 5.3计算个体极值和群体极值
Pbest_fitness=fitness;
Pbest=pop;
[Gbest_fitness(1),min_index]=min(fitness);
Gbest(1,:)=pop(min_index,:);
Length_ave(1)=mean(fitness);

%% 6.迭代寻优
while gen<genmax
    % 6.1更新迭代次数与惯性因子
    gen=gen+1;
    w = ws - (ws-we)*(gen/genmax)^2;

    % 6.2更新速度
    %个体极值修正部分
    change1=position_minus_position(Pbest,pop);
    change1=constant_times_velocity(c1,change1);
    %群体极值修正部分
    change2=position_minus_position(repmat(Gbest(gen-1,:),m,1),pop);
    change2=constant_times_velocity(c2,change2);
    %原速度部分
    v=constant_times_velocity(w,v);
    %修正速度
    for i=1:m
        for j=1:n
            if change1(i,j)~=0
                v(i,j)=change1(i,j);
            end
            if change2(i,j)~=0
                v(i,j)=change2(i,j);
            end
        end
    end

    % 6.3更新位置
    pop=position_plus_velocity(pop,v);

    % 6.4适应度函数值更新
    fitness=zeros(m,1);
    for i=1:m
%         for j=1:n-1
%             fitness(i)=fitness(i) + D(pop(i,j),pop(i,j+1));
%         end
%         fitness(i)=fitness(i) + D(pop(i,end),pop(i,1));
        fitness(i) = Fitness(pop(i,:),Fixedcosts,nuitTransCost,coldRate,congesteRate,goodLossRate,openDoorCost,openDoorCostRate);
    end

    % 6.5个体极值与群体极值更新
    for i=1:m
        if fitness(i)<Pbest_fitness(i)
            Pbest_fitness(i)=fitness(i);
            Pbest(i,:)=pop(i,:);
        end
    end

    [minvalue,min_index]=min(fitness);
    if minvalue<Gbest_fitness(gen-1)
        Gbest_fitness(gen)=minvalue;
        Gbest(gen,:)=pop(min_index,:);
    else
        Gbest_fitness(gen)=Gbest_fitness(gen-1);
        Gbest(gen,:)=Gbest(gen-1,:);
    end

    Length_ave(gen)=mean(fitness);

end

%% 7.结果显示
[Shortest_Length,index] = min(Gbest_fitness);
Shortest_Route = Gbest(index,:);
BestChrom = Shortest_Route;
disp(['最低成本:' num2str(Shortest_Length)]);
fprintf('车辆固定成本：%.2f\n运输成本：%.2f\n货损成本：%.2f\n制冷成本：%.2f\n惩罚成本：%.2f\n', Cost(1),Cost(2),Cost(3),Cost(4),Cost(5))

%% 8.绘图
figure(101)
plot(1:genmax,Gbest_fitness,'b')
legend('最低费用')
xlabel('迭代次数')
ylabel('距离')
title('各代最短距离与平均距离对比')
disp('最短配送路径：');
Num=max(Matrix(2,:));
XX=BestChrom+1;
way=[1];
DDDD=cell(Num,1);
for i=1:Num
    A=find(Matrix(2,:)==i);
    way=[way  XX(A) 1];
    DDDD{i,1}=[0 BestChrom(A) 0];
    manzailv(1,i)=Matrix(3,A(end))/DemandMax;
end
DrawPath(way,citys,Xstart)
for i=1:Num
    p=OutputPath(DDDD{i,1});
end
disp('每辆车的满载率为')
disp(manzailv)
