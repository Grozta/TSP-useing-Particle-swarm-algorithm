%���㷨�ĺ�����Դ�ڶ���Ӧ�Ⱥ�λ�õ�ת����γ��ת��
%% 1.��ջ�������
clear;
clc;
close all;
global  Startdistance D Demand Time ServiceTime v1 DemandMax Matrix Cost
%% 2.��������
citys = load('����.txt'); %���ݼ�
Xstart=[40 50];           %������������
Demand=load('������.txt');
Time=load('ʱ�䴰.txt');
ServiceTime=load('ж��ʱ��.txt');
[n,~] = size(citys);
%% 3.��ʼ������
c1=0.1;                         %����ѧϰ����
c2=0.075;                       %���ѧϰ����
w=0.7;                          %��������
m=50;                           %��������
pop=zeros(m,n);                 %����λ��
v=zeros(m,n);                   %�����ٶ�
gen=1;                          %����������
genmax=500;                     %��������
fitness=zeros(m,1);             %��Ӧ�Ⱥ���ֵ
Pbest=zeros(m,n);               %���弫ֵ·��
Pbest_fitness=zeros(m,1);       %���弫ֵ
Gbest=zeros(genmax,n);          %Ⱥ�弫ֵ·��
Gbest_fitness=zeros(genmax,1);  %Ⱥ�弫ֵ
Length_ave=zeros(genmax,1);     %����·����ƽ������
ws=0.8;                         %�����������ֵ
we=0.5;                         %����������Сֵ
Fixedcosts = 150;                   %�̶��ɱ�
nuitTransCost =2.4;                 %��λ����ɱ�
coldRate=1.3;                       %������
congesteRate=1.5;                   %ӵ����
goodLossRate = 0.39;                %������
openDoorCost = 30;                  %һ�ο��ŷ���
openDoorCostRate = 0.26;            %���ŷ���
v1=60;                              %������ʻ�ٶ�60Km/h=1km/min
DemandMax=200;                      %�������������2000kg

%% 4.������м��໥����
Startdistance=calculate_Startdistance(Xstart,citys);    %��������㵽������֮��ľ���
D=calculateD(citys);                                    %�������Ŀ�ĵ�֮��ľ���

%% 5.������ʼ����
% 5.1����������ӳ�ʼλ�ú��ٶ�
for i=1:m
    pop(i,:)=randperm(n);
    v(i,:)=randperm(n);
end

% 5.2����������Ӧ�Ⱥ���ֵ
for i=1:m

      fitness(i) = Fitness(pop(i,:),Fixedcosts,nuitTransCost,coldRate,congesteRate,goodLossRate,openDoorCost,openDoorCostRate);
      
end

% 5.3������弫ֵ��Ⱥ�弫ֵ
Pbest_fitness=fitness;
Pbest=pop;
[Gbest_fitness(1),min_index]=min(fitness);
Gbest(1,:)=pop(min_index,:);
Length_ave(1)=mean(fitness);

%% 6.����Ѱ��
while gen<genmax
    % 6.1���µ����������������
    gen=gen+1;
    w = ws - (ws-we)*(gen/genmax)^2;

    % 6.2�����ٶ�
    %���弫ֵ��������
    change1=position_minus_position(Pbest,pop);
    change1=constant_times_velocity(c1,change1);
    %Ⱥ�弫ֵ��������
    change2=position_minus_position(repmat(Gbest(gen-1,:),m,1),pop);
    change2=constant_times_velocity(c2,change2);
    %ԭ�ٶȲ���
    v=constant_times_velocity(w,v);
    %�����ٶ�
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

    % 6.3����λ��
    pop=position_plus_velocity(pop,v);

    % 6.4��Ӧ�Ⱥ���ֵ����
    fitness=zeros(m,1);
    for i=1:m
%         for j=1:n-1
%             fitness(i)=fitness(i) + D(pop(i,j),pop(i,j+1));
%         end
%         fitness(i)=fitness(i) + D(pop(i,end),pop(i,1));
        fitness(i) = Fitness(pop(i,:),Fixedcosts,nuitTransCost,coldRate,congesteRate,goodLossRate,openDoorCost,openDoorCostRate);
    end

    % 6.5���弫ֵ��Ⱥ�弫ֵ����
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

%% 7.�����ʾ
[Shortest_Length,index] = min(Gbest_fitness);
Shortest_Route = Gbest(index,:);
BestChrom = Shortest_Route;
disp(['��ͳɱ�:' num2str(Shortest_Length)]);
fprintf('�����̶��ɱ���%.2f\n����ɱ���%.2f\n����ɱ���%.2f\n����ɱ���%.2f\n�ͷ��ɱ���%.2f\n', Cost(1),Cost(2),Cost(3),Cost(4),Cost(5))

%% 8.��ͼ
figure(101)
plot(1:genmax,Gbest_fitness,'b')
legend('��ͷ���')
xlabel('��������')
ylabel('����')
title('������̾�����ƽ������Ա�')
disp('�������·����');
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
disp('ÿ������������Ϊ')
disp(manzailv)
