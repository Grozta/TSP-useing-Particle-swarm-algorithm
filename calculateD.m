function [D] = calculateD(C)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
[n,column]=size(C);
D=zeros(n,n);
for i=1:n
    for j=1:n
        if i<j
            D(i,j)=sqrt((C(i,1)-C(j,1))^2+(C(i,2)-C(j,2))^2);
        end
    D(j,i)=D(i,j);
    end
end
end