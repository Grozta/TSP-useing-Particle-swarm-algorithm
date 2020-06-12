function change = constant_times_velocity(constant,change)
% 以一定概率保留交换序列
for i=1:size(change,1)
    for j=1:size(change,2)
        if rand>constant
            change(i,j)=0;
        end
    end
end
end