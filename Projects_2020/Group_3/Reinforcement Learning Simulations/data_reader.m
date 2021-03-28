clear
clc

fid=fopen('DATA1.txt');
tline = fgetl(fid);

rollout_return=[];
total_steps=[];

train_loss_actor=[];
train_loss_critic=[];

reference_Q_mean=[]
reference_Q_std=[]
while ischar(tline) % Read file untill it ends
    % 'rollout/return'
    search_result=regexp(tline, 'rollout/return ','names');
     if isempty(search_result) ==  0 
         line_places  = strfind(tline,'|');
         rollout_return(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
     % 'total/steps'
     
     search_result=strfind(tline, 'total/steps ');
    
     if isempty(search_result) ==  0
         line_places  = strfind(tline,'|');
         total_steps(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
     %  train/loss_actor
     
     search_result=strfind(tline, 'train/loss_actor ');
    
     if isempty(search_result) ==  0
         line_places  = strfind(tline,'|');
         train_loss_actor(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
     %  train/loss_critic
     
     search_result=strfind(tline, 'train/loss_critic ');
    
     if isempty(search_result) ==  0
         line_places  = strfind(tline,'|');
         train_loss_critic(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
     %  reference_Q_mean
     
     search_result=strfind(tline, 'reference_Q_mean ');
    
     if isempty(search_result) ==  0
         line_places  = strfind(tline,'|');
         reference_Q_mean(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
     %  reference_Q_std
     
     search_result=strfind(tline, 'reference_Q_std ');
    
     if isempty(search_result) ==  0
         line_places  = strfind(tline,'|');
         reference_Q_std(end+1)=str2double(extractBetween(tline,line_places(2)+1,line_places(3)-1));
     end
     
    tline = fgetl(fid); %Jumps to the next line
end



figure(1)
scatter(total_steps,rollout_return)
title('reward vs learning steps')
xlabel('learning steps')
ylabel('price')
grid on

figure(2)
scatter(total_steps,train_loss_actor)
title('actor loss vs learning steps')
xlabel('learning steps')
ylabel('actor loss')
grid on

figure(3)
scatter(total_steps,train_loss_critic)
title('critic loss vs learning steps')
xlabel('learning steps')
ylabel('critic loss')
grid on

figure(4)
scatter(total_steps,reference_Q_mean)
title('mean of Q value vs learning steps')
xlabel('learning steps')
ylabel('mean of Q value')
grid on

figure(5)
scatter(total_steps,reference_Q_std)
title('std of Q value vs learning steps')
xlabel('learning steps')
ylabel('std of Q value')
grid on