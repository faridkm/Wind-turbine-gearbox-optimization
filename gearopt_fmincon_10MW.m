
%%% This code optimizes the gear ratios and estimates the weight of the gear stages based on different configurations of the 
%%% gearbox by using the model presented in "Nejad, A. R., Guo, Y., Gao, Z., & Moan, T. (2016). Development of a 5 MW 
%%% reference gearbox for offshore wind turbines. Wind Energy, 19(6), 1089-1106."


%%% Gearbox configurations:
%%%One stage (paralel) 
%%%One stage (planetary) 
%%%Two stages (planetary-paralel)
%%%Two stages (planetary-planetary) 
%%%Three stages (planetary-planetary-paralel)   
%%%Three stages (planetary-planetary-planetary)  


%%% An example of the implementation of this code can be found in "Moghadam, F. K., & Nejad, A. R. (2020). Evaluation 
%%% of PMSG?based drivetrain technologies for 10?MW floating offshore wind turbines: Pros and cons in a life cycle perspective. 
%%% Wind Energy, 23(7), 1542-1563."



%% 10MW gearbox optimization
clc
clear all
close all



%% Parameters definition 
power_rated= 10e6; % gearbox nominal power in watt
speed_rated= 9.6*2*pi/60; % gearbox nominal input speed in rad/s
q0=power_rated/speed_rated; %input torque
b1=5; % number of plants for the first planetary stage
b2=5; % number of plants for the second planetary stage
b3=5; % number of plants for the third planetary stage
kr1=0.4; % ring scaling factor for the first planetary stage
kr2=0.4; % ring scaling factor for the second planetary stage
kr3=0.4;
k=4e6; % intensity of tooth loads factor
density=7850; % steel density
weight_cofficient=1.15; % correction cofficient to consider the weight of bearings and housing and lubrication



%%  One stage (paralel)    1<ratio<3
i=1;
for ratio= 1:3

u0=[2];
lb = [1];
ub = [3];
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2.*q0./(k.*u(1))) .* (1+1./u(1)+u(1)+u(1).^2),...
    u0, [],[],[1],[ratio],lb,ub,[],options); 
weight=fval*density;
weight_final_one_stage_paralel{i}=weight*weight_cofficient; 
u_one_stage_paralel{i}=u;
i=i+1;
end
a=cell2table(weight_final_one_stage_paralel'); % weight
b=cell2table(u_one_stage_paralel'); % gear ratio
one_stage_paralel= table(a,b); % weight for different gear ratios


    
%% One stage (planetary)    3<ratio<6
i=1;
for ratio= 3:6 % resolu:0.5
u0=[5];
lb = [3];
ub = [6];
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2*q0./(k.*u(1))) .* (1/b1 + 1./(b1.*(u(1)./2-1)) + (u(1)./2-1) + (u(1)./2-1).^2 + kr1.*(u(1)-1).^2./b1 + kr1.*(u(1)-1).^2./(b1.*(u(1)./2-1))),...
    u0, [],[],[1],[ratio],lb,ub,[],options); 
weight=fval*density;
weight_final_one_stage_planetary{i}=weight*weight_cofficient; 
u_one_stage_planetary{i}=u;
i=i+1;
end    
a=cell2table(weight_final_one_stage_planetary');
b=cell2table(u_one_stage_planetary');
one_stage_planetary= table(a,b);  % weight for different gear ratios
    


%% Two stages (planetary-paralel)    6<ratio<18
i=1;
global ratio_two_stage
for ratio_two_stage= 6:18
u0=[5 2];
lb = [3,1];
ub = [6,6];
%nonlcon= 
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2*q0./(k.*u(1))) .* (1/b1 + 1./(b1.*(u(1)./2-1)) + (u(1)./2-1) + (u(1)./2-1).^2 + kr1.*(u(1)-1).^2./b1 + kr1.*(u(1)-1).^2./(b1.*(u(1)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2))) .* (1+1./u(2)+u(2)+u(2).^2), u0, [],[],[],[],lb,ub,@nonconst_two_stage,options);
weight=fval*density;
weight_final_two_stage_planetary_paralel{i}=weight*weight_cofficient; 
u_two_stage_planetary_paralel{i}=u;
i=i+1;
end 
a=cell2table(weight_final_two_stage_planetary_paralel');
b=cell2table(u_two_stage_planetary_paralel');
two_stage_planetary_paralel= table(a,b);  % weight for different gear ratios



%% Two stages (planetary-planetary)    18<ratio<36
i=1;
for ratio_two_stage= 18:36
u0=[5 5];
lb = [3,3];
ub = [6,6];
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2*q0./(k.*u(1))) .* (1/b1 + 1./(b1.*(u(1)./2-1)) + (u(1)./2-1) + (u(1)./2-1).^2 + kr1.*(u(1)-1).^2./b1 + kr1.*(u(1)-1).^2./(b1.*(u(1)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2))) .* (1/b2 + 1./(b2.*(u(2)./2-1)) + (u(2)./2-1) + (u(2)./2-1).^2 + kr2.*(u(2)-1).^2./b2 + kr2.*(u(2)-1).^2./(b2.*(u(2)./2-1))),...
    u0, [],[],[],[],lb,ub,@nonconst_two_stage,options);
weight=fval*density;
weight_final_two_stage_planetary_planetary{i}=weight*weight_cofficient; 
u_two_stage_planetary_planetary{i}=u;
i=i+1;
end        
a=cell2table(weight_final_two_stage_planetary_planetary');
b=cell2table(u_two_stage_planetary_planetary');
two_stage_planetary_planetary= table(a,b);



%%  Three stages (planetary-planetary-paralel)     36<ratio<108
i=1;
global ratio_three_stage
for ratio_three_stage= 50 % 36:108
u0=[5 5 3]; 
lb = [3,3,1];
ub = [6,6,5];
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2*q0./(k.*u(1))) .* (1/b1 + 1./(b1.*(u(1)./2-1)) + (u(1)./2-1) + (u(1)./2-1).^2 + kr1.*(u(1)-1).^2./b1 + kr1.*(u(1)-1).^2./(b1.*(u(1)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2))) .* (1/b2 + 1./(b2.*(u(2)./2-1)) + (u(2)./2-1) + (u(2)./2-1).^2 + kr2.*(u(2)-1).^2./b2 + kr2.*(u(2)-1).^2./(b2.*(u(2)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2).*u(3))) .* (1+1./u(3)+u(3)+u(3).^2), u0, [],[],[],[],lb,ub,@nonconst,options);
weight=fval*density;
weight_final_three_stage_planetary_planetary_paralel{i}=weight*weight_cofficient;
u_three_stage_planetary_planetary_paralel{i}=u;
i=i+1;
end
a=cell2table(weight_final_three_stage_planetary_planetary_paralel');
b=cell2table(u_three_stage_planetary_planetary_paralel');
three_stage_planetary_planetary_paralel= table(a,b);



%%  Three stages (planetary-planetary-planetary)     108<ratio<216
i=1;
for ratio_three_stage= 156 %108:216
u0=[5 5 5]; 
lb = [3,3,3];
ub = [6,6,6];
options = optimoptions(@fmincon,'Display','iter','Algorithm','SQP','MaxFunEval',inf,'MaxIter',Inf,'ConstraintTolerance',1e-3,'StepTolerance',1e-10,'FunctionTolerance',1e-6);     
[u,fval] = fmincon(@(u) (2*q0./(k.*u(1))) .* (1/b1 + 1./(b1.*(u(1)./2-1)) + (u(1)./2-1) + (u(1)./2-1).^2 + kr1.*(u(1)-1).^2./b1 + kr1.*(u(1)-1).^2./(b1.*(u(1)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2))) .* (1/b2 + 1./(b2.*(u(2)./2-1)) + (u(2)./2-1) + (u(2)./2-1).^2 + kr2.*(u(2)-1).^2./b2 + kr2.*(u(2)-1).^2./(b2.*(u(2)./2-1)))  +...
    (2.*q0./(k.*u(1).*u(2).*u(3))) .* (1/b3 + 1./(b3.*(u(3)./2-1)) + (u(3)./2-1) + (u(3)./2-1).^2 + kr3.*(u(3)-1).^2./b3 + kr3.*(u(3)-1).^2./(b3.*(u(3)./2-1))), u0, [],[],[],[],lb,ub,@nonconst,options);
weight=fval*density;
weight_final_three_stage_planetary_planetary_planetary{i}=weight*weight_cofficient;
u_three_stage_planetary_planetary_planetary{i}=u;
i=i+1;
end
a=cell2table(weight_final_three_stage_planetary_planetary_planetary');
b=cell2table(u_three_stage_planetary_planetary_planetary');
three_stage_planetary_planetary_planetary= table(a,b);


weight_final_one_stage_paralel;
weight_final_one_stage_planetary;
weight_final_two_stage_planetary_paralel;
weight_final_two_stage_planetary_planetary;
weight_final_three_stage_planetary_planetary_paralel;

one_stage_paralel
one_stage_planetary
two_stage_planetary_paralel
two_stage_planetary_planetary
three_stage_planetary_planetary_paralel
three_stage_planetary_planetary_planetary



%% nonlinear constrain for 2-stages gearbox optimization
function [c,ceq] = nonconst_two_stage(u)
global ratio_two_stage
ceq = u(1)*u(2)-ratio_two_stage;
c = [];
end


%% nonlinear constrain for 3-stages gearboxoptimization
function [c,ceq] = nonconst(u)
global ratio_three_stage
ceq = u(1)*u(2)*u(3)-ratio_three_stage;
c = [];
end


