clear all
close all
clc
%% 

% links = struct('a1' , 'd1')
links.a2 = 1;
links.d1 = 1; 

% joints = struct('t1' , 't2','d3')
joints.t1 = 0;
joints.t2 = 0;
joints.d3 = 1;
%%

t1 = deg2rad(joints.t1);
t2 = deg2rad(joints.t2);
d3 = deg2rad(joints.d3);

a2 = links.a2;
d1 = links.d1; 

%% DH-Parameter
syms th1 th2 th3

alp1 = deg2rad(90);
alp2 = deg2rad(-90);
alp3 = deg2rad(0);


transmission.t01 = [th1 d1 0 alp1];
transmission.t12 = [th2 0 a2 alp2];
transmission.t23 = [0  d3 0 alp3];
transmission.notation = ["theta" "alpha"];

%% Rotation matrices 

[rotat trans transform03] = rotation(transmission);
%% 
syms t
q1 = sin(t);
q2 = cos(2*t);
q3 = sin(3*t);
th1 = q1;
th2 = q2;
th3 = q3;
  
d_q1 = diff(q1,t);
dd_q1= diff(d_q1,t);
d_q2 = diff(q2,t);
dd_q2 = diff(d_q2,t);
d_q3 = diff(q3,t);
dd_q3 = diff(d_q3,t);

q = [q1;q2;q3];
d_q = [d_q1;d_q2;d_q3];
dd_q = [dd_q1;dd_q2;dd_q3];

alpha_0 = zeros(3,1); 
omega_0 = zeros(3,1); 
ac_0 = zeros(3,1); 
ae_0 = zeros(3,1); 
% newton_dynamics(Ri,i-1 , Zi-1, ri,i+1 , [q2(t), d_q2(t), dd_q2(t)], omega_i-1, alpha1_i-1, ae1_i-1)
[omega1, alpha1, ae1, ac1,g1,C1, proj1]= newton_dynamics(eye(3),eye(3) , trans.o01, trans.o01 , [q1, d_q1, dd_q1], omega_0, alpha_0, ae_0);
[omega2, alpha2, ae2, ac2,g2,C2, proj2]= newton_dynamics(rotat.r01,rotat.r01 , trans.o02, trans.o12 , [q2, d_q2, dd_q2], omega1, alpha1, ae1);
[omega3, alpha3, ae3, ac3,g3,C3, proj3]= newton_dynamics_p(rotat.r02,rotat.r12 , trans.o03, trans.o23 , [q3, d_q3, dd_q3], omega2, alpha2, ae2);

%% backwards path
m = 1;
Fk = [0;0;0];
Tk = [0;0;0];
I = [1/12 0 0; 0 1/12 0;0 0 1/12];
%newton_ft(Ri+1,i,ri+1,i,omega ,alpha,ac,m,force,torque,I)
[F2, T2, pmag2] = newton_ft(rotat.r23,trans.o23,omega3,alpha3,ac3,g3,m,Fk,Tk,I);
[F1, T1, rmag1] = newton_ft(rotat.r12,trans.o12,omega2,alpha2,ac2,g2,m,F2,T2,I);
[F0, T0, rmag0] = newton_ft(rotat.r01,trans.o01,omega1,alpha1,ac1,g1,m,F1,T1,I);

%% plotting q
syms t 
q1 = sin(t);
q2 = cos(2*t);
q3 = sin(3*t);
th1 = sin(t);
th2 = cos(2*t);
d3 = sin(3*t);

t0 = 0:0.1:10 ;
for i= 1 :length(t0)
    t = t0(i);
    omega_1(i,:)= eval(subs(proj1.omega));
    omega_2(i,:)= eval(subs(proj2.omega));
    omega_3(i,:)= eval(subs(proj3.omega));

    alpha_1(i,:)= eval(subs(proj1.alpha));
    alpha_2(i,:)= eval(subs(proj2.alpha));
    alpha_3(i,:)= eval(subs(proj3.alpha));

    ae_1(i,:)= eval(subs(proj1.ae));
    ae_2(i,:)= eval(subs(proj2.ae));
    ae_3(i,:)= eval(subs(proj3.ae));

    ac_1(i,:)= eval(subs(proj1.ac));
    ac_2(i,:)= eval(subs(proj2.ac));
    ac_3(i,:)= eval(subs(proj3.ac));

    g_1(i,:)= eval(subs(proj1.g));
    g_2(i,:)= eval(subs(proj2.g));
    g_3(i,:)= eval(subs(proj3.g));

    centrifugal1(i) = eval(subs(proj1.C));
    centrifugal2(i) = eval(subs(proj2.C));
    centrifugal3(i)= eval(subs(proj3.C)); 
    
    coriollos3(i)= eval(subs(proj3.coriollos));

    T_0(i,:)= eval(subs(rmag0.T));
    T_1(i,:)= eval(subs(rmag1.T));
    F_2(i,:)= eval(subs(pmag2.F));
   
end

save('dynamics_data.mat')