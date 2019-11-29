function [Rot trans T03] = rotation(transmission)
syms t
ca1 = cos(transmission.t01(4:4));
sa1 = sin(transmission.t01(4:4));
a1 = transmission.t01(3:3);
d1 =transmission.t01(2:2);

ca2 = (cos(transmission.t12(4:4)));
sa2 = (sin(transmission.t12(4:4)));
a2 = transmission.t12(3:3);
d2 = transmission.t12(2:2);

ca3 = (cos(transmission.t23(4:4)));
sa3 = (sin(transmission.t23(4:4)));
a3 = transmission.t23(3:3);
d3 = transmission.t23(2:2);

ct1  = (cos(transmission.t01(1:1))); 
st1 = (sin(transmission.t01(1:1)));

ct2  = (cos(transmission.t12(1:1))); 
st2 = (sin(transmission.t12(1:1))); 

ct3  = (cos(transmission.t23(1:1)));
st3  = (sin(transmission.t23(1:1)));

T01 = [ct1 -st1.*ca1 st1.*sa1 a1.*ct1; st1 ct1.*ca1 -ct1.*sa1 a1.*st1; 0 sa1 ca1 d1; 0 0 0 1];
T12 = [ct2 -st2*ca2 st2*sa2 a2*ct2; st2 ct2*ca2 -ct2*sa2 a2*st2; 0 sa2 ca2 d2; 0 0 0 1];
T23 = [ct3 -st3*ca3 st3*sa3 a3*ct3; st3 ct3*ca3 -ct3*sa3 a3*st3; 0 sa3 ca3 d3; 0 0 0 1];
T02 = T01*T12;
T03 = T02*T23;

Rot.r01 = T01(1:3,1:3);
Rot.r12 = T12(1:3,1:3);
Rot.r23 = T23(1:3,1:3);
Rot.r02 = T02(1:3,1:3);
Rot.r03 = T03(1:3,1:3);

trans.o01 = T01(1:3,4);
trans.o12 = T12(1:3,4);
trans.o23 = T23(1:3,4);
trans.o02 = T02(1:3,4);
trans.o03 = T03(1:3,4);


end