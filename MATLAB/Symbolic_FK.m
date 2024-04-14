sympref('FloatingPointOutput',true);
syms t1 t2 t3 t4 t5 t6 a2 a3 d1 d4 d6 

th1 = t1; %z1 = z1 + pi/2
th2 = t2; %z2 = z2 - pi/2
th3 = t3;
th4 = t4;
th5 = t5;
th6 = t6;

m1 = -pi/2;
m2 = 0;
m3 = -pi/2;
m4 = -pi/2;
m5 = pi/2;
m6 = 0;

a_1 = 0;
a_2 = a2; %0.27;
a_3 = a3; %0.07;
a_4 = 0;
a_5 = 0;
a_6 = 0;

d_1 = d1; %0.103;
d_2 = 0;
d_3 = 0;
d_4 = d4; %0.302;
d_5 = 0;
d_6 = d6; %0.072;

H1 = [cos(th1) 0 -sin(th1) a_1*cos(th1);
    sin(th1) 0 cos(th1) a_1*sin(th1);
    0 -1 0 d_1;
    0 0 0 1];

H2 = [cos(th2) -sin(th2) 0 a_2*cos(th2);
    sin(th2) cos(th2) 0 a_2*sin(th2);
    0 0 1 d_2;
    0 0 0 1];

H3 = [cos(th3) 0 -sin(th3) a_3*cos(th3);
    sin(th3) 0 cos(th3) a_3*sin(th3);
    0 -1 0 d_3;
    0 0 0 1];

H4 = [cos(th4) 0 -sin(th4) a_4*cos(th4);
    sin(th4) 0 cos(th4) a_4*sin(th4);
    0 -1 0 d_4;
    0 0 0 1];

H5 = [cos(th5) 0 sin(th5) a_5*cos(th5);
    sin(th5) 0 -cos(th5) a_5*sin(th5);
    0 1 0 d_5;
    0 0 0 1];

H6 = [cos(th6) -sin(th6) 0 a_6*cos(th6);
    sin(th6) cos(th6) 0 a_6*sin(th6);
    0 0 1 d_6;
    0 0 0 1];

HT1 = simplify(H1*H2*H3);
HT2 = simplify(H4*H5*H6);

For_Kin = simplify(HT1*HT2);

