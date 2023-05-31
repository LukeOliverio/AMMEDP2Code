% System model
A = [-0.02,-90; 0, -0.02];
B = [0; 0.00001];
C=[1 , 0];

% Reachability/Controllability matrix
W = ctrb(A,B);
sys = tf(ss(A,B,C,0));
Wo = obsv(A,C);

% Canonical form
a1=0.04; a2=0.0004;% These are from Characteristic Polynomial (CP)
At = [-a1, -a2; 1, 0];
Bt = [1; 0];

% Reachability/Controllability matrix for canonical form
Wt = ctrb(At, Bt);

% Desirted CP

%Pre tuned transeint characteristics ,ts=110;tr=70; 
%Pre tuned characteristics w=1.8/tr; z=3.92/(w*ts)

%tuned variables
z=0.9679;
w=0.045;
p1=2*z*w; p2=w^2; 
overshoot=exp((-3.142*z)/(1-z^2)^0.5);

% Coefficient matching
Kt = [p1, p2] + At(1, :); 

% Calculate transformation matrix to canonical form
T = Wt/W; 

% Controller gains u = -Kx +kr
K = Kt*T;           %state gain
k=-1/(C/(A-B*K)*B); %feed-forward gain


%Observer gains from desired polynomial explained in report
l1=2*10*w*z-0.02;
l2=-100*w^2*z^2/90;

L=[l1;l2];