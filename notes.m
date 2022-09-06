% notes on observer March 31, 2021
% system must be observable in order to form observer of unmeasured states
% 
%% parameters
J= 5.87E-6; % MKS
Lm= 300E-6;
Rm= 2.4;
kt= 0.0057; % MKS
kb = kt;
kv= 2E-6;   % MKS
Ksmc= 50.0;

%% Plant
% states are motor current (im) and velocity (omega)
% and input voltage Vm
% c.t. system
% dim/dt  = (Vm - kb*omega - Rm*im) /L
% domega/dt = (kt*im -kv*omega)/J
% x1= im, x2= omega
% |x1d| = | -Rm/L     -kb/L | |x1|  + |1/L| *Vm
% |x2d|   |  kt/j     -kv/J | |x2|    |0  |
%
A = [-Rm/Lm -kb/Lm; 
      kt/J -kv/J];
B = [1/Lm; 0];
C = [1 0;
     0 kb];  % bemf not available but will display in s
D = 0;
sysc = ss(A,B,C,D);
bode(A,B,C,D);

Ts= 50E-6;  % sample rate 20kHz PWM
sysd = c2d(sysc, Ts);

co = obsv(sysd.a, sysd.c);
result = rank(co);

%% Observer
% Since the plant is observable (co full rank) we can design an observer of
% the bemf (which is proportional to velocity) using motor current as the
% measurement.

Ad= sysd.a;
Bd= sysd.b;
Cd= sysd.c;
Dd= sysd.d;

% Observer equation is
% x(k+1) = Ad*x(k) + Bd*u(k) + L*(y(k) - Ye(k))
% y(k)   = Cd*x(k) + Dd*u(k)
%
% to use Simulink state space block write modified B as:
% x(k+1) = Ad*x(k) + |B1 L1 |u  |
%                    |B2 L2||err|

% eigen values of open loop discrete system are: 0.670397 & 0.99987
% so place observer poles at 0.98 & 0.99 as test.

Cob = Cd(1,:);
L= place(Ad', Cob', [0.98, 0.99000])';
L    % display

% modified B for augmented u input in ss model.
Bmod = [Bd(1) L(1);
        Bd(2) L(2)];

Dd = [0 0;
      0 0];






