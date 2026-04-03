function out = dynamics_template(q, qdot, tau, p)
% Double inverted pendulum dynamics template (MATLAB)
% Input:
%   q    = [x; theta1; theta2]
%   qdot = [xdot; theta1dot; theta2dot]
%   tau  = [tau_x; tau_theta1; tau_theta2]
%   p    = struct with fields:
%          m_total, A, B, Cc, I_link1, I_link2, k1, k2
%
% Output struct fields:
%   M, C, G, qddot, xdot, A_lin, B_lin
%
% Example:
%   p = struct('m_total',2.0,'A',0.25,'B',0.10,'Cc',0.05,...
%              'I_link1',0.30,'I_link2',0.12,'k1',4.2,'k2',1.8);
%   q = [0; 0.05; -0.03];
%   qdot = [0.1; 0; 0];
%   tau = [0; 0; 0];
%   out = dynamics_template(q, qdot, tau, p);

th1 = q(2);
th2 = q(3);

M = [p.m_total,           p.A*cos(th1),               p.B*cos(th2);
    p.A*cos(th1),        p.I_link1,                  p.Cc*cos(th1-th2);
    p.B*cos(th2),        p.Cc*cos(th1-th2),          p.I_link2];

% dM/dx = zeros(3)
dM1 = zeros(3,3);

dM2 = [0,                 -p.A*sin(th1),              0;
    -p.A*sin(th1),      0,                          -p.Cc*sin(th1-th2);
    0,                 -p.Cc*sin(th1-th2),         0];

dM3 = [0,                 0,                          -p.B*sin(th2);
    0,                 0,                           p.Cc*sin(th1-th2);
    -p.B*sin(th2),       p.Cc*sin(th1-th2),          0];

dM = cat(3, dM1, dM2, dM3);

% Christoffel-based C(q,qdot)
C = zeros(3,3);
for i = 1:3
    for j = 1:3
        cij = 0;
        for k = 1:3
            cijk = 0.5 * (dM(i,j,k) + dM(i,k,j) - dM(j,k,i));
            cij = cij + cijk * qdot(k);
        end
        C(i,j) = cij;
    end
end

G = [0;
    -p.k1*sin(th1);
    -p.k2*sin(th2)];

qddot = M \ (tau - C*qdot - G);
xdot = [qdot; qddot];

% Linearized model around theta1=theta2=0
M0 = [p.m_total, p.A, p.B;
      p.A, p.I_link1, p.Cc;
      p.B, p.Cc, p.I_link2];

K = diag([0, p.k1, p.k2]);

A_lin = [zeros(3), eye(3);
        -M0\K,    zeros(3)];

B_lin = [zeros(3);
          M0\eye(3)];

out = struct();
out.M = M;
out.C = C;
out.G = G;
out.qddot = qddot;
out.xdot = xdot;
out.A_lin = A_lin;
out.B_lin = B_lin;

end
