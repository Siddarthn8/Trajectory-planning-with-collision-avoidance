% DH_parameters of Powerball
% Copyrights : HILS lab.
%
% Contributor:- Krushang Gabani ,Amir Memar , Sri Sadan ,Ehsan T. Esfahani
% Do not share this code file with anyone without permission.

% Input = joint values
% Output = DH Parameters 

function [a,d,alpha,theta] = DH_parameters(q)


a       =   [   0     0.350     0       0       0       0    ];
d       =   [   0.205 0         0       0.305   0       0.075];     
alpha   =   [   90    180   90       -90      90   0 ];
theta   =   [   q(1)     q(2)+pi/2  q(3)+pi/2 q(4)      q(5)      q(6)   ];


end