function [t,q,qd,qdd] = fun_trajectory_derivative(t,q,qd)
% Obtain trajectory derivative. from [t,q] or [t,q,qd] to [t,q,qd,qdd].

len = length(t);
dof = size(q,2);

if nargin < 3 || isempty(qd)
    qd = zeros(len,dof);
    qdd = zeros(len,dof);
    for i=1:dof
        qd(:,i) = fun_calc_first_derivative(t,q(:,i));
        qdd(:,i) = fun_calc_first_derivative(t,qd(:,i));
    end
else
    qdd = zeros(len,dof);
    for i=1:dof
        qdd(:,i) = fun_calc_first_derivative(t,qd(:,i));
    end
end