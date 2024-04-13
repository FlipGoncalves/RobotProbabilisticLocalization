% nonlinear discrete time dynamic system motion model
function [xstatet1] = motionmodel(xstate,Vin,Dn,t)
    % xstatet1 - the next robot state (pose) (at t+1)
    % xstate - the current (x,y,th) pose (at instant t)
    % Vin - the linear and angular velocities (V,w)
    % Dn - the uncertainty (errors) in velocities (dV, dw)
    % t - the sampling time

    xstatet1(1)=xstate(1)+(Vin(1)+Dn(1))*t*cos(xstate(3));
    xstatet1(2)=xstate(2)+(Vin(1)+Dn(1))*t*sin(xstate(3));
    xstatet1(3)=xstate(3)+(Vin(2)+Dn(2))*t;
end