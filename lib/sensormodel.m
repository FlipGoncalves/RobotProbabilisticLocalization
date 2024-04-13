% nonlinear measurement equation sensor model
function [z1]=sensormodel(xym, xstate1, ndnphi)
    % z1 - matrix of new sensor data
    % xym - vector of landmark position
    % xstate1 - vector of robot pose (state)
    % ndnphi - vector of uncertainties in measurement (observation noise)

    z1(1)=sqrt((xym(1)-xstate1(1))^2+(xym(2)-xstate1(2))^2)+ndnphi(1);
    z1(2)=atan2(xym(2)-xstate1(2),xym(1)-xstate1(1))-xstate1(3)+ndnphi(2);
end