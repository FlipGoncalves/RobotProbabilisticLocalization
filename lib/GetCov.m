function [cv]=GetCov(P,x,y)
    s=1;
    k=40; % original: k=20, changed by zhan
    
    if trace(P)<1e-5, r=zeros(2,2);
    else
         %P
         r=real(sqrtm(P));
    end
    for j=1:k+1
         q=2*pi*(j-1)/k;
         cv(:,j)=s*3*r*[cos(q); sin(q)]+[x;y];
    end
end