mu = [0 0];
b=1.1;
a=4;
c=1;
Sigma = [a b; b c];
x1 = -5:.2:5; x2 = -5:.2:5;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
figure,
surf(x1,x2,F);

angle=asin(2*b*((a^2)*(c^2)-b^2)/(c^2-a^2))/2/pi*180