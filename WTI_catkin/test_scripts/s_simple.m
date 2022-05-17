syms psi  px py x y
R=[cos(psi) -sin(psi); sin(psi) cos(psi)];

p_w=[px; py];

x_w= [x; y];

b_1=[1 ;0];


b_w=R*b_1;


n_w1=p_w-x_w;
nn=norm(n_w1);
n_w=n_w1/nn ;

s=b_w(1)*n_w(1)+b_w(2)*n_w(2);