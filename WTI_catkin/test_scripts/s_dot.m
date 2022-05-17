syms psi theta phi psid thetad phid x y z px py pz

p_w=[px; py; pz];


R(1,1)=cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta);

R(2,1)=cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta);

R(3,1)=-cos(phi)*sin(theta);


R(1,2)=-cos(phi)*sin(psi);

R(2,2)=cos(phi)*cos(psi);

R(3,2)=sin(phi);


R(1,3)=cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);

R(2,3)= sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);


R(3,3)= cos(phi)*cos(theta);


x_w=[x;y;z];
b_1=[1;0;0];

b_w=R*b_1;
n_w=p_w-x_w;
n_w=n_w/norm(n_w) ;



s=b_w(1)*n_w(1)+b_w(2)*n_w(2)+b_w(3)*n_w(3);


s_d=diff(s,psi)*psid+diff(s,theta)*thetad+diff(s,phi)*phid


disp(R)

