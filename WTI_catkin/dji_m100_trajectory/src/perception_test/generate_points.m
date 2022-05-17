


psi=[0 0 0 30 0 60 0 90 0 120 0 150 0 180];

psi=psi*(pi/180);
px=7*cos(psi);
py=7*sin(psi);

nx= -cos(psi);
ny=-sin(psi);
writematrix(px', "px.txt");
writematrix(py', "py.txt");

writematrix(nx', "nx.txt");
writematrix(ny', "ny.txt");