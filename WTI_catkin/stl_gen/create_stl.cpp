#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <string>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iterator>
#include <numeric>
using namespace std;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib> // for exit functio
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

/*solid AssimpScene
 facet normal 0 0 -1
  outer loop
  vertex  0  0 0
  vertex 0   1 0
  vertex  1 0 0
  endloop
 endfacet
*/
int n1=0;
int n2=0;
int n3=0;

float x[2000][3];
int up;

int H=120;
float w=3;
float t=2;
float l=2;
float b_w=2.7;
float b_l=40;
float b_t=b_w/4;
float t_c=120*(2.0/3);
float dh=0;
//float T[4][4];
//float Rx[4][4];
//float Ry[4][4];
//float Rz[4][4];



int main()
{

x[0][0]=-w/2;
x[0][1]=-t/2;
x[0][2]=-l/2;
x[1][0]=-w/2;
x[1][1]=-t/2;
x[1][2]=l/2;
x[2][0]=w/2;
x[2][1]=-t/2;
x[2][2]=-l/2;


x[3][0]=w/2;
x[3][1]=-t/2;
x[3][2]=l/2;
x[4][0]=-w/2;
x[4][1]=-t/2;
x[4][2]=l/2;
x[5][0]=w/2;
x[5][1]=-t/2;
x[5][2]=-l/2;
















ofstream myfile;
myfile.open ("turbine.stl");

myfile << "solid AssimpScene.\n";


/*
myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[0][0];
myfile << " ";
myfile <<x[0][1];
myfile << " ";
myfile <<x[0][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[1][0];
myfile << " ";
myfile <<x[1][1];
myfile << " ";
myfile <<x[1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[2][0];
myfile << " ";
myfile <<x[2][1];
myfile << " ";
myfile <<x[2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";




myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[3][0];
myfile << " ";
myfile <<x[3][1];
myfile << " ";
myfile <<x[3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[4][0];
myfile << " ";
myfile <<x[4][1];
myfile << " ";
myfile <<x[4][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[5][0];
myfile << " ";
myfile <<x[5][1];
myfile << " ";
myfile <<x[5][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";



int cc=0;

int count=0;

for (int i = 1; i < 2; i++){
for (int j = 0; j < H-1; j++){


if (count==0)
{
 n2=0;
 n1=1;
 n3=0;
}

if (count==1)
{
 n1=0;
 n2=1;
 n3=0;
}

if (count==2)
{
 n2=0;
 n1=-1;
 n3=0;
}

if (count==3)
{
 n2=-1;
 n1=0;
 n3=0;
 count=-1;
}





x[6+(3*j)][0]=x[3][0]*cos((M_PI/2)*(j+1)) - x[3][1]*sin((M_PI/2)*(j+1));
x[6+(3*j)][1]=x[3][0]*sin((M_PI/2)*(j+1)) + x[3][1]*cos((M_PI/2)*(j+1));
x[6+(3*j)][2]=x[3][2]+dh;
x[7+(3*j)][0]=x[4][0]*cos((M_PI/2)*(j+1)) - x[4][1]*sin((M_PI/2)*(j+1));
x[7+(3*j)][1]=x[4][0]*sin((M_PI/2)*(j+1)) + x[4][1]*cos((M_PI/2)*(j+1));
x[7+(3*j)][2]=x[4][2]+dh;
x[8+(3*j)][0]=x[5][0]*cos((M_PI/2)*(j+1)) - x[5][1]*sin((M_PI/2)*(j+1));
x[8+(3*j)][1]=x[5][0]*sin((M_PI/2)*(j+1)) + x[5][1]*cos((M_PI/2)*(j+1));
x[8+(3*j)][2]=x[5][2]+dh;


x[9+(3*j)][0]=x[0][0]*cos((M_PI/2)*(j+1)) - x[3][1]*sin((M_PI/2)*(j+1));
x[9+(3*j)][1]=x[0][0]*sin((M_PI/2)*(j+1)) + x[3][1]*cos((M_PI/2)*(j+1));
x[9+(3*j)][2]=x[0][2]+dh;
x[10+(3*j)][0]=x[1][0]*cos((M_PI/2)*(j+1)) - x[4][1]*sin((M_PI/2)*(j+1));
x[10+(3*j)][1]=x[1][0]*sin((M_PI/2)*(j+1)) + x[4][1]*cos((M_PI/2)*(j+1));
x[10+(3*j)][2]=x[1][2]+dh;
x[11+(3*j)][0]=x[2][0]*cos((M_PI/2)*(j+1)) - x[5][1]*sin((M_PI/2)*(j+1));
x[11+(3*j)][1]=x[2][0]*sin((M_PI/2)*(j+1)) + x[5][1]*cos((M_PI/2)*(j+1));
x[11+(3*j)][2]=x[2][2]+dh;
if ((j-2)%4==0 or j==2)
{
dh=dh+4;
}


count=count+1;

myfile << "facet normal";
myfile << " ";
myfile <<n1;
myfile << " ";
myfile <<n2;
myfile << " ";
myfile <<n3;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[6+(3*j)][0];
myfile << " ";
myfile <<x[6+(3*j)][1];
myfile << " ";
myfile <<x[6+(3*j)][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[7+(3*j)][0];
myfile << " ";
myfile <<x[7+(3*j)][1];
myfile << " ";
myfile <<x[7+(3*j)][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[8+(3*j)][0];
myfile << " ";
myfile <<x[8+(3*j)][1];
myfile << " ";
myfile <<x[8+(3*j)][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";


myfile << "facet normal";
myfile << " ";
myfile <<n1;
myfile << " ";
myfile <<n2;
myfile << " ";
myfile <<n3;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[9+(3*j)][0];
myfile << " ";
myfile <<x[9+(3*j)][1];
myfile << " ";
myfile <<x[9+(3*j)][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[10+(3*j)][0];
myfile << " ";
myfile <<x[10+(3*j)][1];
myfile << " ";
myfile <<x[10+(3*j)][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[11+(3*j)][0];
myfile << " ";
myfile <<x[11+(3*j)][1];
myfile << " ";
myfile <<x[11+(3*j)][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";

cc=cc+1;

}
}

*/

int cc;
cc=0;


x[cc][0]=w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c-b_w;



//int theta=M_PI/5;
//int beta=M_PI/2 - theta;

float m_s=1.5;



for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



///side elements

int k=b_l/m_s;


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]-b_w*0.8+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]-m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";








///


cc=0;
x[cc][0]=w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c-b_w;






for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



//side element 2




















////////////////////




cc=0;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c-b_w;




//int theta=M_PI/5;
//int beta=M_PI/2 - theta;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}

//side element 2

cc=0;
k=b_l/m_s;


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0]+m_s*cos(M_PI/5);
myfile << " ";
myfile <<x[cc+2*k+2][1]+b_t;
myfile << " ";
myfile <<x[cc+2*k+2][2]-b_w*0.8+m_s*sin(M_PI/5);
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";




cc=500;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c-b_w;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



//inclined surfaces//////////////



cc=0;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]-m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]-m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}








///inclined 2




cc=0;
x[cc][0]=w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0]+m_s*cos(M_PI/5);
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]-m_s*sin(M_PI/5);


x[cc+2*k+1][0]=x[cc+2*k-1][0]+m_s*cos(M_PI/5);
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]-m_s*sin(M_PI/5);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<sin(M_PI/5);
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<cos(M_PI/5);
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




























////////////vertical 


cc=300;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=-b_t/2;
x[cc+1][2]=t_c;




//int theta=M_PI/5;
//int beta=M_PI/2 - theta;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




cc=0;
x[cc][0]=-w/2;
x[cc][1]=b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=+w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}




//// vertical 2



cc=300;
x[cc][0]=-w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=-w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;




//int theta=M_PI/5;
//int beta=M_PI/2 - theta;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<-1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";

myfile << "endloop\n";
myfile << "endfacet\n";
}




cc=0;
x[cc][0]=w/2;
x[cc][1]=-b_t/2;
x[cc][2]=t_c;

x[cc+1][0]=w/2;
x[cc+1][1]=b_t/2;
x[cc+1][2]=t_c;





for (int k = 1; k < (b_l+10)/m_s; k++){

x[cc+2*k][0]=x[cc+2*k-2][0];
x[cc+2*k][1]=x[cc+2*k-2][1];
x[cc+2*k][2]=x[cc+2*k-2][2]+m_s*sin(M_PI/2);


x[cc+2*k+1][0]=x[cc+2*k-1][0];
x[cc+2*k+1][1]=x[cc+2*k-1][1];
x[cc+2*k+1][2]=x[cc+2*k-1][2]+m_s*sin(M_PI/2);
}


for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k][0];
myfile << " ";
myfile <<x[cc+2*k][1];
myfile << " ";
myfile <<x[cc+2*k][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}



for (int k = 0; k < b_l/m_s; k++){


myfile << "facet normal";
myfile << " ";
myfile <<1;
myfile << " ";
myfile <<0;
myfile << " ";
myfile <<0;
myfile << " ";
myfile << "\n";
myfile << "outer loop\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+1][0];
myfile << " ";
myfile <<x[cc+2*k+1][1];
myfile << " ";
myfile <<x[cc+2*k+1][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+3][0];
myfile << " ";
myfile <<x[cc+2*k+3][1];
myfile << " ";
myfile <<x[cc+2*k+3][2];
myfile << "\n";
myfile << "vertex ";
myfile << " ";
myfile <<x[cc+2*k+2][0];
myfile << " ";
myfile <<x[cc+2*k+2][1];
myfile << " ";
myfile <<x[cc+2*k+2][2];
myfile << "\n";
myfile << "endloop\n";
myfile << "endfacet\n";
}
























myfile<< "endsolid AssimpScene";

myfile.close();




return 0;
}






    // Passing all the variables inside the vector from the beginning of the vector to the end.
//scopy( lv.begin( ), lv.end( ), output_iterator );