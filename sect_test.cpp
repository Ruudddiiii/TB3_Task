/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */

// Author : Rudresh Singh


#include <bits/stdc++.h>
using namespace std;
#define x "\n"
#define pb push_back
#define mp make_pair
#define mod 1000000007
#define B begin()
#define E end()
#define F first
#define S second
#define mp make_pair
#define pie 3.1415926535897932384626433832795
#define ll  long long

double dist(pair<double,double> p1,pair<double,double> p2)
{

  double dist;
  dist = 1.0*sqrtl((p1.F-p2.F)*(p1.F-p2.F) + (p1.S-p2.S)*(p1.S-p2.S));
  return dist;
}
double poww(double a, double b)
{
  int i;
  double c=1.0;
  for(i=0;i<b;i++)
  {
    c=c*a;

  }
  return c;
}


int main()
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,d,min,radi,radi1;
vector<pair<double,double>> v1,v2,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3;

x1=2.0;  // centre of different cirles
y1=-1.0;
x2=-2.0;
y2=2.0;
x3=4.0;
y3=2.0;
x4=3.0;
y4=5.0;
x5=4.0;
y5=5.0;
p1=mp(x1,y1);
p2=mp(x2,y2);
p3=mp(x3,y3);
p4=mp(x4,y4);
p5=mp(x5,y5);
v.pb(p1);
v.pb(p2);
v.pb(p3);
v.pb(p4);
v.pb(p5);
min=0;
for(i=0;i<v.size();i++)
{
  if(dist(mp(0.0,0.0),v[i])>min)
  {min=dist(mp(0.0,0.0),v[i]);
  point1=v[i];}
}
cout<<"The farthest point to origin : \n";
cout<<point1.F<<" "<<point1.S<<x;
cout<<"Distance is of this point to origin : \n";
cout<<fixed<<min<<x;
double mx =0;
double x3_net,y3_net;
for(i=0;i<v.size();i++)
{
  if(v[i]==point1)
  continue;
  else
  {
    
    x1=point1.F;
    y1=point1.S;
    x2=v[i].F;
    y2=v[i].S;

    radi1 = -(sqrtl((x1*x1 + y1*y1))*(x1*x1 - 2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2))/(2*(-x1*x1 + x2*x1 - y1*y1 + y2*y1));

    if(radi1 >= mx )
    {mx=radi1;
    x3_net = (x1*(x1*x1+y1*y1-y2*y2-x2*x2))/(2*(x1*x1+y1*y1-x2*x1-y1*y2));
    y3_net = (y1/x1)*(x3_net);
    point2=mp(v[i].F,v[i].S);
    }


  }

}
cout<<"The second point is : \n";
cout<<fixed<<point2.F<<" "<<point2.S<<x;
cout<<"New centre is : \n";
cout<<fixed<<x3_net<<" "<<y3_net<<x;
if((point1.F+point2.F)/2 == x3_net && (point1.S+point2.S)/2 == y3_net)
{
  cout<<"No further iteration required and the centre is : \n";
  cout<<fixed<<x3_net<<" "<<y3_net<<x;
}
else 
{
  double a,b,c,d,minn=0;
  pair<double,double> p1;
  d=0.0;
  a = (point1.F+point2.F)/2;
  b = (point1.S+point2.S)/2;
  for(i=0;i<v.size();i++)
  {
    c = dist(mp(a,b),mp(v[i].F,v[i].S));
    if(c>dist(mp(point1.F,point2.F),mp(point1.S,point2.S))/2)
    {
        if(c>minn)
        {
            p1=mp(v[i].F,v[i].S);
            minn=c;

        }
      d=9.0;

    }
  }
//   cout<<p1.F<<" "<<p1.S<<x;
  if(d==0.0)
  {
    cout<<"No further iteration required and the centre is : \n";
   cout<<fixed<<a<<" "<<b<<x;

  }
  else
  {
    double x5_net,y5_net,m,c,avgp_x,avgp_y,distt;
    avgp_x = (point1.F+point2.F)/2;
    avgp_y = (point1.S+point2.S)/2;
    m = (avgp_y - y3_net)/(avgp_x - x3_net);
    c = y3_net - x3_net*m;
    x1=point1.F;
    y1=point1.S;
    x5_net = -(- x1*x1 + p1.F*p1.F - y1*y1 + 2*c*y1 + p1.S*p1.S  - 2*c*p1.S )/(2*(x1 - p1.F + m*y1 - m*p1.S));
    y5_net = m*(x5_net - x3_net) + y3_net;
    distt = dist(mp(x5_net,y5_net),mp(p1.F,p1.S));
    cout.precision(15);
    cout<<"further iteration was needed it's new min radi is : \n";
    cout<<distt<<x;
    cout<<"And the new centre is\n";
    cout<<x5_net<<" "<<y5_net<<x;
  
}
 


}
}
