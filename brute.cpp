/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */


// Author : Rudresh Singh

#include <bits/stdc++.h>
#include <algorithm>
using namespace std;
#define F first
#define S second
#define mp make_pair
#define pb push_back
#define x "\n"


double dist(pair<double,double> p1,pair<double,double> p2)
{

  double dist;
  dist = 1.0*sqrtl((p1.F-p2.F)*(p1.F-p2.F) + (p1.S-p2.S)*(p1.S-p2.S));
  return dist;
}

// bool incircle(double a1,double a2, double b1,double b2,double r)
// {
//   double a,b,c;
//   pair<double,double> p1,p2;
//   p1=mp(a1,a2);
//   p2=mp(b1,b2);
//   if(dist(p1,p2)<=r)
//   return true;
//   else
//   return false;
// }
int main()
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx;
vector<pair<double,double>> v1,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> v2;
srand(time(0));
 int num_points = 500000; 

    for(int i=0;i<num_points;i++)
    {
        double random_x = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
        double random_y = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
        v1.pb(mp(random_x, random_y));
    }
v.pb(mp(2,3));
v.pb(mp(2,-4));
v.pb(mp(-6,3));
v.pb(mp(-6,-3));
v.pb(mp(-4,1));
v.pb(mp(1,-2));

// for(i=0;i<v1.size();i++)
// cout<<v1[i].F<<" "<<v1[i].S<<x;
// cout<<x;
mx=0;
mxx=INT_MAX;
b=0;
for(i=0;i<v1.size();i++)
{
    for(r=0;r<=5;r=r+0.03)
    {
        mx=0;
        for(j=0;j<v.size();j++)
        {
            a=abs(dist(v1[i],v[j]) - r);
            mx=max(mx,a);
        }
        if(mx<mxx)
        {
            mxx=mx;
            v3.pb(mp(r,mxx));
            p1=v1[i];
        }
    }

}
cout<<v3[v3.size()-1].F<<"    "<<p1.F<<" "<<p1.S<<x;




}
