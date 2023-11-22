/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */


// Author : Rudresh Singh

#include <bits/stdc++.h>
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

bool incircle(double a1,double a2, double b1,double b2,double r)
{
  double a,b,c;
  pair<double,double> p1,p2;
  p1=mp(a1,a2);
  p2=mp(b1,b2);
  if(dist(p1,p2)<=r)
  return true;
  else
  return false;
}
int main()
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0;
vector<pair<double,double>> v1,v2,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;

 srand(time(0));
 int num_points = 5; 

    for (int i = 0; i < num_points; ++i)
    {
        double random_x = static_cast<double>(rand()) / RAND_MAX * 1000 - 5; 
        double random_y = static_cast<double>(rand()) / RAND_MAX * 10000 - 5; 
        v.pb(mp(random_x, random_y));
    }


for(i=0;i<v.size();i++)
cout<<v[i].F<<" "<<v[i].S<<x;
cout<<x;

double a1,a2;
double mx = INT_MAX;
 
   for(i=0;i<v.size();i++)
   {
    for(j=0;j<v.size();j++)
    {
      if(j==i)
      continue;
      for(k=0;k<v.size();k++)
      {
        if(k==i || k==j)
        continue;
        if(incircle((v[i].F+v[j].F)/2,(v[i].S+v[j].S)/2,v[k].F,v[k].S,dist(v[i],v[j])/2)==false)
        {
          d=9;
          break;
        }
      }
      if(d!=9)
      {
        if(dist(v[i],v[j])/2 < mx)
        {
          mx=dist(v[i],v[j])/2;
          point5 = mp((v[i].F+v[j].F)/2,(v[i].S+v[j].S)/2);
        }

      }
      d=0;
    }
   }
   d=0;
   for(i=0;i<v.size();i++)
   {
    for(j=0;j<v.size();j++)
    {
      if(j==i)
      continue;
      for(k=0;k<v.size();k++)
      {
        if(k==i || k==j)
        continue;
        for(l=0;l<v.size();l++)
        {
          if(l==i || l==j || l==k)
          continue;
          point1 = v[i];
          point2 = v[j];
          point3 = v[k];
          a1 = -(- point1.F*point1.F*point2.S + point1.F*point1.F*point3.S - point1.S*point1.S*point2.S + point1.S*point1.S*point3.S + point1.S*point2.F*point2.F + point1.S*point2.S*point2.S - point1.S*point3.F*point3.F - point1.S*point3.S*point3.S - point2.F*point2.F*point3.S - point2.S*point2.S*point3.S + point2.S*point3.F*point3.F + point2.S*point3.S*point3.S)/(2*(point1.F*point2.S - point1.S*point2.F - point1.F*point3.S + point1.S*point3.F + point2.F*point3.S - point2.S*point3.F));
          a2 =  (- point1.F*point1.F*point2.F + point1.F*point1.F*point3.F + point1.F*point2.F*point2.F + point1.F*point2.S*point2.S - point1.F*point3.F*point3.F - point1.F*point3.S*point3.S - point1.S*point1.S*point2.F + point1.S*point1.S*point3.F - point2.F*point2.F*point3.F + point2.F*point3.F*point3.F + point2.F*point3.S*point3.S - point2.S*point2.S*point3.F)/(2*(point1.F*point2.S - point1.S*point2.F - point1.F*point3.S + point1.S*point3.F + point2.F*point3.S - point2.S*point3.F));
          point4 = mp(a1,a2);
          
          if(incircle(a1,a2,v[l].F,v[l].S,dist(point4,point2))==false)
          {
          d=9;
          break;
          }



        }
        if(d!=9)
      {
        if(dist(point4,point2) < mx)
        {
          mx=dist(point4,point2);
          point5 = mp(a1,a2);
        }

      }
      d=0;
      }
      
    }
   }
   cout.precision(10);
   cout<<"Brute Force : \n";
   cout<<point5.F<<" "<<point5.S<<x;


}
