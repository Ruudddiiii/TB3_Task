/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */
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


int main()
{
double x1,y1,x2,y2,x3,y3,x4,y4,i,j,k,l,d,min;
vector<pair<double,double>> v1,v2,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,pmin;

x1=-2.5;  // centre of different cirles
y1=2.5;
x2=-0.9;
y2=-2.1;
x3=2.0;
y3=-2.1;
x4=2.9;
y4=0.3;
p1=mp(x1,y1);
p2=mp(x2,y2);
p3=mp(x3,y3);
p4=mp(x4,y4);
v.pb(p1);
v.pb(p2);
v.pb(p3);
v.pb(p4);
min=LLONG_MAX;
for(i=0;i<v.size();i++)
{
  if(dist(mp(0.0,0.0),v[i])<min)
  {min=dist(mp(0.0,0.0),v[i]);
  pmin=v[i];}
}

cout.precision(15);
cout<<"The closest point : \n";
cout<<pmin.F<<" "<<pmin.S<<x;
cout<<"Distance is : \n";
cout<<fixed<<min<<x;







































// for(i=0;i<=10;i=i+0.1)
// {d=0;

//     for(j=-3;j<=3;j=j+0.1)
//     {
//         for(k=-3;k<=3;k=k+0.1)
//       {
//         if((j-x1)*(j-x1) + (k-y1)*(k-y1) - i*i < 0)
//         {
//             se1.insert(mp(j,k));
//         }
        
//       }

//     }
//     for(j=-3;j<=3;j=j+0.1)
//     {
//         for(k=-3;k<=3;k=k+0.1)
//       {
//         if((j-x2)*(j-x2) + (k-y2)*(k-y2) - i*i < 0)
//         {
//             se2.insert(mp(j,k));
//         }
        
//       }

//     }
//     for(j=-3;j<=3;j=j+0.1)
//     {
//         for(k=-3;k<=3;k=k+0.1)
//       {
//         if((j-x3)*(j-x3) + (k-y3)*(k-y3) - i*i < 0)
//         {
//             se3.insert(mp(j,k));
//         }
        
//       }

//     }
//     for(j=-3;j<=3;j=j+0.1)
//     {
//         for(k=-3;k<=3;k=k+0.1)
//       {
//         if((j-x4)*(j-x4) + (k-y4)*(k-y4) - i*i < 0)
//         {
//             se4.insert(mp(j,k));
//         }
        
//       }

//     }
//     for(auto it : se1)
// {
//   if(se2.count(it)!=0 && se3.count(it)!=0 && se4.count(it)!=0)
//   {
//     cout<<"Found the point (x,y) : ";
//     cout<<it.F<<" , "<<it.S<<x;
//     d=9;
//     break;
//   }
    
// }
// if(d==9)
// {
   
//   cout<<"Radius is : ";
//   cout<<i*i<<x;
//   break;
// }
// se1.clear();
// se2.clear();
// se3.clear();
// se4.clear();

// }
// d=0;

}
