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
#define pie 3.1415926535897932384626433832795
#define ll  long long

int main()
{
double x1,y1,x2,y2,x3,y3,i,j,k,l,d1,d2,d3,d=0;
vector<pair<double,double>> v1,v2,v3;
x1=-0.5;  // centre of different cirles
y1=-0.5;
x2=0.9;
y2=-0.1;
x3=-1.0;
y3=-1.1;
d1 = sqrtl((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
d2 = sqrtl((x2-x3)*(x3-x2) + (y3-y2)*(y3-y2));
d3 = sqrtl((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3));
for(i=0;i<=3;i=i+0.1)  // increasing radius (not more than 3m)
{d=0;

    for(j=-3;j<=3;j=j+0.1)
    {
        for(k=-3;k<=3;k=k+0.1)
      {
        if((j-x1)*(j-x1) + (k-y1)*(k-y1) - i*i < 0)  // checking if the point is inside the circle or not                                                  // 
        {
            v1.pb(mp(j,k));             
        }
        
      }

    }
    for(j=-3;j<=3;j=j+0.1)
    {
        for(k=-3;k<=3;k=k+0.1)
      {
        if((j-x2)*(j-x2) + (k-y2)*(k-y2) - i*i < 0)
        {
            v2.pb(mp(j,k));
        }
        
      }

    }

    for(j=-3;j<=3;j=j+0.1)
    {
        for(k=-3;k<=3;k=k+0.1)
      {
        if((j-x3)*(j-x3) + (k-y3)*(k-y3) - i*i < 0)
        {
            v3.pb(mp(j,k));
        }
        
      }

    }



    for(k=0;k<v1.size();k++)
{
    for(j=0;j<v2.size();j++)
    {
      for(l=0;l<v3.size();l++)
    {

       
            if(v1[k].F==v2[j].F && v1[k].F==v3[l].F  && v1[k].S==v2[j].S && v1[k].S==v3[l].S )
            {
              cout<<"Point of interection is : ";
              cout<<v1[k].F<<" , "<<v1[k].S<<x;
            d=9;
            break;}
    }
    if(d==9)
    break;
        
        
    }
    if(d==9)
    break;
}
if(d==9)
{
  cout<<i<<x;
  break;
}
v1.clear();
v2.clear();

}
d=0;





}

