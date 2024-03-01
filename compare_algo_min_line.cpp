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
#define ll longlong


double dist(pair<double,double> p1,pair<double,double> p2)
{

  double dist;
  dist = 1.0*sqrtl((p1.F-p2.F)*(p1.F-p2.F) + (p1.S-p2.S)*(p1.S-p2.S));
  return dist;
}
typedef pair<double, double> Point;

int orientation(Point p, Point q, Point r)
{
    double val = (q.second - p.second) * (r.first - q.first) -
                 (q.first - p.first) * (r.second - q.second);
    if (val == 0)
        return 0;
    return (val > 0) ? 1 : 2;
}
vector<Point> convexHull(vector<Point> points)
{
    int n = points.size();
    vector<Point> hull;
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].first < points[l].first)
            l = i;
    int p = l, q;
    do
    {
        hull.push_back(points[p]);
        q = (p + 1) % n;
        for (int i = 0; i < n; i++)
        {
            if (orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }
        p = q;
    } while (p != l);
    return hull;
}

int main()
{

//freopen("cheeseburger_corollary_1_input.txt", "r", stdin);

freopen("output.txt", "w", stdout);
    int r;

for(r=0;r<20;r++)
{
     cout<<r<<". \n";
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx;
int i;

vector<pair<double,double>> v1,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> vm,vc;

srand(time(0));

    vector<Point> points = {{static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 },
                            {static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 },   
                            {static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 },
                            {static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 },
                            {static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 },
                            {static_cast<double>(rand()) / RAND_MAX * 10 - 5 , static_cast<double>(rand()) / RAND_MAX * 10 - 5 }
                            };


    
    v = convexHull(points);
    // cout<<"Points: \n";
    // for(i=0;i<points.size();i++)
    // cout<<points[i].F<<" "<<points[i].S<<x;
    // cout<<x;
    // cout<<"Convex Hull: \n";
    // for(i=0;i<v.size();i++)
    // cout<<v[i].F<<" "<<v[i].S<<x;




int num_points = 10000;

for(int i=0;i<num_points;i++)
{
    double random_x = static_cast<double>(rand()) / RAND_MAX * 100 - 50; 
    double random_y = static_cast<double>(rand()) / RAND_MAX * 100 - 50; 
    vm.pb(random_x);
    vc.pb(random_y);
}


// for(i=0;i<vc.size();i++)
// cout<<vc[i]<<x;

mxxx = INT_MAX;

for(i=0;i<vc.size();i++)
{
    
    for(j=0;j<vm.size();j++)
    {
       mxx=0;
        for(k=0;k<v.size();k++)
        {
            mx = abs(v[k].S-vm[j]*v[k].F-vc[i])/sqrt(1+vm[j]*vm[j]);
            if(mx>mxx)
            mxx=mx;
        }
        if(mxx<mxxx)
        {mxxx=mxx;
        p1.F=vc[i];
        p1.S=vm[j];
        }
        
    }
}
cout<<x<<x;
cout<<"Brute force : \n";
cout<<"y = "<<p1.S<<"x + "<<p1.F<<x;

pair<double ,double> pp;
pp.F=p1.S;
pp.S=p1.F;

v1.clear();
    // vector<Point>  v,v1;
    Point p;
    vector<pair<vector<Point>,Point>> vv;
    int e;
    double m,c;
    mx=0;
    mxx=INT_MAX;
long long szz = v.size();
for(i=0;i<v.size();i++)
    {
        
        m = (v[(i+1)%szz].S-v[i].S)/(v[(i+1)%szz].F-v[i].F);
        c = m*v[i].F-v[i].S;
        for(j=0;j<v.size();j++)
        {
            d = abs(v[j].S - m*v[j].F + c)/(sqrtl(1+m*m));
            if(d>mx)
            {p=v[j];
            mx=d;}
            

        }
        v1.pb(v[i]);
        v1.pb(v[(i+1)%szz]);
        
        // cout<<p.F<<x;
        vv.pb(make_pair(v1,p));
        if(mx<mxx)
        {
            e=i;
            mxx=mx;
        }
        v1.clear();
        mx=0;


    }
    // // cout<<vv[i].F;

    // for(i=0;i<vv.size();i++)
    // {
    //     cout<<vv[i].F[0].F<<" "<<vv[i].F[0].S<<" "<<vv[i].F[1].F<<" "<<vv[i].F[1].S<<"    "<<vv[i].S.F<<" "<<vv[i].S.S<<x;
    
    // }
    // cout<<x<<x<<x;
    // cout<<vv[e].F[0].F<<" "<<vv[e].F[0].S<<" "<<vv[e].F[1].F<<" "<<vv[e].F[1].S<<"    "<<vv[e].S.F<<" "<<vv[e].S.S<<x;
 
    x1 = vv[e].F[0].F;
    y1 = vv[e].F[0].S;
    x2 = vv[e].F[1].F;
    y2 = vv[e].F[1].S;
    x3 = vv[e].S.F;
    y3 = vv[e].S.S;
    m = (y2 - y1) / (x2 - x1);
    x4 = ( y3 - y1 + x1*m + x3/m)/(m + 1/m);
    y4 = - x4/m + x3/m + y3;
    x5 = (x4 + x3)/2;
    y5 = (y4 + y3)/2;
    c = (y5 - m * x5);
    cout<<x<<x;
    cout<<"Our Algorithm : \n";
    cout<<"y = "<<m<<"x + "<<c<<x;
    cout<<x<<x;

    cout<<"Error % : \n";
    double ee1,ee2;
    // cout<<m<<x;
    // cout<<pp.F<<x;
    ee1 = (abs((abs(m)-abs(pp.F)))/abs(m))*100;
    ee2 = (abs((abs(c)-abs(pp.S)))/abs(c))*100;
    cout<<ee1<<" "<<ee2<<x<<x;






}





}