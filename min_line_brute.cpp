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
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx;
vector<pair<double,double>> v1,v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> vm,vc;

vector<Point> points = {{4, 8}, {-2, 6}, {8, -2}, {4, 4},
                            {10, 6}, {3, 3}, {2, 2}};
    v = convexHull(points);
    for(i=0;i<v.size();i++)
    cout<<v[i].F<<" "<<v[i].S<<x;



srand(time(0));
int num_points = 10000;

for(int i=0;i<num_points;i++)
{
    double random_x = static_cast<double>(rand()) / RAND_MAX * 20 - 10; 
    double random_y = static_cast<double>(rand()) / RAND_MAX * 20 - 10; 
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
cout<<"y = "<<p1.S<<"x + "<<p1.F<<x;











}