/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */

// Hull code is from gfg with minor changes.


#include <bits/stdc++.h>
using namespace std;
#define F first 
#define S second
#define x "\n"
#define pb push_back

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
    vector<Point>  v,v1;
    Point p;
    vector<pair<vector<Point>,Point>> vv;
    int i,j,e;
    double m,c,d,mx=0,mxx=INT_MAX,p1,p2;
    vector<Point> points = {{0, 3}, {-6, 4}, {1, -4}, {-2, -2},
                            {4, 2}, {-5, -5}, {-8, -2}};
    v = convexHull(points);
    for(i=0;i<v.size();i++)
    cout<<v[i].F<<" "<<v[i].S<<x;
    for(i=0;i<v.size()-1;i++)
    {
        m = (v[i+1].S-v[i].S)/(v[i+1].F-v[i].F);
        c = m*v[i].F-v[i].S;
        for(j=0;j<v.size();j++)
        {
            d = abs(v[j].S - m*v[j].F + c)/(sqrtl(1+m*m));
            if(d>mx)
            {p=v[j];
            mx=d;}
            

        }
        v1.pb(v[i]);
        v1.pb(v[i+1]);
        
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

    for(i=0;i<vv.size();i++)
    {
        cout<<vv[i].F[0].F<<" "<<vv[i].F[0].S<<" "<<vv[i].F[1].F<<" "<<vv[i].F[1].S<<"    "<<vv[i].S.F<<" "<<vv[i].S.S<<x;
    
    }
    cout<<x<<x<<x;
    cout<<vv[e].F[0].F<<" "<<vv[e].F[0].S<<" "<<vv[e].F[1].F<<" "<<vv[e].F[1].S<<"    "<<vv[e].S.F<<" "<<vv[e].S.S<<x;
    double x1,x3,x2,x4,x5,y1,y2,y3,y4,y5;
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
    cout<<"y = "<<m<<"x + "<<c<<x;
    

}
