/*
   कर्मण्येवाधिकारस्ते मा फलेषु कदाचन ।
   मा कर्मफलहेतुर्भुर्मा ते संगोऽस्त्वकर्मणि ॥
                                   */

// Hull code is from gfg with minoe changes.


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
    m = (vv[e].F[1].S - vv[e].F[0].S)/(vv[e].F[1].F - vv[e].F[0].F);
    p1 = (vv[e].S.F + vv[e].F[0].F)/2;
    p2 = (vv[e].S.S + vv[e].F[0].S)/2;
    cout<<"y = "<<m<<"x + "<<p2-m*p1<<x;
    

}
