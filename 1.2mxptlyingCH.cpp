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
#define xx "\n"


double dist(pair<double,double> p1,pair<double,double> p2)
{

  double dist;
  dist = 1.0*sqrtl((p1.F-p2.F)*(p1.F-p2.F) + (p1.S-p2.S)*(p1.S-p2.S));
  return dist;
}
struct Point
{
    double x, y;
};
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    double val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
// Prints convex hull of a set of n points.
vector<Point> convexHull(Point points[], int n)
{
    // There must be at least 3 points
 
    // Initialize Result
    vector<Point> hull;
 
    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].x < points[l].x)
            l = i;
 
    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do
    {
        // Add current point to result
        hull.push_back(points[p]);
 
        // Search for a point 'q' such that orientation(p, q,
        // x) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p+1)%n;
        for (int i = 0; i < n; i++)
        {
           // If i is more counterclockwise than current q, then
           // update q
           if (orientation(points[p], points[i], points[q]) == 2)
               q = i;
        }
 
        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;
 
    } while (p != l);  // While we don't come to first point
 
    // Print Result
    return hull;
}

int main()
{

freopen("output.txt", "w", stdout);
//  int num_points = 100000; 
//  vector<pair<double,double>> v1;
//     for(int i=0;i<num_points;i++)
//     {
//         double random_x = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
//         double random_y = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
//         v1.pb(mp(random_x, random_y));
//     }
int rr;
for(rr=0;rr<10000;rr++)
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx,m1,m2,x1net,x2net,y1net,y2net,xfinal,yfinal,m,ndist;
vector<pair<double,double>> v3,v,v4;
set<pair<double,double>> se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> v2;
random_device rd;
srand(rd());

    
// cout<<rr+1<<" ";




v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));



Point points[] = {{ v[0].F, v[0].S },
                         { v[1].F, v[1].S  },
                         { v[2].F, v[2].S  },
                          { v[3].F, v[3].S  },
                          { v[4].F, v[4].S },
                         { v[5].F, v[5].S  },
                         { v[6].F, v[6].S  },
                          { v[7].F, v[7].S  }
                      
                      };
    int n = sizeof(points)/sizeof(points[0]);
    vector<Point> vch = convexHull(points, n);
     pair<double,double> ppp;
    for(i=0;i<vch.size();i++)
    {
        ppp = mp(vch[i].x,vch[i].y);
        v4.pb(ppp);

    }
    set<pair<double,double>> se1;

    for(i=0;i<v4.size();i++)
    {
        se1.insert(v4[i]);
    }

    // for(auto it :se1)
    // cout<<it.F<<" "<<it.S<<xx;

    // cout<<xx<<xx;

         
// cout<<"Points are : \n";
// for(i=0;i<v.size();i++)
// {
//     cout<<"( "<<v[i].F<<" , "<<v[i].S<<" ) "<<x;
// }



mx=0;
mxx=INT_MAX;
b=0;
double dd=0;
// for(i=0;i<v1.size();i++)
// {
    
//     for(r=0;r<=10;r=r+0.01)
//     {
//         mx=0;

//         for(j=0;j<v.size();j++)
//         {
            
//             a=abs(dist(v1[i],v[j]) - r);
//             mx=max(mx,a);
//         }
//         if(mx<mxx)
//         {
//             mxx=mx;
//             v3.pb(mp(r,mxx));
//             p1=v1[i];
//         }
//     }

// }
// cout<<"Minimum distance from each point Brute Force : ";
// cout<<mxx<<x;
// cout<<"Radius : "<<v3[v3.size()-1].F<<"   centre : ("<<p1.F<<" , "<<p1.S<<")"<<x;
// v3[v3.size()-1].F = mxx;
double finn, radd ;
pair<double,double> ff,ans;
finn = INT_MAX;
p1={0,0};
p2={0,0};
p3={0,0};
p4={0,0};
for(i=0;i<v.size();i++)
{
    for(j=0;j<v.size();j++)
    {
        for(k=0;k<v.size();k++)
        {
            for(l=0;l<v.size();l++)
            {
                if(v[i]!=v[j] &&  v[k] != v[l]  && v[i] != v[k] && v[i] != v[l] && v[j] != v[k] && v[j] != v[l])
                {
                    m1 = (v[j].F - v[i].F)/(v[i].S - v[j].S); 
                    m2 = (v[l].F - v[k].F)/(v[k].S - v[l].S); 
                    x1net = (v[i].F + v[j].F)/2;
                    x2net = (v[k].F + v[l].F)/2;
                    y1net = (v[i].S + v[j].S)/2;
                    y2net = (v[k].S + v[l].S)/2;
                    xfinal = ((y2net - y1net) + (m1*x1net - m2*x2net))/(m1 - m2);
                    yfinal = m1*xfinal - m1*x1net + y1net;
                    ff = mp(xfinal,yfinal);
                    
                    mx=0;
                    mxx=INT_MAX;
                    for(m=0;m<v.size();m++)
                    {
                        ndist = dist(ff,v[m]);
                        if(ndist>mx)
                        mx= ndist;
                        if(ndist<mxx)
                        mxx=ndist;



                    }
                    mxxx = mx - (mx + mxx)/2;
                    // cout<<ff.F<<" "<<ff.S<<"  RAD:   "<<mxxx<<x;
                    if(mxxx < finn)
                    {finn=mxxx;
                    ans=ff;
                    radd = (mx + mxx)/2;
                    p1=v[i];
                    p2=v[j];
                    p3=v[k];
                    p4=v[l];

                    // cout<<ans.F<<" "<<ans.S<<x;
                     
                    
                    
                    }

                }
                else
                continue;

            }
        }
    }
}
// cout<<"Minimum distance from each point our algo : ";
// cout<<finn<<x;
// // cout<<x;
// cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<x;
// // cout<<ans.F<<" "<<ans.S<<x;

// cout<<"ERROR : "<<v3[v3.size()-1].F - finn <<x<<x<<x;

// cout<<p1.F<<" "<<p1.S<<x;
// cout<<p2.F<<" "<<p2.S<<x;
// cout<<p3.F<<" "<<p3.S<<x;
// cout<<p4.F<<" "<<p4.S<<x;
if(se1.count(p1) || se1.count(p2) ||se1.count(p3) ||se1.count(p4) )
{
    int aa=0;
    pair<double,double> ppp1=ans,ppp2=ans,ppp3=ans,ppp4=ans;
    if(se1.count(p1))
    {
        aa++;
        ppp1 = p1;

    }
    if(se1.count(p2))
    {
        aa++;
        ppp2 = p2;

    }
    if(se1.count(p3))
    {
        aa++;
        ppp3 = p3;

    }
    if(se1.count(p4))
    {
        aa++;
        ppp4 = p4;

    }
    if(aa==2)
    {
        int ccc = 0;
        
        
        double bb1,bb2,bb3,bb4,bb5,bb6,bb7,bb8;
        bb1 = dist(ans,ppp1);
        bb2 = dist(ans,ppp2);
        bb3 = dist(ans,ppp3);
        bb4 = dist(ans,ppp4);
        bb5 = dist(ans,p1);
        bb6 = dist(ans,p3);
        bb7 = dist(ans,p2);
        bb8 = max({bb6,bb7,bb5});



        // if(bb1 == bb2 && bb6 == bb1)
        // cout<<bb1<<" "<<bb2<<" "<<bb3<<" "<<bb4<<" "<<bb8<<xx;
        const double epsilon = 0.00001; // Define a small value for precision

    if (fabs(bb1 - bb8) < epsilon && fabs(bb2 - bb8) < epsilon)
        cout << "TRUE \n";
    else if (fabs(bb1 - bb8) < epsilon && fabs(bb3 - bb8) < epsilon)
        cout << "TRUE \n";
    else if (fabs(bb1 - bb8) < epsilon && fabs(bb4 - bb8) < epsilon)
        cout << "TRUE \n";
    else if (fabs(bb2 - bb8) < epsilon && fabs(bb3 - bb8) < epsilon)
        cout << "TRUE \n";
    else if (fabs(bb2 - bb8) < epsilon && fabs(bb4 - bb8) < epsilon)
        cout << "TRUE \n";
    else if (fabs(bb3 - bb8) < epsilon && fabs(bb4 - bb8) < epsilon)
        cout << "TRUE \n";
    else {
        cout << "NOOOOOT TRUEEEEE \n";
        cout << bb1 << " " << bb2 << " " << bb3 << " " << bb4 << " " << bb8 << "\n";



            // cout<<"Points are : \n";
            // for(i=0;i<v.size();i++)
            // {
            //     cout<<"( "<<v[i].F<<" , "<<v[i].S<<" ) "<<xx;
            // }

            // cout<<"Minimum distance from each point our algo : ";
            // cout<<finn<<xx;
            // // cout<<x;
            // cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<xx;
            // // cout<<ans.F<<" "<<ans.S<<x;

            // // cout<<"ERROR : "<<v3[v3.size()-1].F - finn <<xx<<xx<<xx;

            // cout<<p1.F<<" "<<p1.S<<xx;
            // cout<<p2.F<<" "<<p2.S<<xx;
            // cout<<p3.F<<" "<<p3.S<<xx;
            // cout<<p4.F<<" "<<p4.S<<xx;
            // cout<<xx<<xx<<xx;
        }

    //    cout<<"EQUALLLL and is outer 2 circle \n";
        // else
        // {

            // cout<<"Points are : \n";
            // for(i=0;i<v.size();i++)
            // {
            //     cout<<"( "<<v[i].F<<" , "<<v[i].S<<" ) "<<xx;
            // }

            // cout<<"Minimum distance from each point our algo : ";
            // cout<<finn<<xx;
            // // cout<<x;
            // cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<xx;
            // // cout<<ans.F<<" "<<ans.S<<x;

            // // cout<<"ERROR : "<<v3[v3.size()-1].F - finn <<xx<<xx<<xx;

            // cout<<p1.F<<" "<<p1.S<<xx;
            // cout<<p2.F<<" "<<p2.S<<xx;
            // cout<<p3.F<<" "<<p3.S<<xx;
            // cout<<p4.F<<" "<<p4.S<<xx;
            // cout<<xx<<xx<<xx;

        // }

    }
    else
    cout<<"NOT 2 \n";
   
}


}
}
