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


int main()
{

freopen("output.txt", "w", stdout);
 int num_points = 100000; 
 vector<pair<double,double>> v1;
    for(int i=0;i<num_points;i++)
    {
        double random_x = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
        double random_y = static_cast<double>(rand()) / RAND_MAX * 10 - 5; 
        v1.pb(mp(random_x, random_y));
    }
int rr;
for(rr=0;rr<50;rr++)
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx,m1,m2,x1net,x2net,y1net,y2net,xfinal,yfinal,m,ndist;
vector<pair<double,double>> v3,v;
set<pair<double,double>> se1,se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> v2;
random_device rd;
srand(rd());

    
cout<<rr<<". \n";




v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));

cout<<"Points are : \n";
for(i=0;i<v.size();i++)
{
    cout<<"( "<<v[i].F<<" , "<<v[i].S<<" ) "<<x;
}





mx=0;
mxx=INT_MAX;
b=0;
double dd=0;
for(i=0;i<v1.size();i++)
{
    
    for(r=0;r<=10;r=r+0.01)
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
cout<<"Minimum distance from each point Brute Force : ";
cout<<mxx<<x;
cout<<"Radius : "<<v3[v3.size()-1].F<<"   centre : ("<<p1.F<<" , "<<p1.S<<")"<<x;
v3[v3.size()-1].F = mxx;
double finn, radd ;
pair<double,double> ff,ans;
finn = INT_MAX;
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
                    // cout<<ans.F<<" "<<ans.S<<x;
                     
                    
                    
                    }

                }
                else
                continue;

            }
        }
    }
}
cout<<"Minimum distance from each point our algo : ";
cout<<finn<<x;
// cout<<x;
cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<x;
// cout<<ans.F<<" "<<ans.S<<x;

cout<<"ERROR : "<<v3[v3.size()-1].F - finn <<x<<x<<x;

}
}