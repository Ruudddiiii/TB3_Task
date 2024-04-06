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


int main()
{

freopen("output.txt", "w", stdout);

int rr;
for(rr=0;rr<1;rr++)
{
double x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,i,j,k,l,min,radi,radi1,d=0,a,b=0,r=0,mx,mxx,mxxx,m1,m2,x1net,x2net,y1net,y2net,xfinal,yfinal,m,ndist;
vector<pair<double,double>> v3,v,v4;
set<pair<double,double>> se2,se3,se4;
pair<double,double> p1,p2,p3,p4,p5,point1,point2,point3,point4,point5;
pair<pair<double,double>,pair<double,double>> point11;
vector<double> v2;
random_device rd;
srand(rd());


v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));
v.pb(mp(static_cast<double>(rand()) / RAND_MAX * 10 - 5,static_cast<double>(rand()) / RAND_MAX * 10 - 5));

for(i=0;i<v.size();i++)
cout<<v[i].F<<","<<v[i].S<<xx;
cout<<xx;

mx=0;
mxx=INT_MAX;
b=0;
double dd=0;

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
cout<<"Minimum distance from each point our algo : ";
cout<<finn<<xx;
// cout<<x;
cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<xx<<xx;

cout<<xx;


double c1;
double dd1,dd2,dd3,dd4,dd5;
p1={0,0};
p2={0,0};
p3={0,0};
p4={0,0};
vector<pair<double,double>> vp;
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
                    
                    vp.pb(ff);
                    // vp.pb(v[i]);
                    // vp.pb(v[j]);
                    // vp.pb(v[k]);
                    // vp.pb(v[l]);

                     
                    
                    
                    

                }
                else
                continue;

            }
        }
    }
}
ans = {0.0,0.0};
mx=0;
mxx=INT_MAX;
double ndist1,ndist2,ndist3,nmax,nmin;
finn = INT_MAX;
for(i=0;i<vp.size();i++)
{

    mx=0;
    mxx=INT_MAX;
    for(j=0;j<v.size();j++)
    {
        ndist1 = dist(vp[i],v[j]);


        if(ndist1>mx)
        mx = ndist1;
        if(ndist1<mxx)
        mxx = ndist1;
        


    }
    // cout<<mx<<xx;
    mxxx = mx - (mx + mxx)/2;
    if(mxxx < finn)
    {finn=mxxx;
    ans=vp[i];
    radd = (mx + mxx)/2;
    }
}
// cout<<dd5<<xx;
cout<<"Minimum distance from each point our algo : ";
cout<<finn<<xx;
// cout<<x;
cout<<"Radius : "<<radd<<"   centre : ("<<ans.F<<" , "<<ans.S<<")"<<xx;



}
}
