
using namespace std;

class dynamics{
public:
    void move(robot &r1, float img_width, float img_height){

        CvPoint prev,next;
        int phi1, phi2;
        float del_theta,theta_p, theta_n, del_s,b;


        b=20;

        prev=r1.r_pos();
        theta_p=r1.r_theta();
        phi1=r1.r_phi1();
        phi2=r1.r_phi2();

        del_s=(phi1+phi2)/2;
        del_theta=(phi1-phi2)/b;

        float b1=prev.x+round(del_s*cos(theta_p+del_theta/2));
        next.x=b1;

        float b2=prev.y-round(del_s*sin(theta_p+del_theta/2));//DIFFERENT AXIS
        next.y=b2;

        theta_n=theta_p+del_theta;

        if(next.x+15>img_width) next.x=15;
        else if(next.x-15<0) next.x=img_width-15;

        if(next.y+15>img_height) next.y=15;
        else if(next.y-15<0) next.y=img_height-15;

        //cout<<next.x<<"  "<<next.y<<" "<<theta_p<<endl;

        r1.updt_pos(next,theta_n);
        r1.updt_V(cvPoint(next.x-prev.x,next.y-prev.y));

        //cout<<r1.r_V().x<<endl;



    }

    void move(obstacle &o, int width, int height){
        CvPoint pos=o.r_pos();
        CvPoint vel=o.r_vel();

        CvPoint next=cvPoint(pos.x+vel.x,pos.y+vel.y);

        if(next.x+1>width) next.x=1;
        else if(next.x-1<0) next.x=width-1;

        if(next.y+1>height) next.y=1;
        else if(next.y-1<0) next.y=height-1;

        o.updt_pos(next);
    }

    void apf(vector <obstacle> &obs,goal &goals, robot &r, vector <robot> &robots){


        vector <CvPoint> F_rep;
        vector <CvPoint> F_att;
        CvPoint F_rep_total, F_att_total, F_total;

        float k_f,k_turn;
        k_f=0.03;
        k_turn=20;

        F_rep_total=cvPoint(0,0);
        F_att_total=cvPoint(0,0);

        for(int i=0;i<obs.size();i++){
            F_rep.push_back(fn_F_rep(r.r_pos(),obs[i].r_pos(),goals));

        }
        for(int i=0;i<robots.size();i++){
            if(robots[i].r_id()!=r.r_id())F_rep.push_back(fn_F_rep(r.r_pos(),obs[i].r_pos(),goals));

        }

        for(int i=0;i<F_rep.size();i++){
        F_rep_total.x+=F_rep[i].x;
        F_rep_total.y+=F_rep[i].y;
        }



            F_att.push_back(fn_F_att(r.r_pos(),goals.r_pos()));


        for(int i=0;i<F_att.size();i++){
        F_att_total.x+=F_att[i].x;
        F_att_total.y+=F_att[i].y;
        }

        //float k_app=0.03;
        F_total.x=F_att_total.x+F_rep_total.x;
        F_total.y=F_att_total.y+F_rep_total.y;

        float theta_turn=atan2(-F_total.y,F_total.x)-r.r_theta();

        float mag_f=sqrt(F_total.x*F_total.x+F_total.y*F_total.y);

        float phi1, phi2;

        phi1=k_f*mag_f+k_turn*theta_turn;
        phi2=k_f*mag_f;
        //cout<<"    "<<F_rep_total.x<<"    "<<F_rep_total.y<<"positions:";
        r.updt_spd(phi1,phi2);


        CvPoint r1=r.r_pos();
        CvPoint g=goals.r_pos();
        float rho_goal=sqrt((r1.x-g.x)*(r1.x-g.x)+(r1.y-g.y)*(r1.y-g.y));
        //cout<<"xxx"<<rho_goal;
            if(rho_goal<35 &&(float)(phi1+phi2)/2<1) goals.reach();
            //cout<<(phi1+phi2)/2<<endl;






    }

void Eapf(vector <obstacle> &obs,goal &goals, robot &r, vector <robot> &robots){


        vector <CvPoint> F_rep;
        vector <CvPoint> F_att;
        CvPoint F_rep_total, F_att_total, F_total;

        float theta, alpha,beta,gamma;

        theta=atan2(r.r_V().y,r.r_V().x);

        beta=0;//vg=0


        float k_f,k_turn;
        k_f=0.03;
        k_turn=20;

        F_rep_total=cvPoint(0,0);
        F_att_total=cvPoint(0,0);

        for(int i=0;i<obs.size();i++){
            F_rep.push_back(fn_F_rep(r.r_pos(),obs[i].r_pos(),goals));
           // gamma=atan2()

        }
        for(int i=0;i<robots.size();i++){
            if(robots[i].r_id()!=r.r_id())F_rep.push_back(fn_F_rep(r.r_pos(),obs[i].r_pos(),goals));

        }

        for(int i=0;i<F_rep.size();i++){
        F_rep_total.x+=F_rep[i].x;
        F_rep_total.y+=F_rep[i].y;
        }



            F_att.push_back(fn_F_att(r.r_pos(),goals.r_pos()));


        for(int i=0;i<F_att.size();i++){
        F_att_total.x+=F_att[i].x;
        F_att_total.y+=F_att[i].y;
        }

        //float k_app=0.03;
        F_total.x=F_att_total.x+F_rep_total.x;
        F_total.y=F_att_total.y+F_rep_total.y;

        float theta_turn=atan2(-F_total.y,F_total.x)-r.r_theta();

        float mag_f=sqrt(F_total.x*F_total.x+F_total.y*F_total.y);

        float phi1, phi2;

        phi1=k_f*mag_f+k_turn*theta_turn;
        phi2=k_f*mag_f;
        //cout<<"    "<<F_rep_total.x<<"    "<<F_rep_total.y<<"positions:";
        r.updt_spd(phi1,phi2);


        CvPoint r1=r.r_pos();
        CvPoint g=goals.r_pos();
        float rho_goal=sqrt((r1.x-g.x)*(r1.x-g.x)+(r1.y-g.y)*(r1.y-g.y));
        //cout<<"xxx"<<rho_goal;
            if(rho_goal<35 &&(float)(phi1+phi2)/2<1) goals.reach();

    }

    vector <CvPoint> assign_goals(vector <robot> &robots, vector <goal> &goals){

        vector <CvPoint> table;

        //clustering(goals);
        int cluster;


                for(int i=0;i<goals.size();i++){
                    goals[i].clr_assign();
                }
                for(int i=0;i<robots.size();i++){
                    robots[i].clr_assign();
                    robots[i].updt_spd(0,0);
                }

                for(int j=0;j<goals.size();j++){
cluster=-1;
                    for(int i=0;i<goals.size();i++){

                        if(goals[i].r_reach()==0 && goals[i].r_cluster()!=cluster && goals[i].r_assign()==0){
                                table.push_back(cvPoint(min_dist(robots,goals[i]),goals[i].r_id()));
                                goals[i].upd_assign();
                                cluster=goals[i].r_cluster();

                        }
                    }
               }

            /*for(int j=0;j<3;j++){

                for (int i=0;i<goals.size();i++){

                        if(goals[i].r_reach()==0 && goals[i].r_cluster()==j && goals[i].r_assign()==0){
                                table.push_back(cvPoint(min_dist(robots,goals[i]),goals[i].r_id()));
                                goals[i].upd_assign();

                                    if(table[i].x!=-1) break;
                        }



                }
            }*/

           // cout<<"   "<<table.size();//<<table[0].x<<";"<<table[0].y;

        return table;

    }
    void clustering(vector <goal> &goals){


        CvMat A;
        int z=0;
        int *vals;
        vals=(int*)malloc(2*goals.size()*sizeof(int*));

            for(int j=0;j<goals.size();j++){


                    vals[z]=goals[j].r_pos().x;
                    vals[z+1]=goals[j].r_pos().y;
                    z=z+2;

            }
            cvInitMatHeader(&A,goals.size(),2,CV_32FC1,vals,CV_AUTOSTEP);
           // printf("\n %f",CV_MAT_ELEM(A,float,0,0));//A is a nx3 matrix with all the colors (RGB) in the image
            //cout<<CV_MAT_ELEM(A,int,3,0);

            CvMat *clusters = cvCreateMat( goals.size(), 1, CV_32S );

            cvKMeans2( &A, 3, clusters,cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0 ),10, 0, 0, 0, 0 );
//            cout<<clusters->data.i[3];

            for(int i=0;i<goals.size();i++){
                goals[i].upd_cluster(clusters->data.i[i]);
            }

    }

private:
    CvPoint fn_F_rep(CvPoint r1, CvPoint o, goal &go){

        float rho_0=150;
        float rho;
        float k_rep=200000000;

        float buf,ap,bp;
        CvPoint aux, g;
        g=go.r_pos();

        ap=(float)(r1.y-g.y)/(r1.x-g.x);
        bp=-ap*r1.x+r1.y;

        rho=sqrt((r1.x-o.x)*(r1.x-o.x)+(r1.y-o.y)*(r1.y-o.y));

        if(rho>=rho_0)  r1=cvPoint(0,0);
        else{
        buf=k_rep*(1/rho - 1/rho_0)*1/(rho*rho*rho);
        //cout<<buf;
        r1.x=buf*(r1.x-o.x);
        r1.y=buf*(r1.y-o.y);

        aux.x=r1.x;
        aux.y=r1.y;

            if(o.y>ap*o.x+bp){
                r1.x=-aux.y;
                r1.y=aux.x;
            }
            else{
                r1.x=aux.y;
                r1.y=-aux.x;
            }
        }


        return r1;
    }
CvPoint fn_F_rep_robot(CvPoint r1, CvPoint o){
float rho_0=150;
        float rho;
        float k_rep=300000000;

        float buf,ap,bp;
        CvPoint aux, g;

        rho=sqrt((r1.x-o.x)*(r1.x-o.x)+(r1.y-o.y)*(r1.y-o.y));

        if(rho>=rho_0)  r1=cvPoint(0,0);
        else{
        buf=k_rep*(1/rho - 1/rho_0)*1/(rho*rho*rho);
        //cout<<buf;
        r1.x=buf*(r1.x-o.x);
        r1.y=buf*(r1.y-o.y);
        }



        return r1;
}
    CvPoint fn_F_att(CvPoint r1, CvPoint g){


        //float rho;
        float k_att=1;

        //float buf;

        //rho=sqrt((r1.x-o.x)*(r1.x-o.x)+(r1.y-o.y)*(r1.y-o.y));

        //if(rho>=rho_0)  r1=cvPoint(0,0);
        //else{
        //buf=k_rep*(1/rho - 1/rho_0)*1/(rho*rho*rho);
        r1.x=-k_att*(r1.x-g.x);
        r1.y=-k_att*(r1.y-g.y);

        //}

        return r1;
    }

    int min_dist(vector <robot> &robots, goal g){
        int id=0;
        float mdist=500000;
        CvPoint r1, g1=g.r_pos();

        for(int i=0;i<robots.size();i++){

            r1=robots[i].r_pos();
            if(sqrt((r1.x-g1.x)*(r1.x-g1.x)+(r1.y-g1.y)*(r1.y-g1.y))<mdist && robots[i].r_assign()==0){
                mdist=sqrt((r1.x-g1.x)*(r1.x-g1.x)+(r1.y-g1.y)*(r1.y-g1.y));

                id=robots[i].r_id();
            }
        }
        robots[id-1].upd_assign();


        if (mdist==500000) return -1;

        return id;
    }




};

