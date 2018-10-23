

class goal{
public:

    CvPoint r_pos(){return pos;}

    int r_id(){return id;}

    int r_cluster(){return cluster;}

    void upd_cluster(int c){cluster=c;}


    int r_assign(){return assign;}
    void upd_assign(){assign=1;}
    void clr_assign(){assign=0;}


    void updt_pos(CvPoint P){pos=P;};

    void reach(){
        reach_flag=1;
        //cout<<"        Goal "<<id<< " reached"<<endl;
        }

    int r_reach(){return reach_flag;}


    goal(int i, CvPoint P){id=i; pos=P;reach_flag=0;cluster=0;assign=0;};

private:
    int id;
    CvPoint pos;
    int reach_flag;
    int cluster;
    int assign;

};
