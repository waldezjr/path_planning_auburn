using namespace std;

class robot{
public:

    CvPoint r_pos(){return pos;}
    float r_theta(){return theta;}
    int r_id(){return id;}
    float r_phi1(){return phi1;}
    float r_phi2(){return phi2;}
    int r_assign(){return assign;}
    void upd_assign(){assign=1;}
    void clr_assign(){assign=0;}
    CvPoint r_V(){return V;}
    void updt_V(CvPoint vel){V=vel;}

    void updt_pos(CvPoint P, float t){pos=P; theta=t;};
    void updt_spd(float p1, float p2){

                int aux=p1-p2;
                if(aux>5) aux=5;

                if (p1>20){
                    if(p1!=p2) phi1=20+aux;
                    else phi1=20;
                }
                else phi1=p1;

                if(p2>20) phi2=20;
                else phi2=p2;
                //phi1=p1;
                //phi2=p2;
        };

    robot(int i, CvPoint P,float t,float p1, float p2){
            id=i;
            pos=P;
            theta=t;
                if (p1>12) phi1=12;
                else phi1=p1;
                if(p2>12) phi2=12;
                else phi2=p2;
            assign=0;
    };

private:
    int id;
    CvPoint pos;
    float	theta;
    float phi1;//right
    float phi2;//left
    int assign;
    CvPoint V;

};
