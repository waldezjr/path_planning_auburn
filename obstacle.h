

class obstacle{
public:

    CvPoint r_pos(){return pos;}
    CvPoint r_vel(){return vel;}

    int r_id(){return id;}



    void updt_pos(CvPoint P){pos=P;};
    void updt_vel(CvPoint V){vel=V;};

    obstacle(int i, CvPoint P,CvPoint V){id=i; pos=P;vel=V;};

private:
    int id;
    CvPoint pos;
    CvPoint vel;

};
