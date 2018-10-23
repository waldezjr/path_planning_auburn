#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc_c.h>
#define pi 3.14159265359
#define PI 3.14159265359
#include "5530.h"

int main(void){

IplImage* display=cvCreateImage( cvSize(800,600), IPL_DEPTH_8U, 3 );
cvNamedWindow("Waldez Jr - Final Project - Mobile Robots Design", CV_WINDOW_AUTOSIZE);
cvZero(display);

drawing pen;//drawing object that will draw the other objects in the screen
dynamics motion;//dynamics object that will define to where the objects in the scene will move

int check_reach1=0,check_reach2=0;//flags that are used to check if a robot reached its goal


//creates lists
    vector <robot> robots;
    vector <obstacle> obs;
    vector <goal> goals;
    vector <CvPoint> assignments, assignments2;
//lists' inputs

    //goals.push_back(goal(1,cvPoint(580,300)));//SIMPLE DEMO

    goals.push_back(goal(1,cvPoint(510,50)));
    goals.push_back(goal(2,cvPoint(600,50)));
    goals.push_back(goal(3,cvPoint(750,100)));
    goals.push_back(goal(4,cvPoint(200,500)));
    goals.push_back(goal(5,cvPoint(750,550)));

    //obs.push_back(obstacle(1,cvPoint(400,300),cvPoint(0,0)));//SIMPLE DEMO

    obs.push_back(obstacle(1,cvPoint(400,150),cvPoint(0,0)));//moving obstacles parameter for presentation: 1,0
    obs.push_back(obstacle(2,cvPoint(400,300),cvPoint(0,0)));//-1,0
    obs.push_back(obstacle(3,cvPoint(400,450),cvPoint(0,0)));//2,0
    obs.push_back(obstacle(4,cvPoint(550,200),cvPoint(0,0)));//-3,0
    obs.push_back(obstacle(5,cvPoint(550,350),cvPoint(0,0)));//4,0

    //robots.push_back(robot(1,cvPoint(50,300),pi-pi,0,0));//SIMPLE DEMO

    robots.push_back(robot(1,cvPoint(50,100),pi-pi,0,0));
    robots.push_back(robot(2,cvPoint(50,430),0,0,0));
    robots.push_back(robot(3,cvPoint(50,250),pi-pi,0,0));

//MANUAL ASSIGNMENTS

    //assignments.push_back(cvPoint(1,3));
    //assignments.push_back(cvPoint(2,2));
    //assignments.push_back(cvPoint(3,4));

//CLUSTERING OF GOALS FOLLOWED BY AUTOMATIC ASSIGNMENT OF GOALS TO EACH ROBOT
    motion.clustering(goals);
    assignments=motion.assign_goals(robots,goals);



    for(/*ever*/;;){
                    cvZero(display);
                    check_reach1=0;
                            for(int i=0;i<obs.size();i++){
                            pen.draw_obs(obs[i],display);
                            motion.move(obs[i],display->width,display->height);
                            }

                            for(int i=0;i<goals.size();i++){
                            pen.draw_goal(goals[i],display);

                            check_reach1+=goals[i].r_reach();
                            }

                            for(int i=0;i<robots.size();i++){
                            pen.draw_robot(robots[i],display);
                            motion.move(robots[i],display->width,display->height);
                            }



                            //motion.apf(obs,goals[1],robots[0]);



                            for(int i=0;i<assignments.size();i++){
                                if(assignments[i].x!=-1){//if the robot hasn't been assigned a goal yet
                                    motion.apf(obs,goals[assignments[i].y-1],robots[assignments[i].x-1], robots);
                                }
                            }

                            if(check_reach2!=check_reach1) {
                                cout<<"INTERRUPT!!!!";
                                assignments.clear();
                                assignments=motion.assign_goals(robots,goals);
                                //cout<<assignments2[2].x<<";"<<assignments2[2].y<<endl;
                                //break;
                                }
                            check_reach2=check_reach1;




                    cvShowImage("Waldez Jr - Final Project - Mobile Robots Design",display);


                    if(cvWaitKey(120)=='q') break;//time the presentation of the simulation in the screen


    }









return 0;
}
