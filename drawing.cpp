#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc_c.h>
#define pi 3.14159265359
#define PI 3.14159265359
#include "robot.h"
#include "drawing.h"
#include "dynamics.h"
void draw_robot(robot r,IplImage* img){
                int L;
                L=20;


                CvPoint u_left,u_right,l_left,l_right,w1_l,w1_r,w2_r,w2_l,w3_l,w3_r,w4_l,w4_r,h;

                CvPoint pos=r.r_pos();

                u_left=cvPoint(-L,-L);
                u_right=cvPoint(+L,-L);
                l_left=cvPoint(-L,+L);
                l_right=cvPoint(+L,+L);

                w1_l=cvPoint(-L/2,-L/2);
                w1_r=cvPoint(L/2,-L/2);
                w3_l=cvPoint(-L/2,-L);
                w3_r=cvPoint(L/2,-L);
                w2_l=cvPoint(-L/2,L/2);
                w2_r=cvPoint(L/2,L/2);
                w4_l=cvPoint(-L/2,+L);
                w4_r=cvPoint(L/2,+L);
                h=cvPoint(L,0);


                float theta=r.r_theta();


                u_left=cvPoint(u_left.x*cos(theta)+u_left.y*sin(theta),u_left.x*-sin(theta)+u_left.y*cos(theta));
                u_right=cvPoint(u_right.x*cos(theta)+u_right.y*sin(theta),u_right.x*-sin(theta)+u_right.y*cos(theta));
                l_left=cvPoint(l_left.x*cos(theta)+l_left.y*sin(theta),l_left.x*-sin(theta)+l_left.y*cos(theta));
                l_right=cvPoint(l_right.x*cos(theta)+l_right.y*sin(theta),l_right.x*-sin(theta)+l_right.y*cos(theta));

                w1_l=cvPoint(w1_l.x*cos(theta)+w1_l.y*sin(theta),w1_l.x*-sin(theta)+w1_l.y*cos(theta));
                w1_r=cvPoint(w1_r.x*cos(theta)+w1_r.y*sin(theta),w1_r.x*-sin(theta)+w1_r.y*cos(theta));
                w2_l=cvPoint(w2_l.x*cos(theta)+w2_l.y*sin(theta),w2_l.x*-sin(theta)+w2_l.y*cos(theta));
                w2_r=cvPoint(w2_r.x*cos(theta)+w2_r.y*sin(theta),w2_r.x*-sin(theta)+w2_r.y*cos(theta));
                w3_l=cvPoint(w3_l.x*cos(theta)+w3_l.y*sin(theta),w3_l.x*-sin(theta)+w3_l.y*cos(theta));
                w3_r=cvPoint(w3_r.x*cos(theta)+w3_r.y*sin(theta),w3_r.x*-sin(theta)+w3_r.y*cos(theta));
                w4_l=cvPoint(w4_l.x*cos(theta)+w4_l.y*sin(theta),w4_l.x*-sin(theta)+w4_l.y*cos(theta));
                w4_r=cvPoint(w4_r.x*cos(theta)+w4_r.y*sin(theta),w4_r.x*-sin(theta)+w4_r.y*cos(theta));
                h=cvPoint(h.x*cos(theta)+h.y*sin(theta),h.x*-sin(theta)+h.y*cos(theta));

                u_left.x+=pos.x;
                u_left.y+=pos.y;
                u_right.x+=pos.x;
                u_right.y+=pos.y;
                l_left.x+=pos.x;
                l_left.y+=pos.y;
                l_right.x+=pos.x;
                l_right.y+=pos.y;

                w1_r.x+=pos.x;
                w1_r.y+=pos.y;
                w1_l.x+=pos.x;
                w1_l.y+=pos.y;
                w2_l.x+=pos.x;
                w2_l.y+=pos.y;
                w2_r.x+=pos.x;
                w2_r.y+=pos.y;
                w3_l.x+=pos.x;
                w3_l.y+=pos.y;
                w3_r.x+=pos.x;
                w3_r.y+=pos.y;
                w4_l.x+=pos.x;
                w4_l.y+=pos.y;
                w4_r.x+=pos.x;
                w4_r.y+=pos.y;
                h.x+=pos.x;
                h.y+=pos.y;



                cvLine(img, u_left, u_right, CV_RGB(0,0,255), 2,8, 0);
                cvLine(img, u_right, l_right, CV_RGB(0,0,255), 2,8, 0);
                cvLine(img, l_right, l_left, CV_RGB(0,0,255), 2,8, 0);
                cvLine(img, l_left, u_left, CV_RGB(0,0,255), 2,8, 0);
                cvFloodFill( img, pos, CV_RGB(0,0,255), cvScalarAll(0), cvScalarAll(0), NULL, 4, NULL );

                CvPoint center1,center2;
                center1.x=(w1_l.x+w1_r.x+w3_r.x+w3_l.x)/4+1;
                center1.y=(w1_l.y+w1_r.y+w3_r.y+w3_l.y)/4+1;

                center2.x=(w2_l.x+w2_r.x+w4_r.x+w4_l.x)/4+1;
                center2.y=(w2_l.y+w2_r.y+w4_r.y+w4_l.y)/4+1;

                cvLine(img, w1_l, w3_l, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w1_l, w1_r, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w1_r, w3_r, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w3_r, w3_l, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w2_l, w4_l, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w2_l, w2_r, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w2_r, w4_r, CV_RGB(255,0,0), 2,8, 0);
                cvLine(img, w4_r, w4_l, CV_RGB(255,0,0), 2,8, 0);

                int phi1, phi2;
                phi1=r.r_phi1();
                phi2=r.r_phi2();
                //double b1=phi1/10;
                //printf(" %f ", b1);

                cvFloodFill( img, center1, CV_RGB(255,0,0), cvScalarAll(0), cvScalarAll(0), NULL, 0, NULL );
                cvFloodFill( img, center2, CV_RGB(255,0,0), cvScalarAll(0), cvScalarAll(0), NULL, 0, NULL );

                cvCircle(img,h,5,CV_RGB(0,255,0),-1);

                CvFont font;
                int    lineWidth=2;
                cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, 1,1,0,lineWidth);

                char buffer[1];
                sprintf(buffer,"%d",r.r_id());
                cvPutText (img,buffer,cvPoint(pos.x-10,pos.y+10), &font, cvScalarAll(255));




        }
