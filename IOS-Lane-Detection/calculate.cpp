//
//  calculate.cpp
//  IOS-Lane-Detection
//
//  Created by juedoul on 12/5/15.
//  Copyright © 2015 geoffrey. All rights reserved.
//

#include "calculate.hpp"
#include "iostream"


#define IF_PTR_NULL(ptr, result)\
{\
    if ((NULL) == (ptr))\
    {\
        printf("%s:%d [%s] ",__FILE__,__LINE__,__FUNCTION__);\
        printf("NULL == ptr \n");\
        return result;\
    }\
}

/************************************************************************
 getAngle
 according to the 2 points of a line, get the angle of the line,
 fron x-axis rotation interclockwise to the line
 *
 *					|
 *					|
 *					|
 *					|
 *------------------------------------> x
 *					| 0
 *					|
 *					|
 *					|
 *                   v
 *					y
 *
 return degree
 **************************************************************************/
double getAngle(CvPoint pointO,CvPoint pointA)
{
    double angle = 0;
    CvPoint point;
    double temp;
    
    point = cvPoint((pointA.x - pointO.x), (pointA.y - pointO.y));
    //the same point
    if ((0==point.x) && (0==point.y))
    {
        return 0;
    }
    //a line parallel to y-axis
    if (0==point.x)
    {
        angle = 90;
        return angle;
    }
    // a line parallel to x-axis
    if (0==point.y)
    {
        angle = 0;
        return angle;
    }
    
    temp = fabsf(float(point.y)/float(point.x));
    temp = atan(temp);
    temp = temp*180/CV_PI ;
    
    if ((0<point.x)&&(0<point.y))
    {
        angle = 360 - temp;
        return angle;
    }
    
    if ((0>point.x)&&(0<point.y))
    {
        angle = 360 - (180 - temp);
        return angle;
    }
    
    if ((0<point.x)&&(0>point.y))
    {
        angle = temp;
        return angle;
    }
    
    if ((0>point.x)&&(0>point.y))
    {
        angle = 180 - temp;
        return angle;
    }
    
    printf("sceneDrawing :: getAngle error!");
    return -1;
}


//getDistance, return distance between 2 points
double getDistance (CvPoint pointO,CvPoint pointA )
{
    double distance;
    distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);
    distance = sqrtf(distance);
    
    return distance;
}


//create a linklist with length num
Line * linkListCreat(int num, Line * pcornerPair)
{
    Line *head, *p, *s;
    //double ax,ay,bx,by;
    head = (Line*)malloc(sizeof(Line));
 //   IF_PTR_NULL(head,NULL);
    if(num ==0){
        head->next = NULL;
        head->nextIsNull = 1;
    }
    p = head;
    for( int i = 0; i < num; i++ )
    {
        s = (Line*)malloc(sizeof(Line));
    //    IF_PTR_NULL(s,NULL);
        *s = pcornerPair[i];
        p->next = s;
        p = s;
    }
    
    head = head->next;
    p -> next = NULL;
    p -> nextIsNull = 1;
    return(head);
}

//create a linklist according to the output of houghlineP
Line * hough_link_list_create(cv::vector<cv::Vec4i> lines)
{
    int num = int(lines.size());
    
    Line *head,*p,*s;
    //double ax,ay,bx,by;
    head = (Line*)malloc(sizeof(Line));
    if(num == 0){
        head->next = NULL;
        head->nextIsNull = 1;}
 //   IF_PTR_NULL(head,NULL);
    p = head;
    for( int i = 0; i < num; i++ )
    {
        cv::Vec4i l = lines[i];
        s = (Line*)malloc(sizeof(Line));
  //      IF_PTR_NULL(s,NULL);
        //determine which is the start point which is the end point according to y coordinate
        if (l[1] > l[3])
        {
            s->p1  = cvPoint(l[0], l[1]);
            s->p2  = cvPoint(l[2], l[3]);
        }
        else
        {
            s->p1  = cvPoint(l[2], l[3]);
            s->p2  = cvPoint(l[0], l[1]);
        }
        
        s->angle = getAngle(s->p1,s->p2);
        s->lenth = getDistance(s->p1,s->p2);
        
        p->next = s;
        p = s;
    }
    
    head = head->next;
    p -> next = NULL;
    p->nextIsNull = 1;

    return(head);
}

// delete a node from the linklist, return the head of the linklist
Line * delnode(Line * h , Line * maxp)
{
    Line * t;
    
    if (h==maxp)                         //if maxp == head
    {
        t= maxp-> next;					 //return the next
        maxp = NULL;
        return t;
    }
    else								//or
    {
        t=h;
        while(t->next!=maxp ) {t=t->next;}   //find the node before the maxp
        t->next = maxp->next ;                //delete maxp, link maxp->next to the node
        free(maxp);
        maxp = NULL;
        return h;							  //return head
    }
}

// if the angle is within the threshold
bool angle_ok(double ang, double angThresh)
{
    bool flag = false;
    
    if (ang == 180)
    {
        return flag;
    }
    
    if (ang < 180)
    {
        if (std::abs(ang - 90.00) < angThresh )
        {
            flag = true;
            
        }
        else
        {
            flag = false;
        }
    }
    
    if (ang > 180)
    {
        if (std::abs(ang - 270) < angThresh )
        {
            flag = true;
            
        }
        else
        {
            flag = false;
        }
    }
    
    return flag;
}
//modify a linklist, if the line in this linklist is not in the thresh angle, delete.
Line * angleThresh(Line* head, double angThresh)
{
    if(head != NULL){
        if(!head->nextIsNull ){
            Line * p = head;
            Line pBak = *head;
            
            while(NULL != p)
            {
                std::cout<<p->angle<<std::endl;
                
                if (angle_ok(p->angle,angThresh) == true)
                {
                    p = p->next;
                }
                else
                {
                    pBak = * p;
                    head = delnode(head,p);
                    p = pBak.next;
                }
            }
            
            return head;

        }
        else{
            if (angle_ok(head->angle, angThresh) == true)
            {
                return head;
                
            }
            else
            {
                head=delnode(head,head);
                return NULL;
            }

        }
    }
    else{
        return NULL;
    }
    
    
}

int listLengGet(Line*head)
{
    int n = 0;
    Line *p;
    p = head;
    while(p != NULL)
    {
        p = p->next;
        n++;
    }
    return(n);
}

//delete a node from the linklist, return the head
Line * delNodeForSort(Line * h , Line * maxp)
{
    Line * t;
    
    if (h == maxp)                                //if maxp==head, delete head and return head->next
    {
        t = maxp-> next;
        maxp-> next = NULL;
        return t;
    }
    else
    {
        t = h;
        while(t->next != maxp )
        {
            t = t->next;
        }
        t->next = maxp->next ;
        maxp->next = NULL;
        return h;
    }
}

//sort the linklist according to the length of the line
Line * linklenthSort(Line *st)
{
    if(NULL == st)
    {
        //printf("delnode : NULL == st!");
        return NULL;
    }
    
    Line * h = NULL , *t = NULL, *maxp = NULL, *head = NULL, *end = NULL;
    double maxn;
    h = st;                                     // h = head
    while(NULL!= h)                             //look until the linklist is not NULL
    {                                           
        t = h;                                  // t is a temp
        maxn = t->lenth;
        maxp = t;                               //assume t has the maximum length
        while (t->next != NULL)                 //loop if there are other nodes after t
        {                                       
            t = t->next ;
            if (t -> lenth > maxn)                
            {	
                maxn = t->lenth;
                maxp = t;
            } 
        }                                       //find the line with longest length in the list
        
        h = delNodeForSort(h,maxp);             //delete the longest one from the original one, then create a new one
        maxp->next = NULL;			            //maxp  is the new linklist with a decreasing order of the length
        if (head == NULL)                       //If the head is NULL, maxp is the head
        {
            head = maxp;
            end = maxp;
        }
        else                                    //else, maxp is the end of the linklist
        {
            end->next = maxp;
            end = end->next;
        }
    }
    
    return head;
}

