#ifndef _RINGLIST_H
#define _RINGLIST_H
#include<stdio.h>  
#include<stdlib.h>  
typedef struct node  
{  
    char name[20];  
    struct node *link;  
}sRoller,*pRoller;  


#endif