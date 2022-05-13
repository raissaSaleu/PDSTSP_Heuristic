#ifndef _Node_h
#define _Node_h

#include "Globals.h"

using namespace std;

typedef struct Label
{
    int current_node_id;
    double c1;
    double c2;
    Label *father;
}Label;


typedef struct Label2 //label definition for the case with K vehicles
{
    int current_node_id;
    double c_gloVeh;
    double c1;
    double c2;
    int numVeh;
    Label2 *father;
}Label2;

struct labelSortCriterion1
{
	inline bool operator()(const Label2* l1, const Label2* l2)
	{
		 return (l1->c_gloVeh+l1->c1+l1->c2) < (l2->c_gloVeh+l2->c1+l2->c2);
	}
};

struct labelSortCriterion2
{
	inline bool operator()(const Label2* l1, const Label2* l2)
	{
		 return (max(max(l1->c1,l1->c2),l1->c_gloVeh) < max(max(l2->c1,l2->c2),l2->c_gloVeh));
	}
};


class Node
{
    public:
        deque <Label*> labels; //Each node has a labels list
        deque <Label2*> labels2; //Each node has a labels list

        Node(int idin, double xin, double yin, int visit, int drone);
        Node(Node & source);
        Node ();
        ~Node();// Destructor

        void set_x(double i); //Set the value of x variable
        double get_x(); //Returns the value of x variable

        void set_y(double i); //Set the value of y variable
        double get_y(); //Returns the value of y variable

        void set_id(int i); //Set the value of id variable
        int get_id(); //Returns the value of id variable

        void set_visit(int i); //Set the value of visit variable
        int get_visit(); //Returns the value of visit variable

        void set_drone(int i); //Set to 0 if the customer is drone eligible and 1 else
        int get_drone(); //Returns the value of drone variable

    private:
        int id;
        double x;
        double y;
        int visit;//equals 1 if the node has already been visited in the TSP tour ans 0 otherwise
        int drone;//equals 0 if the node is drone eligible and 1 otherwise
};

#endif
