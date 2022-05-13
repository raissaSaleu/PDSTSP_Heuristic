#include "Node.h"

using namespace std;

//Constructor that takes all members as input
Node::Node(int idin, double xin, double yin, int visitin, int dronein) : id(idin), x(xin), y(yin), visit(visitin), drone(dronein) {}

//Copy constructor
Node::Node(Node & source)
{
    x = source.get_x();
    y = source.get_y();
    id = source.get_id();
    visit = source.get_visit();
    drone = source.get_drone();

}

// Default Constructor
Node::Node(){}

//Destructor
Node::~Node(){

    // Destroy the labels list of the node
    std::for_each(labels.begin(), labels.end(), Delete());
    labels.clear();
    std::for_each(labels2.begin(), labels2.end(), Delete());
    labels2.clear();

}

void Node::set_x(double i)
{
    x = i;
}

double Node::get_x()
{
    return x;
}

void Node::set_y(double i)
{
    y = i;
}

double Node::get_y()
{
    return y;
}

void Node::set_id(int i)
{
    id = i;
}

int Node::get_id()
{
    return id;
}

void Node::set_visit(int i)
{
    visit = i;
}

int Node::get_visit()
{
    return visit;
}

void Node::set_drone(int i)
{
    drone = i;
}

int Node::get_drone()
{
    return drone;
}
