#ifndef _TSP_h
#define _TSP_h

#include "Node.h"

using namespace std;

class tsp
{
    public:

        deque<int> solution; //Stores the tsp solution

        tsp(deque <Node*> &listeNode);
        tsp();
        ~tsp();

        deque <int> get_solution(); //Returns solution
        void set_solution(deque <int> source); //Set solution
        double get_solution_distance(); //Finds total distance of solution

        void copy_deque_to_deque(deque <int> & source, deque <int> & dest);
        int find_index(int id); //Returns the position of node n in original_list

        double nearest_neighbor(int seed); //Generates basic nearest neighbor tour beginning at start_index
        void apply_inversion(int i, int j); //Apply inversion operation moves (2-opt) in segment [i,j]
        void apply_exchange(int i, int j);  //Apply exchange operation moves
        bool gain_cost(int i, int j); //Returns true if applying 2 opt move will yield a cost gain
        void amelioration(); //Apply amelioration with 2-opt moves
        double run_tsp(int seed);//Run tsp algorithm and returns the cost associated to the solution

    private:
        deque <int> original_list; //list of customers + depot

};

#endif



