#include "TSP.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include "math.h"

using namespace std;

//Creates a tsp from a list of Nodes
tsp::tsp(deque <Node*> &listeNode)
{
    int taille = listeNode.size();

    //we make sure that original_list is empty
    original_list.clear();

    for (int i = 0; i < taille; ++i)
    {
        original_list.push_back(listeNode[i]->get_id());
    }
}

//Default constructor
tsp::tsp(){}

//Destructor clears original_list and solution
tsp::~tsp()
{
    //original_list.clear();
    solution.clear();
}

//Returns solution
deque <int> tsp::get_solution(){
    return solution;
}

//Set solution
void tsp::set_solution(deque <int> source){

    int length = source.size();

    //we make sure that solution is empty
    solution.clear();

    for (int i = 0; i < length; ++i)
        solution.push_back(source[i]);
}

//Calculate the cost of solution tour
double tsp::get_solution_distance()
{
    double total_dist = 0;
    int taille = solution.size();
    for (int i = 0; i < taille - 1; ++i)
    {
        total_dist += matrixTruck[solution[i]][solution[i+1]];
    }

    total_dist += matrixTruck[solution[taille-1]][solution[0]];

    return total_dist;
 }

//Returns the position of id in original_list
int tsp::find_index(int id){
    int taille = original_list.size(), iter=0, pos = -1;
    bool found = false;
    while(!found && iter<taille){
        if(original_list[iter] == id){
            found = true;
            pos = iter;
        }
        iter++;
    }
    return pos;
}

//Copies a deque of customer id from source to dest
void tsp::copy_deque_to_deque(deque <int> & source, deque <int> & dest)
{
    deque<int>::iterator it;  //An iterator on Labels list

    dest.clear();

    for(it = source.begin(); it!=source.end(); ++it)
        dest.push_back(*it);
}


//Generates nearest neighbor tour in solution from list of customers in original_list.
//K number of nearest neighbors considered
double tsp::nearest_neighbor(int seed)
{
    //cout<<"seed = "<<seed<<endl;
    srand(seed);

    int taille = original_list.size();

    int node_added = 0, length, nb_neighbors=0, choice;
    double closest = 9999999;
    double total_dist = 0;
    double current_dist = 0;
    int closest_index = 0;
    int current_num = taille;
    int start_index = 0;
    deque <int> temp2;
    deque <int> best_neighbors;

    solution.clear();
    solution.push_back(original_list[start_index]);     //move depot to solution
    original_list.erase(original_list.begin() + start_index);       //erase from original_list
    --current_num;        //customers remaining in original_list
    ++node_added;       //number of customers in solution so far
    while(current_num != 0)             //loop until no customer remaining in original_list
    {
        copy_deque_to_deque(original_list, temp2);       //save original list
        nb_neighbors = 0;
        best_neighbors.clear();
        while((nb_neighbors<NB_NEIGHBOR)&&(temp2.size()!=0)){
            closest = 9999999;  //reset closest to a large number so that comparison will work

            length = temp2.size();
            for (int i = 0; i < length; ++i)
            {
                current_dist =  matrixTruck[solution[node_added-1]][temp2[i]];
                if (current_dist < closest)
                {
                    closest_index = i;
                    closest = current_dist;
                }
            }

            best_neighbors.push_back(temp2[closest_index]);
            temp2.erase(temp2.begin() + closest_index);
            nb_neighbors++;
        }

        choice = rand()%best_neighbors.size();
        //cout<<"choice = "<<choice<<endl;

        total_dist += matrixTruck[solution[node_added-1]][best_neighbors[choice]];
        solution.push_back(best_neighbors[choice]);

        original_list.erase(original_list.begin() + find_index(best_neighbors[choice]));

        --current_num;
        ++node_added;
    }
    temp2.clear();

    return total_dist + matrixTruck[solution[0]][solution[node_added-1]];

}


//Apply inversion operation
 void tsp::apply_inversion(int i, int j)
{
    int index1, index2;

    if(j>i){
       index1 = i;
       index2 = j;
    }else{
       index1 = j;
       index2 = i;
    }

    while(index2 > index1){
        apply_exchange(index1,index2);
        index1 ++;
        index2 --;
    }
}

//Apply inversion operation
void tsp::apply_exchange(int i, int j)
{
    int c;

    c = solution[j];
    solution[j] = solution[i];
    solution[i] = c;
}

//Returns true if applying 2 opt move will yield a cost gain
 bool tsp::gain_cost(int i, int j){

    double cost_before = 0; //sum of the deleted edges cost
    double cost_after = 0; //sum of the added edges cost
    int index1, index2, i1, j1;

    int taille = solution.size();
    if(j>i){
       index1 = i;
       index2 = j;
    }else{
       index1 = j;
       index2 = i;
    }

    i1 = index1-1;
    j1 = index2+1;
    if(i1 == -1)
        i1 = taille-1;
    if(j1 == taille)
        j1 = 0;


    cost_before = matrixTruck[solution[i1]][solution[index1]] +
                  matrixTruck[solution[index2]][solution[j1]];

    cost_after = matrixTruck[solution[i1]][solution[index2]] +
                 matrixTruck[solution[index1]][solution[j1]];

    if(cost_after < cost_before){
        return true;
    }else
        return false;
}

//Apply amelioration with 2-opt moves
 void tsp::amelioration(){

    int taille = solution.size();
    bool modif = true;

    while(modif)
    {
        modif = false;

        for(int i=1; i<taille-1; i++){//test all possible 2-opt moves
            for(int j=i+1; j<taille; j++){
                if (gain_cost(i,j)){
                    apply_inversion(i,j);
                    modif = true;
                }
            }
        }
    }
 }

//Run tsp algorithm and returns the cost associated to the solution
double tsp::run_tsp(int seed){
    //srand(time(NULL));
    nearest_neighbor(seed); //apply nearest neighbor heuristic
    //amelioration(); //apply amelioration with 2-opt moves

    return get_solution_distance();
}

