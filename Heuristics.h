#ifndef _Heuristics_h
#define _Heuristics_h
#include <fstream>
#include "TSP.h"

using namespace std;

class Heuristics
{
    public:

        int nbGenLab, nbDelLab; //number of generated labels and deleted labels in a split
        Heuristics(const char * filen); //Constructor, takes filename to read from as input
        ~Heuristics();

        void read_file1(const char * filename); //Read our instances files
        void read_file2(const char * filename); //Read Murray and CHU files

        bool dominance_test(Label *l, Label *p); //Returns true if label l is dominated by label p
        bool dominance_test2(Label2 *l, Label2 *p, int id); //Returns true if label l is dominated by label p (case K vehicles)

        double bSup1(deque <Node*> tour); //Returns upper bound proposed by ALAIN
        deque<int> solveSimplePb(deque <int> &tourN, double coef);//Solve the problem 1 vehicle 1 drone considering acceleration coef and returns the vehicle tour
        double bSup2(deque <Node*> &tour); //Returns upper bound proposed by DOMINIQUE
        double bSup3(deque <int> &tourN);//bSup1 for the case with k vehicles
        double bSup4(deque <int> &tourN);//bSup2 for the case with k vehicles
        double* bInf(deque <Node*> tour, double bSup, int option); //Computes lower bound for each node (sum vehicle drone or sum vehicle)
        double** preprocess(deque<int> &tour);//returns V*(x,k) values that will be used in BInf for the case with k vehicles
        double bInf(deque <Node*> tour,int indice, double** V, Label2 *l);//bInf in the case with K vehicles considering a label l
        double decompose(deque<int> &tour, deque< deque<int> > &sol_trucks_tours, int option); //Decomposes tour in NB_VEH tours and returns the acceleration coefficient
        double lengthPath(deque<int> &tour, int y, int x); //Gives the length of the path going from 0 to x in a tour
        double lengthPath2(deque<int> &tour, int i, int j);//Gives the length of the path from index i to j in tour

        //bool labelSortCriterion (const Label2* l1,const Label2* l2);//Returns true if sum(l1->c^gloVeh+l1->c^currVeh+l1->c^drone) < sum(l2->c^gloVeh+l2->c^currVeh+l2->c^drone)
        void filter1(deque <Label2*>& L, int choix);//filters label list L
        int transformLabel(Label2* l); //changes the unit of l
        void filter2(deque <Label2*>& L);//filters label list L

        void applyExchangeMove(deque<int> &tour1, deque<int> &tour2, int i, int j);
        void applyExchange2(deque<int> &tour, deque<int> &drone, int i, int j);
        void applyCrossMove(deque<int> &tour1, deque<int> &tour2, int i, int j);
        void applytransfer1(deque<int> &tour,deque<int> &drone,int i, int j);
        void applytransfer2(deque<int> &tour, int i, deque<int> &drone);
        bool echange(deque<int> &tour1, deque<int> &tour2, double makespan);
        bool echange2(deque<int> &tour, deque<int> &drone, double makespan);
        bool transfer2(deque<int> &tour, deque<int> &drone, double makespan);//transfer a vehicle node to drone
        bool transfer1_echange(deque<int> &tour, deque<int> &drone, double makespan);
        void sortCost(deque < deque<int> > &tours, deque < deque<int> > &drones);//sort tours ans drone according to increasing value of the cost

        Label* split(deque <Node*> tour,int iteration); //Run BELLMAN algorithm
        Label2* split2(deque <Node*> tour,int iteration, int limitLab); //Run BELLMAN algorithm with K vehicles
        double lpt_heuristic (); //Apply Longest Processing Time Heuristic and return the related cost
        void run_heuristic(string path, string file_n); //Run the Two-step heuristic
        void run_heuristic2(string path, string file_n); //Run the Two-step heuristic (case K vehicles)

        void apply_exchange(int i, int j);  //Apply exchange operation moves
        void apply_inversion(int i, int j); //Apply inversion operation moves (2-opt) in segment [i,j]
        bool gain_cost(int i, int j); //Returns true if applying 2 opt move will yield a cost gain
        double amelioration(); //Apply amelioration of truck_tour with 2-opt moves and returns the cost after improvement
        void create_inputFile_forLkh(deque <int> listeN); //Creates the input file for LKH
        double run_lkh(deque <int> listeN); //run LKH and return the result

        Node* find_node_by_id(int id); //Returns a Node given his id by searching in all_customers list
        bool is_in_truck_tour(int id, deque<int> &given_tour); //Returns true if id is in tour
        bool is_drone_eligible(int id); //Returns true if node with id id is in droneEligible list
        void best_insertion(deque<int> &given_tour, int id); //Best insertion of node with id id in given_tour
        void random_insertion(int id); //random insertion of node with id id in truck_tour
        double get_truck_cost(deque <int> & given_tour); //Finds cost related to given_tour
        double get_drone_cost(deque <int> & given_drone_list); //Finds cost related to given_drone_list
        double get_worst_veh_cost(deque < deque<int> > &tours);//Returns worst vehicle cost
        double get_worst_drone_cost(deque < deque<int> > &drones);//Returns worst drone cost
        bool compare_label(Label *l1, Label *l2); //Returns true if l1 is better than l2
        bool compare_label2(Label2 *l1, Label2 *l2); //Returns true if l1 is better than l2
        bool compare_solutions1(Label *l1, Label *l2, deque < deque<int> > &Veh, deque < deque<int> > &Dro, deque < deque<int> > &bestVeh, deque < deque<int> > &bestDro); //Returns true current solution is better than recor solution
        bool compare_solutions2(Label *l1, Label *l2, deque < deque<int> > &Veh, deque < deque<int> > &Dro, deque < deque<int> > &bestVeh, deque < deque<int> > &bestDro); //Returns true current solution is better than recor solution
        void sortDroneDeque(); //Sort drone_list according to the distance to the depot
        int findSmallestDroneTime(double endingTimes[]); //Finds the drone with the smallest ending time
        bool double_equals(double a, double b); //Returns yes is a is equal to b
        Label* find_best_label(deque <Label*> L); //Find the best label in a labels list L
        Label2* find_best_label2(deque <Label2*> L); //Find the best label in a labels list L
        deque<Label2*> find_best_label3(deque <Label2*> &L); //Find the 3 best labels in a labels list L
        int succ(deque<int> &tour, int i); //returns the direct successor of i in tour
        int getIndex(deque<int> &tour, int i); //returns the position index of i in tour
        void statAnalysis(string path, string file_n);//analysis of impact of the quality of the TSP tour
        void limitLabelAnalysis(string path, string file_n);//analysis of impact of the quality of the TSP tour


        double maxi(double a, double b, double c);//returns the max between a, b and c

        deque< deque<int> > transformed(deque<int>* &tab);
        void copy_deque_to_deque(deque <int> & source, deque <int> & dest);//Copies a deque of int from source to dest
        void copy_dequetab_to_dequetab(deque< deque <int> > &source, deque< deque <int> > &dest);
        void concate(deque <int> & tour1, deque <int> & tour2);

        void write_result(const char * file_name, const char * file_n, Label *best_label, int best_nb_truck_customers, int best_nb_drone_customers, double time,int nb_restart, int nbSplitCall,int averageNbLabel,double percentDelLab, double averageGapDrone); //Writes the result of the heuristic into file_name

    private:
        deque <Node*> all_customers; //List of all customers + depot
        deque <int> truck_tour; //truck tour after split
        deque <int> drone_list; //drone customers list after split
        deque < deque<int> > drone_assignment;

        deque <int> best_truck_tour; //best truck tour after all the iterations (in a single start)
        deque <int> best_drone_list; //drone customers list after all the iterations (in a single start)
        deque < deque<int> > best_drone_assignment; //drone assignment after all the iterations (in a single start)

        deque <int> final_truck_tour; //final truck tour after all restarts
        deque <int> final_drone_list; //drone customers list after all restarts
        deque < deque<int> > final_drone_assignment; //final drone assignment after all restart

        deque <int> bsup1_truck_tour; //truck tour associated to bsup1 (we need to keep it for the case where the labels list of depot destination after split is empty))
        deque <int> bsup2_truck_tour;
        deque <int> bsup1_drone_list;
        deque <int> bsup2_drone_list;

        Label* bsup1_label; //two component cost after calculating bsup1 (ALAIN upper bound)
        Label* bsup2_label; //best label after calculating bsup2 (DOMINIQUE upper bound)
        Label* recor_label; //Best label found so far in a restart
        Label* final_recor_label; //Best label found so far after all restarts


        //For the case with K vehicles
        Label* bsup3_label; //two component cost after calculating bsup1 (ALAIN upper bound)
        Label* bsup4_label; //best label after calculating bsup2 (DOMINIQUE upper bound)

        deque < deque<int> > bsup3_trucks_tours;//K trucks tours after bSup procedure
        deque < deque<int> > bsup4_trucks_tours;//K trucks tours after bSup procedure
        deque <int> bsup3_drone_list;
        deque <int> bsup4_drone_list;

        Label* bsup_label;

        deque < deque<int> > trucks_tours;//K trucks tours after the split procedure
        deque < deque<int> > best_trucks_tours;//best trucks tour distribution after all the iterations (in a single start)
        deque < deque<int> > final_trucks_tours;//final trucks tour distribution after all restarts

        Label2* recor_label2; //Best label found so far in a restart
        Label2* final_recor_label2; //Best label found so far after all restarts

        deque <Node*> tour; //contains a tour visiting all the customers
        const char * filename;
};

#endif
