
Utilisation de LKH.exe

- créer le fichier Parameter.Win qui contient le nom du fichier d'entrée et du fichier de sortie (voir exemple)

- créer le fichier d'entrée (voir example)

- appeler dans le code C++ la commande : system("..\\LKH-1.3\\LKH.exe ..\\Routes\\Parameter.Win > tempfile.tmp"); 

(le pipe vers tempfile.tmp permet de ne pas afficher la console à l'exécution)

- lire le fichier résultat (voir exemple)

Ci-dessous un bout de code qui fait plus ou moins tout cela : 

        ofstream route_file;
        ifstream route_solution_file;
        ofstream tsplib_file;
        name_route_file = "..\\Routes\\Parameter.Win";
        name_distance_file = "..\\Routes\\route.tsp";
        name_route_solution_file = "..\\Routes\\solution.sol";

        route_file.open(name_route_file.c_str());
        tsplib_file.open(name_distance_file.c_str());

        route_file << "PROBLEM_FILE = " << name_distance_file.c_str() << endl;
        route_file << "TOUR_FILE = " << name_route_solution_file.c_str() << endl;

        route_file.close();
        construct_file(&tsplib_file);
        tsplib_file.close();
        //perform the lin kernigham heuristic
        system("..\\LKH-1.3\\LKH.exe ..\\Routes\\Parameter.Win > tempfile.tmp");
        route_solution_file.open(name_route_solution_file.c_str());
        construct_route(&route_solution_file);
        route_solution_file.close();

