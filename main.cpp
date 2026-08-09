#include "simulation.hpp"

int main(){
    omp_set_num_threads(6);
    simulate();
    return 0;
}