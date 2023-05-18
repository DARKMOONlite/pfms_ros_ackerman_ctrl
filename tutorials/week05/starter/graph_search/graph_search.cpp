#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <queue>
#include <map>
#include <algorithm>

using namespace std;

//! A container for the adjacency list
typedef vector<vector<int> > AdjacencyList;

//! Students should attempt to create a function that prints the adjacency list
void printGraph(AdjacencyList graph){
  int node_num=0;

  for (auto node : graph){
    std::cout << node_num++ << " : ";
    for (auto edge : node){
      std::cout << edge << " " ;
    }
      std::cout << std::endl;
  }

}
//vector<int>
 int Bredth_Search(AdjacencyList graph, int start, int end){
map<int,int> parents;
set<int> Visited; //Make it a set
 queue<int> Queue;//There is a STL Queue



 Queue.push(start); // Push onto the queue the start

for(int j =1; j < 10; j++){
  Visited.insert(Queue.front()); //add the newest location to the Visited list
  std::cout << "Visited List: ";
  for(auto Number : Visited){
    std::cout << Number << "  ";
  }
  std::cout << std::endl;


  for(auto node : graph.at(Queue.front())){ //Go through each node connected to the current node

    if(Visited.find(node) == Visited.end()){ // if the checked not has not been visited 
      Queue.push(node); //then add it to the queue
    }
    if(node == end){ 
    std::cout << "Found it"<< std::endl;
    return(0);}
  

  std::cout << std::endl;

}
Queue.pop();
  // Queue_Mock.erase(Queue_Mock.begin());
  // std::cout << "Current Queue: ";
  // for(auto Number : Queue_Mock){
  //   std::cout << Number << " ";
  // }
}
 }



int main () {
    // Build the graph
    AdjacencyList example_graph = {
        {1, 2},     // Node 0 is connected to nodes 1 and 2 (via edges)
        {0, 4},     // Node 1 is connected to nodes 0 and 4 (via edges)
        {0, 3, 4},  // Node 2 is connected to nodes 0, 3 and 4 (via edges)

        //! Complete for remaining nodes 3,4,5 and 6
        {2, 4, 5},  // Node 3 is connected to nodes 2, 4 and 5 (via edges)
        {1, 2,  3},    
        {3, 6},    
        {5}
    };
    Bredth_Search(example_graph,2,6);

    printGraph(example_graph);


    return 0;
}
