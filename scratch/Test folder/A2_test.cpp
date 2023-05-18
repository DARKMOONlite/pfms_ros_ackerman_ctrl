

#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>
#include <utility>
#include <set>
#include <stack>
#include <vector>
#include <algorithm>

int main(){
    std::vector<int> goals_;
    goals_.push_back(0);
    goals_.push_back(1);
    goals_.push_back(2); 
    goals_.push_back(3);
    goals_.push_back(4);
    goals_.push_back(5);
    goals_.push_back(6);
    goals_.push_back(7);
    goals_.push_back(8);
    goals_.push_back(9);
    
    uint num=0;

    std::deque<int> visited;
    std::vector<int> indexes;
    std::vector<std::set<int>> Keys(goals_.size()+1); //keys should be 1 bigger. but it is unused
    //fill(Keys.begin(), Keys.end(),std::set<int>());
 
    for(int i =0; i< goals_.size(); i++){
        indexes.push_back(i);
        
        }
    
   
    auto min_path = std::numeric_limits<double>::max();
    auto start = std::chrono::system_clock::now();
    do{
        
        for(int i = 0; i< goals_.size(); i++){ //This checks if each goal has already been reached or checked already
        
            //if(Keys.at(visited.size()).find(i) == Keys.at(visited.size()).end() && visited.find(i) == visited.end()){ // If the goal hasn't already been visited 
        
            if(std::find(visited.begin(), visited.end(),i) == visited.end() && Keys.at(visited.size()).find(i) == Keys.at(visited.size()).end()){ // If the goal hasn't already been visited 
                Keys.at(visited.size()).insert(i);
                visited.push_back(i);
                //stack.push(i); //pushes our new position to the stack
                //visited.insert(stack.top());
                if(visited.size() != goals_.size()){
                    Keys.at(visited.size()).clear(); // since we just added to stack this clears the memory of the level we are just entering. thus 
                }
                
                //calculate distance between
                //if distance is greater than min_total distance then pop
                else{ // if this is the final goal in the list then
                    //check total distance, if smaller than current min distance, then update.
                // std::deque<int> print = visited;
                // std::cout << "Path is : ";
                // for(int j = 0; j < visited.size(); j++){
                //     int x = print.front();
                //     std::cout << x << " " << std::endl;
                //     print.pop_front();

                // }
                num++;
                }
                break; // break out of for loop
            }
            else{
                if(i == goals_.size()-1){ // if it reaches this point. it means that all goals in the next step are either on the stack. or already visited.
                    //visited.erase(stack.top());s
                    //stack.pop(); //go back 1 level
                    visited.pop_back();
                    
                }
            }

        }

        //std::cout << "current queue size: " << visited.size() << std::endl;
    }while(visited.size()!= 0 || Keys.at(0).size() != goals_.size() );
        
        auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed = end-start;
        std::cout << "time taken for " << goals_.size() << " goals: " << elapsed.count() << std::endl;
        std::cout << "number of paths: " << num << std::endl;


    


    return 1;


    // // Start with vector of goals;
    // std::vector<Goal> Order;
    // std::set<int> Passed;
    // std::vector<float> distances;
    
    // //Start by comparing all their distances to and order them by distance into a queue 
    // while(Passed.size() < goals_.size()){
    // distances.clear();
    // distances.assign(goals_.size(),100);
    // for(auto goal : goals_){
        
    //     if(Passed.find(goal.seq)!= Passed.end()){
    //         distances.at(goal.seq) = distanceToGoal(); //Find was to change distance to goal to take variables or some other way.
    //     }
    // }
    // int minimum = std::distance(distances.begin(), std::min_element(distances.begin(), distances.end()));
    // Passed.insert(goals_.at(minimum).seq);
    // Order.push_back(goals_.at(minimum));
    //     // Add a method that allows for ackerman to return the fact that a goal may not be reachable.
    //     // Add function that changes the "postion" of the vehicle and look for next goal from here
    //     // Add if clause that will exit if none of the goals can be reached
    // }
    // //! 2nd component: 2 Opt
    // // At this point we have a full queue of goals and passed has every goal
    // while(1){
    //    vector<std::pair<double, std::pair<int, int>>> swap_distances;
    //    std::vector<Goal> Swap_Queue;
    //     double original_distance = DistanceFromTo(0,Order.size(),Order);
    //     for(int i=0; i< goals_.size(); i++){

    //     }

    //     for(int i=0; i< Order.size(); i++){
    //         for(int j=0; j< Order.size(); j++){
    //             if(i != j){
    //             Swap_Queue = Order;
    //             std::swap(Swap_Queue.at(i), Swap_Queue.at(j));
    //             auto pair = std::make_pair(DistanceFromTo(0,Swap_Queue.size(),Swap_Queue),std::make_pair(i,j));
    //             // makes pair of distance as well as indexes swapped.
    //            swap_distances.push_back(pair);
    //             }
    //         }
    //     }
    //     auto minimum_index = std::min_element(swap_distances.begin(), swap_distances.end());
    //     if(minimum_index->first < original_distance){
    //         std::swap(Order.at(minimum_index->second.first), Order.at(minimum_index->second.second));
    //     }
    //     else{ // if swaping any 2 pairs doesn't make the distance smaller, then we must be optimised.
    //         break;
    //         //! not completed because Alan said don't do it this way.
    //     }
        
    // }
    



}