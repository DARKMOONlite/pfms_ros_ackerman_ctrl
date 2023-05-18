#include <iostream> 
#include <vector>
#include <iterator>




int main () {
std::vector<int> v = {1,2,3};
    std::vector<int>::iterator it;
for (it = v.end()-1 ; it >= v.begin() ; --it){
    std::cout << *it;
}

}