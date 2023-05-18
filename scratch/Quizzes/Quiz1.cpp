#include <iostream> 
#include <vector>
#include <iterator>

int main () {

    std::vector<int> myVec = {-1, 6, 8, 5, 11, -7};

    for (auto i = myVec.end()-1 ; i >= myVec.begin() ;i--)
    {
        std::cout << *i  << "   ";
    }
    std::cout << std::endl;
}