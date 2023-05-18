#include <string>
#include <vector>
#include <iostream>


#include <mutex>

class Nav
{
private:
    int count_;
    std::mutex mx_;

public:
    Nav() : count_(0)
    { }
    void increment()
    {  
         std::lock_guard<std::mutex> lk(mx_); 
         count_++;
    }
    int getVal(){
        return count_;
    }
};

int main()
{
    Nav nav;
    for (auto i=0; i < 3; ++i)
    {
        Nav nav;
        nav.increment();
    }
    std::cout << nav.getVal() << std::endl;
    return 0;
}