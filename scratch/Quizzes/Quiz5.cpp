#include <iostream>
#include <mutex>

class NavBase
{
protected:
    int count_;
public:
    NavBase() : count_(-1) {}
};

class Nav : public NavBase
{
private:
    std::mutex mx_;

public:
    void multiply()
    {
        std::lock_guard<std::mutex> lk(mx_);
        count_*=count_;
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
        nav.multiply();
    }
    std::cout << nav.getVal() << std::endl;
    return 0;
}