#include <stdlib.h>
#include <iostream>
#include <memory>
#include <vector>

using namespace std;

class Base
{
public:
    int a;
};

class Derived:public Base
{
public:
    int b;
};


int main()
{
    //Old style rock'n'roll
    vector<Base*> old_style;
    Base *old_base = new Base();
    Derived *old_derived = new Derived();
    old_base->a=1;old_derived->b=5;
    old_style.push_back(old_base);
    old_style.push_back(old_derived);

    while(!old_style.empty())
    {
        cout << old_style.at(old_style.size()-1)->a << endl;
        delete old_style.at(old_style.size()-1);    //without this delete the world won't be happy...
        old_style.pop_back();
    }


    //New hippy flavored
    vector<shared_ptr<Base>> new_style;
    shared_ptr<Base> new_base = make_shared<Base>();
    shared_ptr<Derived> new_derived = make_shared<Derived>();
    new_base->a=11;new_derived->b=55;
    new_style.push_back(new_base);
    new_style.push_back(new_derived);

    while(!new_style.empty())
    {
        cout << new_style.at(new_style.size()-1)->a << endl;
        new_style.pop_back();
    }

    // perform a check with valgrind
    // valgrind --leak-check=yes ./shared_test PROGRAM

    return 0;
}
