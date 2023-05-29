#include <iostream>
#include "services/Program.h"
using namespace std;

    int main()
    {
        system("clear");
        cout<<"Run program with: \n[1] Toy graphs\n[2] Real graphs\n";
        cout<<"Choose an option: ";
        int option;
        cin>>option;
        string x="25793.4";
        double y= stod(x);
        Program program;
        program.run();
        system("clear");
        return 0;
    }


