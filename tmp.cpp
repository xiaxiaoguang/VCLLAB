#include <iostream>
#include <string>

using namespace std;

struct A {
    int a;
    int b;
    int c;
    int d;
};

int main() {
    A a1 = {1, 2, 3, 4};

    // Using string array to access members
    string members[] = {"a", "b", "c", "d"};
    for (int i = 0; i < 4; i++) {
        cout << a1.*(members[i].c_str()) << endl;
    }

    // Using pointer arithmetic to access members
    int *ptr = (int *)&a1;
    for (int i = 0; i < 4; i++) {
        cout << *(ptr + i) << endl;
    }

    return 0;
}