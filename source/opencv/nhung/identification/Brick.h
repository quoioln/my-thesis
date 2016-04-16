#include <string>

using namespace std;


class Brick{

public:
    Brick(void);
    ~Brick(void);


    string getColour();
    void setColour(string whatColour);

    int getnoteNum();
    void setnoteNum(int whatnoteNum);


private:

    int noteNum;
    string colour;



};
