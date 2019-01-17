

#include "my_basics/basics.h"


namespace my_basics{


string int2str(int num, int width, char char_to_fill){
    std::stringstream ss;
    ss << std::setw(width) << std::setfill(char_to_fill) << num;
    return ss.str();
}

vector<int> getIntersection(vector<int> v1, vector<int> v2)
{
    int comb_len = v1.size() + v2.size();
    std::vector<int> v(comb_len);
    std::vector<int>::iterator it;
    it = std::set_intersection(
        v1.begin(), v1.end(),
        v2.begin(), v2.end(),
        v.begin());
    v.resize(it - v.begin());
    return v;
}


void preTranslatePoint(const float T[4][4], float &x, float &y, float &z){
    double p[4]={x,y,z,1};
    double res[3]={0,0,0};
    for(int row=0;row<3;row++){
        for(int j=0; j<4; j++)
            res[row]+=T[row][j]*p[j];
    }
    x=res[0];
    y=res[1];
    z=res[2];
}

}