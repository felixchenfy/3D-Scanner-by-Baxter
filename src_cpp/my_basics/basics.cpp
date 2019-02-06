

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

void preTranslatePoint(const vector<vector<float>> &T, float &x, float &y, float &z){
    assert(T.size()==4 && T[0].size()==4);
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

void inv(const float T_src[4][4], float T_dst[4][4]){
    cv::Mat T_src_ = (cv::Mat_<double>(4,4)<<
     T_src[0][0], T_src[0][1], T_src[0][2], T_src[0][3],
     T_src[1][0], T_src[1][1], T_src[1][2], T_src[1][3],
     T_src[2][0], T_src[2][1], T_src[2][2], T_src[2][3],
     T_src[3][0], T_src[3][1], T_src[3][2], T_src[3][3]
    );
    cv::Mat T_dst_ = T_src_.inv();
    for(int i=0;i<4;i++){
        for (int j=0;j<4;j++){
            T_dst[i][j]=T_dst_.at<double>(i,j);
        }
    }
}


}