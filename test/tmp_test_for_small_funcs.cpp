
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <assert.h>
// g++ tmp_test_for_small_funcs.cpp -o test.out && ./test.out

// #include "my_pcl/pcl_io.h"
// #include "my_pcl/pcl_commons.h"
// #include "my_pcl/pcl_filters.h"
// #include "my_pcl/pcl_advanced.h"
// using namespace pcl;
// using namespace my_pcl;

using namespace std;

void read_from_file(string filename)
{
    ifstream fin;
    cout << "openning " << filename << endl;
    fin.open(filename);
    char output[100];
    float num;
    int cnt=0;
    if (fin.is_open())
    {
        // while (!fin.eof()){fin >> output;
        while(fin>>output){
            num = stof(output);
            cout << cnt++<<": "<<output << "-->" << num <<  endl;
        }
    }
    fin.close();
    return;
}

void read_T_from_file(float T_16x1[16], string filename)
{
    ifstream fin;
    fin.open(filename);
    float val;
    int cnt=0;
    assert(fin.is_open()); // Fail to find the config file
    while(fin>>val)
        T_16x1[cnt++]=val;
    fin.close();
    return;
}

int main(int argc, char **argv)
{
    cout << "---- TESTING ---- " << endl;

    string folder = "/home/feiyu/baxterws/src/scan3d_by_baxter/config/";
    string file = "T_baxter_to_chess.txt";
    float T_16x1[16];
    read_T_from_file(T_16x1, folder + file);
    for (auto val:T_16x1)cout<<val<<endl;
    return (0);
}