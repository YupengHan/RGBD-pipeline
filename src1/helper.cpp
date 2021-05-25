#include<iostream>
#include<fstream>
#include <vector>
#include <ctime>
#include <cmath>
#include <unordered_map>

using namespace std;


#ifndef MATHPI
#define MATHPI 3.14159265358979323846
#endif

void readmask(string fn, bool* l) {
    ifstream f(fn);
    string ts;
    for (int i = 0; i < 480*640; i++) {
        f >> ts;
        if (stoi(ts) != 0) {
            l[i] = 1;
        }
        else {
            l[i] = 0;
        }
    }    
}


/*
    read uvz 480*640!!!
*/
void readuvz(string fn, float* uvz) {


    ifstream file(fn);
    string tu, tv, tz; //480 640 ?
    // float minx = 4, miny = 4, minz = 7, max = -4, maxy = -4, maxz = 0.3;    
    // while(f >> tu >> tv >> tz) {
    //     float 
    // }

    // float f = 5.453;
    for (int i = 0; i < 480*640; i++) {
        file >> tu >> tv >> tz;
        uvz[i] = stof(tz);
    }
}

void loadxyz (int su, int sv, int wh, float* uvz, float* xyz, float* xyzlimit) {
    float f = 5.453;
    int i = 0;
    float min_x = 3000, min_y = 3000, min_z = 7000, max_x = -3000, max_y = -3000, max_z = 0;
    for (int cu = su; cu < su+wh; cu++) {//480
        for (int cv = sv; cv < sv+wh; cv++) {//640
            int idx = cu*640+cv;
            float cz = uvz[idx];
            if (cz != 0) {
                float cx = -(cz/f)*((cv - 320)*0.0093+0.063);
                float cy = -(cz/f)*((cu - 240)*0.0093+0.039);
                xyz[3*i + 0] = cx;
                xyz[3*i + 1] = cy;
                xyz[3*i + 2] = cz;
                
                min_x = cx < min_x ? cx : min_x;
                min_y = cy < min_y ? cy : min_y;
                min_z = cz < min_z ? cz : min_z;
                max_x = cx > max_x ? cx : max_x;
                max_y = cy > max_y ? cy : max_y;
                max_z = cz > max_z ? cz : max_z;
            }
            else {
                xyz[3*i + 0] = 0;
                xyz[3*i + 1] = 0;
                xyz[3*i + 2] = 0;
            }
            i++;
        }
    }
    xyzlimit[0] = min_x;
    xyzlimit[1] = max_x;
    xyzlimit[2] = min_y;
    xyzlimit[3] = max_y;
    xyzlimit[4] = min_z;
    xyzlimit[5] = max_z;
}


void loadxyz_downsample (int su, int sv, int wh, float* uvz, float* xyz, float* xyzlimit) {
    float f = 5.453;
    int i = 0;
    float min_x = 3000, min_y = 3000, min_z = 7000, max_x = -3000, max_y = -3000, max_z = 0;
    int delta = wh/20;
    for (int cu = su; cu < su+wh; cu += delta) {//480
        for (int cv = sv; cv < sv+wh; cv += delta) {//640
            int idx = cu*640+cv;
            float cz = uvz[idx];
            if (cz != 0) {
                float cx = -(cz/f)*((cv - 320)*0.0093+0.063);
                float cy = -(cz/f)*((cu - 240)*0.0093+0.039);
                xyz[3*i + 0] = cx;
                xyz[3*i + 1] = cy;
                xyz[3*i + 2] = cz;
                
                min_x = cx < min_x ? cx : min_x;
                min_y = cy < min_y ? cy : min_y;
                min_z = cz < min_z ? cz : min_z;
                max_x = cx > max_x ? cx : max_x;
                max_y = cy > max_y ? cy : max_y;
                max_z = cz > max_z ? cz : max_z;
            }
            else {
                xyz[3*i + 0] = 0;
                xyz[3*i + 1] = 0;
                xyz[3*i + 2] = 0;
            }
            i++;
        }
    }
    xyzlimit[0] = min_x;
    xyzlimit[1] = max_x;
    xyzlimit[2] = min_y;
    xyzlimit[3] = max_y;
    xyzlimit[4] = min_z;
    xyzlimit[5] = max_z;
}

// void readxyz(string fn, float* xyz) {


//     ifstream file(fn);
//     string tu, tv, tz; //480 640 ?
//     // float minx = 4, miny = 4, minz = 7, max = -4, maxy = -4, maxz = 0.3;    
//     // while(f >> tu >> tv >> tz) {
//     //     float 
//     // }

//     float f = 5.453;
//     for (int i = 0; i < 480*640; i++) {
//         file >> tu >> tv >> tz;
//         float u = stof(tu), v = stof(tv), cz = stof(tz);
//         float cx = -(cz/f)*((v - 320)*0.0093+0.063);
//         float cy = -(cz/f)*((u - 240)*0.0093+0.039);
//         xyz[3*i] = cx;
//         xyz[3*i+1] = cy;
//         xyz[3*i+2] = cz;
//     }
// }




void writexyz(string fn, float* xyz, int len) {
    ofstream file(fn);
    float tx, ty, tz; //480 640 ?
    // float minx = 4, miny = 4, minz = 7, max = -4, maxy = -4, maxz = 0.3;
    // while(f >> tu >> tv >> tz) {
    //     float 
    // }
    for (int i = 0; i < len; i++) {
        tx = xyz[3*i];
        ty = xyz[3*i+1];
        tz = xyz[3*i+2];
        if (tx != 0 || ty != 0 || tz != 0) {
            file << tx << " " << ty << " " << tz << endl;
        }
    }
    file.close();
}


void print_pred_center(std::string fn, const int na, std::vector<int> txyz,const int inner_r,const int outer_r) {
    std::ofstream myout;
    myout.open(fn);

    int tx = txyz[0];
    int ty = txyz[1];
    int tz = txyz[2];

    int rnd = 1.0;
    float offset = 2.0/na;
    float increment = MATHPI * (3. - sqrt(5.));

    for (int i = 0; i < na; i++) {
        float y = ((i*offset) - 1) + (offset/2);
        float r = sqrt(1-y*y);

        float phi = ((i + rnd) % na) * increment;
        float x = cos(phi)*r;
        float z = sin(phi)*r;
        
        int ix = round(x*inner_r) + tx;
        int iy = round(y*inner_r) + ty;
        int iz = round(z*inner_r) + tz;

        int ox = round(x*outer_r) + tx;
        int oy = round(y*outer_r) + ty;
        int oz = round(z*outer_r) + tz;

        myout << ix << " " << iy << " " << iz << std::endl;
        myout << ox << " " << oy << " " << oz << std::endl;


    }
    myout.close();

}