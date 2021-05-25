#include<iostream>
#include<fstream>
#include <vector>
#include <ctime>
#include <cmath>
#include <unordered_map>

// #include <thrust/host_vector.h>
// #include <thrust/device_vector.h>
#include <cuda_runtime_api.h>
#include <cuda.h>

#include</home/yupeng/RGBD-pipeline/MakeSrc/src/helper.cpp>
#include "pl.cuh"
using namespace std;

#ifndef MATHPI
#define MATHPI 3.14159265358979323846
#endif

// nvcc -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcutil -lcudpp -lcuda -lcudart -std=c++11 -c main.cpp; g++ -o file main.o -L/usr/local/cuda/lib64 -lcudart; ./file
// nvcc -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcutil -lcudpp -lcuda -lcudart -std=c++11 -c main.cpp; nvcc -c pl.cu; g++ -o file main.o pl.o -L/usr/local/cuda/lib64 -lcudart; ./file



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
            if (cz != 0 && cz < min_z + 1500) {
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
            if (cz != 0 && cz < min_z + 1500) {
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


int main () {
    string prefix = "/home/yupeng/RGBD-pipeline/data/kinect/txt/";
    size_t score_size = 4 * 60 * 80 * sizeof(int);
    int* host_rgb_score = (int*) malloc(score_size);

    for (int i = 0; i < 4 * 60 * 80; i++) {
        host_rgb_score[i] = 0;
    }
    
    size_t mask_size = 480*640 * sizeof(bool);
    bool* host_mask = (bool*) malloc(mask_size);
    
    bool* device_mask;
    cudaMalloc((void **)&device_mask, mask_size);
    

    int* device_rgb_score;
    cudaMalloc((void **)&device_rgb_score, score_size);
    // cudaMemcpy(device_rgb_score, host_rgb_score, score_size, cudaMemcpyHostToDevice);

    size_t ti = 3*sizeof(int);
    int* host_uvl = (int*) malloc(ti);
    host_uvl[0] = 0;
    host_uvl[1] = 0;
    host_uvl[2] = 0;
    int* device_uvl;
    cudaMalloc((void **)&device_uvl, ti);
    cudaMemcpy(device_uvl, host_uvl, ti, cudaMemcpyHostToDevice);

    size_t uvz_all_size = 1*480*640*sizeof(float);
    float* host_uvz_all = (float*) malloc(uvz_all_size);

    size_t xyz_bbox_size = 3*(128+4)*(128+4)*sizeof(float);
    float* host_xyz_bbox = (float*) malloc(xyz_bbox_size);
    
    float* device_xyz_bbox;
    cudaMalloc((void **)&device_xyz_bbox, xyz_bbox_size);
    

    float* xyz_limit = (float*) malloc(6*sizeof(float));

    int* host_loc_scores = (int*) malloc(100*100*400*sizeof(int));
    
    int* device_loc_scores;
    cudaMalloc((void **)&device_loc_scores, 100*100*400*sizeof(int));
    // cudaMemcpy(device_xyz_bbox, host_xyz_bbox, 100*100*400*sizeof(int), cudaMemcpyHostToDevice);



    float* host_pred_xyz = (float*) malloc(3*sizeof(float));
    float* device_pred_xyz;
    cudaMalloc((void **)&device_pred_xyz, 3*sizeof(float));



    float* device_xyz_limits;
    cudaMalloc((void **)&device_xyz_limits, 6*sizeof(float));
    // cudaMemcpy(device_uvl, host_uvl, ti, cudaMemcpyHostToDevice);


    for (int i = 130; i < 150; i++) {
        std::clock_t start, end, smid, emid;
        start = std::clock();
        string fn = prefix + to_string(i) + ".txt";
        
        
        readmask(fn, host_mask);
        // host_mask[0] = 1;
        // host_mask[9614] = 1;
        // int cnt = 0;
        // for (int i = 0; i < 640*480; i++) {
        //     if (host_mask[i]) {
        //         cnt++;
        //     }
        // }
        // cout << cnt << endl;
        
        readuvz("/home/yupeng/RGBD-pipeline/data/kinect/txt/depth_"+ to_string(i) +".txt", host_uvz_all);
        emid = std::clock();
        std::cout << "read mask image and depth: " << 1000. * (emid - start)/CLOCKS_PER_SEC << "ms" <<std::endl;
        
        cudaMemcpy(device_mask, host_mask, mask_size, cudaMemcpyHostToDevice);
        rgb_window(device_mask, device_rgb_score, device_uvl);

        smid = std::clock();
        cudaMemcpy(host_uvl, device_uvl, ti, cudaMemcpyDeviceToHost);
        emid = std::clock();
        std::cout << "compy mem: " << 1000. * (emid - smid)/CLOCKS_PER_SEC << "ms" <<std::endl;
        cout << host_uvl[0] << " " << host_uvl[1] << " " << host_uvl[2] << endl;
        //shift between RGB - D
        host_uvl[0] +=  8 + (-2);
        host_uvl[1] -= 10 + (-2);

        //expand 4 pixel at here
        int wh;
        if (host_uvl[2] == 0) {
            wh = 16 + 4;
        }
        else if (host_uvl[2] == 1) {
            wh = 32 + 8;
        }
        else if (host_uvl[2] == 2) {
            wh = 64 + 16;
        }
        else if (host_uvl[2] == 3) {
            wh = 128 + 32;
        }

        smid = std::clock();
        loadxyz_downsample(host_uvl[0], host_uvl[1], wh, host_uvz_all, host_xyz_bbox, xyz_limit);
        int ptnum = 20*20;
        // loadxyz(host_uvl[0], host_uvl[1], wh, host_uvz_all, host_xyz_bbox, xyz_limit);
        // int ptnum = wh*wh;
        // writexyz("/home/yupeng/Apr4.xyz", host_xyz_bbox, wh*wh);
        
        cudaMemcpy(device_xyz_bbox, host_xyz_bbox, ptnum *3*sizeof(float), cudaMemcpyHostToDevice);
        // cudaMemcpy(device_xyz_bbox, host_xyz_bbox, xyz_bbox_size, cudaMemcpyHostToDevice);

        emid = std::clock();
        std::cout << "loadxyz: " << 1000. * (emid - smid)/CLOCKS_PER_SEC << "ms" <<std::endl;
        // cout << "x: " << xyz_limit[0] << " " << xyz_limit[1] << endl;
        // cout << "y: " << xyz_limit[2] << " " << xyz_limit[3] << endl;
        // cout << "z: " << xyz_limit[4] << " " << xyz_limit[5] << endl;
        cudaMemcpy(device_xyz_limits, xyz_limit, 6*sizeof(float), cudaMemcpyHostToDevice);
        
        
        smid = std::clock();
        find_loc(device_xyz_bbox, ptnum, device_loc_scores, device_xyz_limits, device_pred_xyz);
        emid = std::clock();
        cudaMemcpy(host_pred_xyz, device_pred_xyz, 3*sizeof(float), cudaMemcpyDeviceToHost);
        std::cout << "pnum: " << ptnum <<  "  findloc: " << 1000. * (emid - smid)/CLOCKS_PER_SEC << "ms" <<std::endl;
        // cudaMemcpy(host_loc_scores, device_loc_scores, 100*100*400*sizeof(int), cudaMemcpyDeviceToHost);
        // for (int i = 0; i < 100*100*400; i++) {
        //     if (host_loc_scores[i] > 0) {
        //         cout << "host_loc_scores[" << to_string(i) << "]: " << host_loc_scores[i] << endl;
        //     }
        // }
        

        cout << "x: " << host_pred_xyz[0] << endl;
        cout << "y: " << host_pred_xyz[1] << endl;
        cout << "z: " << host_pred_xyz[2] << endl;
        vector<int> txyz(3,0);
        txyz[0] = int(host_pred_xyz[0]), txyz[1] = int(host_pred_xyz[1]), txyz[2] = int(host_pred_xyz[2]);
        print_pred_center("/home/yupeng/RGBD-pipeline/out/kinect_"+to_string(i)+".xyz", 1000, txyz, 50, 52);


        // int ccnt = 0;
        // for (int i = 0; i < 4*60*80; i++) {
        //     if (host_rgb_score[i] > 0) {
        //         ccnt++;
        //     }
        // }
        // cout << ccnt << endl;

        // writexyz("/home/yupeng/testApr3_"+ to_string(i) +".txt", host_xyz_all);
        
        




        end = std::clock();
        std::cout << "                        Runtime: " << 1000. * (end - start)/CLOCKS_PER_SEC << "ms" <<std::endl;

    }

    return 0;
    
}