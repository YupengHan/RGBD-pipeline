#include<iostream>
#include<fstream>
#include <vector>
#include <ctime>

// #include <thrust/host_vector.h>
// #include <thrust/device_vector.h>
#include <cuda_runtime_api.h>
#include <cuda.h>

#include</home/yupeng/RGBD-pipeline/src1/helper.cpp>
#include "pl.h"
using namespace std;


// nvcc -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcutil -lcudpp -lcuda -lcudart -std=c++11 -c main.cpp; g++ -o file main.o -L/usr/local/cuda/lib64 -lcudart; ./file
// nvcc -I/usr/local/cuda/include -L/usr/local/cuda/lib64 -lcutil -lcudpp -lcuda -lcudart -std=c++11 -c main.cpp; nvcc -c pl.cu; g++ -o file main.o pl.o -L/usr/local/cuda/lib64 -lcudart; ./file




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

    size_t pose_size = 100*100*400*3*sizeof(float);
    float* host_pose = (float*)malloc(pose_size);
    float* device_pose;
    cudaMalloc((void **)&device_pose, pose_size);
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
        // writexyz("/home/yupeng/Apr4.xyz", host_xyz_bbox, wh*wh);
        int ptnum = 20*20;
        cudaMemcpy(device_xyz_bbox, host_xyz_bbox, ptnum *3*sizeof(float), cudaMemcpyHostToDevice);
        // cudaMemcpy(device_xyz_bbox, host_xyz_bbox, xyz_bbox_size, cudaMemcpyHostToDevice);

        emid = std::clock();
        std::cout << "loadxyz: " << 1000. * (emid - smid)/CLOCKS_PER_SEC << "ms" <<std::endl;
        // cout << "x: " << xyz_limit[0] << " " << xyz_limit[1] << endl;
        // cout << "y: " << xyz_limit[2] << " " << xyz_limit[3] << endl;
        // cout << "z: " << xyz_limit[4] << " " << xyz_limit[5] << endl;
        int posenum = int(xyz_limit[1]-xyz_limit[0])*int(xyz_limit[3]-xyz_limit[2])*int(xyz_limit[5]-xyz_limit[4]);
        cout << "posenum: " << posenum << endl;




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
        print_pred_center("/home/yupeng/RGBD-pipeline/out_fine/kinect_"+to_string(i)+".xyz", 1000, txyz, 50, 52);


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