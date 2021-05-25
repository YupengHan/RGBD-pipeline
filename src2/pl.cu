#include <stdio.h>
#include <cmath>
#include <math.h>
#include <ctime>



__global__ void findwindow (bool* mask_img,
                            int* scores) {
    int wh_p = blockIdx.x * blockDim.x + threadIdx.x; // threadIdx.x;
    int ui   = blockIdx.y * blockDim.y + threadIdx.y; // 60
    int vi   = blockIdx.z * blockDim.z + threadIdx.z; // 80

    int wh = 16;
    int cwhp = wh_p;
    while(cwhp > 0) {
        wh*=2;
        cwhp--;
    }
    

    if (ui == 0 && vi == 0) {
        //printf("wh:　%d;   wh_p:　%d \n", wh, wh_p);
    }


    int score_id = wh_p * 60 * 80 + ui * 80 + vi;

    int start_u = ui*8; //60 480
    int start_v = vi*8; //80 640
    int sc = 0;
    if (start_u + wh >= 480 || start_v + wh >= 640) {
        scores[score_id] = 0;
        return;
    }

    for (int cu = ui*8; cu < ui*8+wh; cu++) {
        for (int cv = vi*8; cv < vi*8+wh; cv++) {

            int mask_id = cu * 640 + cv;

            if (mask_img[mask_id]) {
                sc = sc+1;   
                //printf("cu: %d;   cv: %d;   mask_id: %d\n" , cu, cv, mask_id);
            }
        }
    }
    scores[score_id] = sc;

    if (sc > 0 && ui == 0 && vi == 0) {
        //printf("sc: %d            wh:　%d ui:　%d vi:　%d \n", sc, wh, ui, vi);
    }

}


__global__ void loop_2d_bbox (int* scores,
                            int* uvl) {
    //float mul = 1.3*1.3*1.3*1.3*1.3;
    float mul = 3*3*3*3*3;
    float final_c_best = 0.;
    //printf("start! \n");
    for (int idx = 0; idx < 4; idx++) {
        //mul /= 3.5;
        mul /= 3;
        int icbest = 0;
        int csu = 0, csv = 0, cwh = 0;
        for (int i = 0; i < 60; i++) {
            for (int j = 0; j < 80; j++) {
                if (scores[idx*4800+i*80+j] > icbest) {
                    icbest = scores[idx*4800+i*80+j];
                    csu = i*8;
                    csv = j*8;
                    cwh = idx;
                    //printf("icbest: %d  su: %d, sv: %d, wh: %d\n", icbest, csu, csv, cwh);
                }
            }
        }


        if (float(icbest)*mul > final_c_best) {
            final_c_best = float(icbest)*mul;
            uvl[0] = csu;
            uvl[1] = csv;
            uvl[2] = cwh;
            //printf("su: %d, sv: %d, wh: %d\n", csu, csv, cwh);
        }
    }
    
}

void rgb_window(bool* mask_img, int* scores, int* uvl) {
    //dim3 grid(4, 1, 1);
    //dim3 block(1, 60, 80);
    dim3 grid(4, 60, 80);
    dim3 block(1, 1, 1);
    
    findwindow<<<grid, block>>>(mask_img, scores);
    
    
    
    dim3 loop_grid(1, 1, 1);
    dim3 loop_block(1, 1, 1);
    loop_2d_bbox<<<loop_grid, loop_block>>>(scores, uvl);
    
    
    
    cudaDeviceSynchronize();

}







__global__ void para_find_loc (float* pts,
                            int ptnum,
                            int* scores,
                            float* xyz_limits) {
    int d_ix = blockIdx.x * blockDim.x + threadIdx.x;
    int d_iy = blockIdx.y * blockDim.y + threadIdx.y;
    int d_iz = blockIdx.z * blockDim.z + threadIdx.z;
    //printf("d_ix: %d d_iy: %d d_iz: %d\n", d_ix, d_iy, d_iz);
    float start_x = xyz_limits[0];
    float start_y = xyz_limits[2];
    float start_z = xyz_limits[4];


    //printf("start_x: %.0f start_y: %.0f start_z: %.0f\n", start_x, start_y, start_z);

    float end_x = xyz_limits[1];
    float end_y = xyz_limits[3];
    float end_z = xyz_limits[5];

    
    float cx = start_x + d_ix*10;
    float cy = start_y + d_iy*10;
    float cz = start_z + d_iz*10;

    if (cx > end_x || cy > end_y || cz > end_z) {
        //printf("cx: %.0f cy: %.0f cz: %.0f end_x: %.0f end_y: %.0f end_z: %.0f ", cx, cy, cz, end_x, end_y, end_z);
        scores[d_ix*100*400+d_iy*400+d_iz] = 0;
        return;
    }
    //printf("cx: %.0f cy: %.0f cz: %.0f end_x: %.0f end_y: %.0f end_z: %.0f \n", cx, cy, cz, end_x, end_y, end_z);
    
    
    
    int cnt = 0;
    for(int i = 0; i < ptnum; i++) {
        float tx = pts[i*3];
        float ty = pts[i*3+1];
        float tz = pts[i*3+2];
        if (tz > cz) continue;
        float d2c = sqrt((tx-cx)*(tx-cx) + (ty-cy)*(ty-cy) + (tz-cz)*(tz-cz));
        //printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);  
        
        /*
        if (d2c < 1000) {
            printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);  
        }
        */
        //printf("tx: %.0f ty: %.0f tz: %.0f cx: %.0f cy: %.0f cz: %.0f \n", tx, ty, tz, cx, cy, cz);
        //printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);
        
        
        
        if (d2c >= 50 && d2c <= 53 ) {
            cnt += 1;
        }
    }
    scores[d_ix*100*400+d_iy*400+d_iz] = cnt;
}




__global__ void find_best_score (int* scores,
                                float* xyz_limits,
                                float* device_pred_xyz) {
    int c_best = 0;
    device_pred_xyz[0] = -10000;
    device_pred_xyz[1] = -10000;
    device_pred_xyz[2] = -10000;


    int ixmax = int((xyz_limits[1] - xyz_limits[0])/10);
    if (ixmax > 100) ixmax = 100;
    int iymax = int((xyz_limits[3] - xyz_limits[2])/10);
    if (iymax > 100) iymax = 100;
    int izmax = int((xyz_limits[5] - xyz_limits[4])/10);
    //if (izmax > 400) izmax = 400;
    if (izmax > 100) izmax = 100;
    printf("ixmax : %d;  iymax : %d;  izmax : %d\n", ixmax, iymax, izmax);

    for (int ix = 0; ix < ixmax; ix++) {
        for (int iy = 0; iy < iymax; iy++) {
            for (int iz = 0; iz < izmax; iz++) {
                //c_best = c_best > scores[ix*100*400+iy*400+iz] ? c_best : scores[ix*100*400+iy*400+iz];
                if (c_best < scores[ix*100*400+iy*400+iz]) {
                    c_best = scores[ix*100*400+iy*400+iz];
                    device_pred_xyz[0] = xyz_limits[0] + 10*ix;
                    device_pred_xyz[1] = xyz_limits[2] + 10*iy;
                    device_pred_xyz[2] = xyz_limits[4] + 10*iz;
                    //printf("Score: %d    x: %.0f    y: %.0f      z:%.0f \n", c_best, device_pred_xyz[0], device_pred_xyz[1], device_pred_xyz[2]);
                }
                
            }
        }
    }
}

void find_loc(float* pts, int ptnum, int* scores, float* xyz_limits, float* device_pred_xyz) {

    
    //dim3 grid(10, 100, 1);
    //dim3 block(10, 1, 400);
    
    dim3 grid(100, 100, 2);
    dim3 block(1, 1, 50);
    std::clock_t start, end;
    start = std::clock();
    para_find_loc<<<grid, block>>>(pts, ptnum, scores, xyz_limits);
    end = std::clock();
    printf("para_find_loc: %.3f ms\n", 1000. * (end - start)/CLOCKS_PER_SEC);
    
    start = std::clock();
    find_best_score<<<1, 1>>>(scores, xyz_limits, device_pred_xyz);
    end = std::clock();
    printf("find_best_score: %.3f ms\n", 1000. * (end - start)/CLOCKS_PER_SEC);




    

    cudaDeviceSynchronize();
}





__global__ void para_find_loc_fine (float* pts,
                            int ptnum,
                            int* scores,
                            float* xyz_limits) {
    int d_ix = blockIdx.x * blockDim.x + threadIdx.x;
    int d_iy = blockIdx.y * blockDim.y + threadIdx.y;
    int d_iz = blockIdx.z * blockDim.z + threadIdx.z;
    //printf("d_ix: %d d_iy: %d d_iz: %d\n", d_ix, d_iy, d_iz);
    float start_x = xyz_limits[0];
    float start_y = xyz_limits[2];
    float start_z = xyz_limits[4];


    //printf("start_x: %.0f start_y: %.0f start_z: %.0f\n", start_x, start_y, start_z);

    float end_x = xyz_limits[1];
    float end_y = xyz_limits[3];
    float end_z = xyz_limits[5];

    
    /*
    float cx = start_x + d_ix*10;
    float cy = start_y + d_iy*10;
    float cz = start_z + d_iz*10;
    */

    float cx = start_x + d_ix*5;
    float cy = start_y + d_iy*5;
    float cz = start_z + d_iz*5;

    if (cx > end_x || cy > end_y || cz > end_z) {
        //printf("cx: %.0f cy: %.0f cz: %.0f end_x: %.0f end_y: %.0f end_z: %.0f ", cx, cy, cz, end_x, end_y, end_z);
        scores[d_ix*100*400+d_iy*400+d_iz] = 0;
        return;
    }
    //printf("cx: %.0f cy: %.0f cz: %.0f end_x: %.0f end_y: %.0f end_z: %.0f \n", cx, cy, cz, end_x, end_y, end_z);
    
    
    
    int cnt = 0;
    for(int i = 0; i < ptnum; i++) {
        float tx = pts[i*3];
        float ty = pts[i*3+1];
        float tz = pts[i*3+2];
        if (tz > cz) continue;
        float d2c = sqrt((tx-cx)*(tx-cx) + (ty-cy)*(ty-cy) + (tz-cz)*(tz-cz));
        //printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);  
        
        /*
        if (d2c < 1000) {
            printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);  
        }
        */
        //printf("tx: %.0f ty: %.0f tz: %.0f cx: %.0f cy: %.0f cz: %.0f \n", tx, ty, tz, cx, cy, cz);
        //printf("tx: %.0f ty: %.0f tz: %.0f          d2c: %.0f\n", tx, ty, tz, d2c);
        
        
        
        if (d2c >= 51 && d2c <= 54 ) {
            cnt += 1;
        }
    }
    scores[d_ix*100*400+d_iy*400+d_iz] = cnt;
}

__global__ void find_best_score_fine (int* scores,
                                float* xyz_limits,
                                float* device_pred_xyz) {
    int c_best = 0;
    device_pred_xyz[0] = -10000;
    device_pred_xyz[1] = -10000;
    device_pred_xyz[2] = -10000;


    int ixmax = int((xyz_limits[1] - xyz_limits[0])/2);
    if (ixmax > 100) ixmax = 100;
    int iymax = int((xyz_limits[3] - xyz_limits[2])/2);
    if (iymax > 100) iymax = 100;
    int izmax = int((xyz_limits[5] - xyz_limits[4])/2);
    if (izmax > 400) izmax = 400;
    printf("ixmax : %d;  iymax : %d;  izmax : %d\n", ixmax, iymax, izmax);

    for (int ix = 0; ix < ixmax; ix++) {
        for (int iy = 0; iy < iymax; iy++) {
            for (int iz = 0; iz < izmax; iz++) {
                //c_best = c_best > scores[ix*100*400+iy*400+iz] ? c_best : scores[ix*100*400+iy*400+iz];
                if (c_best < scores[ix*100*400+iy*400+iz]) {
                    c_best = scores[ix*100*400+iy*400+iz];
                    device_pred_xyz[0] = xyz_limits[0] + 5*ix;
                    device_pred_xyz[1] = xyz_limits[2] + 5*iy;
                    device_pred_xyz[2] = xyz_limits[4] + 5*iz;
                    //printf("Score: %d    x: %.0f    y: %.0f      z:%.0f \n", c_best, device_pred_xyz[0], device_pred_xyz[1], device_pred_xyz[2]);
                }
                
            }
        }
    }
}



void find_loc_fine(float* pts, int ptnum, int* scores, float* xyz_limits, float* device_pred_xyz) {

    
    //dim3 grid(10, 100, 1);
    //dim3 block(10, 1, 400);
    
    dim3 grid(100, 10, 8);
    dim3 block(1, 10, 50);
    para_find_loc_fine<<<grid, block>>>(pts, ptnum, scores, xyz_limits);


    find_best_score<<<1, 1>>>(scores, xyz_limits, device_pred_xyz);





    

    cudaDeviceSynchronize();
}