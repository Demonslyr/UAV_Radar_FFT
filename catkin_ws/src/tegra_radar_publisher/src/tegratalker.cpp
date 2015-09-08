///////////////////////////////////////////////////////////////////
// Title: Radar Fourier Transform
//
// Copyright (C) 2014  Daniel Murtha
//
// This file is distributed under GPLv2 Licence
///////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////
// Conditional Compilation Options
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////
// Included Files
///////////////////////////////////////////////////////////////////
#include <fftw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <assert.h>
#include <boost/thread.hpp>
#include <string.h>
#include <usb.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "std_msgs/String.h"
#include "ros/ros.h"
using namespace std;

#define USE_TIMING_ITER 50
#define BUFSIZE  1048576
#define TALKER_NAME "Talker" //change made


///////////////////////////////////////////////////////////////////
// USB Device Finder
///////////////////////////////////////////////////////////////////

struct usb_device *device;
usb_dev_handle *handle;
char buf[BUFSIZE];

// find the first ucecho device
struct usb_device *find_device ()
{
    struct usb_bus *bus_search;
    struct usb_device *device_search;

    bus_search = usb_busses;
    while (bus_search != NULL)
    {
	device_search = bus_search->devices;
    	while (device_search != NULL)
	{
	    if ( (device_search->descriptor.idVendor == 0x221a) && (device_search->descriptor.idProduct == 0x100) ) 
	    {
		handle = usb_open(device_search);
		usb_get_string_simple(handle, device_search->descriptor.iProduct, buf, BUFSIZE);
		std::cout<<buf<<std::endl;
		if ( ! strncmp(buf, "GPU2 ", 5 )  ){
		    return device_search;}
		usb_close(handle);
	    }
	    device_search = device_search->next;
	}
        bus_search = bus_search->next;
    }
    return NULL;
}


///////////////////////////////////////////////////////////////////
// Union Declaration to parse FPGA inflow
///////////////////////////////////////////////////////////////////

union I{
	unsigned char o[2];
	short n;
};


///////////////////////////////////////////////////////////////////
// Thread functions
///////////////////////////////////////////////////////////////////

void IO_func(vector<vector<double> >& input, vector<vector<double> >& output, double han_1[], double han_2[], char buf[], I testarr[],usb_dev_handle* handle){
	
	int cont = (BUFSIZE/2),y,z;
	y = usb_bulk_read(handle, 0x82, buf, BUFSIZE, 1000);
	if ( y < 0 ) {
	    fprintf(stderr, "Error readin data: %s\n", usb_strerror());
		}
	for(int i=0;i<cont;i++){
		testarr[i].o[0]=buf[(2*i)+0];
		testarr[i].o[1]=buf[(2*i)+1];
		}
	for(int j=512, z=cont-1;j--;) {
		for(int i=801; i--;z--) {
			output[j][i] = testarr[z].n * han_1[i] * han_2[j];
			}
		}
	}

///////////////////////////////////////////////////////////////////
//FFT Function
///////////////////////////////////////////////////////////////////

void FFT_func(vector<vector<double> >& input, vector<vector<vector<double> > >& output, double* Fin, fftw_complex* Fout,fftw_plan& plan)
{
    for(int j=512;j--;) {
		for(int i=801; i--;) {
			Fin[i] = input[j][i];
		}
		fftw_execute(plan);
		for(int l=401; l--;) {
			output[j][l][0] = Fout[l][0];
			output[j][l][1] = Fout[l][1];
		}
    }
}

///////////////////////////////////////////////////////////////////
// Time difference method
///////////////////////////////////////////////////////////////////

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

///////////////////////////////////////////////////////////////////
// Main method
///////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{

    ///////////////////////////////////////////////////////////
    // Variables
    ///////////////////////////////////////////////////////////
    char bkt[14]; //for reading in .csv files for testing
    
    char* buf = new char [BUFSIZE];
    
    I* testarr = new I [(BUFSIZE/2)];
    
    int i=0,j=0,h=0,k=0,id=0,y=0;

    double *in;//real input array
    double totalns=0,totals=0,itotalns=0,itotals=0;//timing vars

    //Horizontal hanning window
    double hanning_1 [801] = {0,0.063165,0.25266,0.56846,1.0106,1.5789,2.2735,3.0943,4.0413,5.1143,6.3133,7.6383,9.0891,10.666,12.368,14.196,16.149,18.228,20.432,22.76,25.214,27.793,30.496,33.324,36.276,39.352,42.552,45.875,49.322,52.893,56.586,60.403,64.342,68.403,72.586,76.892,81.319,85.867,90.536,95.326,100.24,105.27,110.42,115.69,121.08,126.58,132.21,137.96,143.82,149.8,155.89,162.11,168.44,174.88,181.45,188.12,194.91,201.82,208.84,215.97,223.22,230.58,238.05,245.63,253.32,261.13,269.04,277.07,285.2,293.44,301.79,310.25,318.82,327.49,336.27,345.15,354.14,363.23,372.43,381.73,391.13,400.64,410.25,419.95,429.76,439.67,449.68,459.78,469.99,480.29,490.69,501.18,511.77,522.46,533.24,544.11,555.07,566.13,577.28,588.52,599.85,611.26,622.77,634.37,646.05,657.82,669.67,681.61,693.63,705.74,717.93,730.2,742.56,754.99,767.5,780.1,792.77,805.51,818.34,831.24,844.22,857.27,870.39,883.59,896.85,910.19,923.6,937.08,950.63,964.24,977.92,991.67,1005.5,1019.4,1033.3,1047.3,1061.4,1075.5,1089.7,1103.9,1118.2,1132.6,1147,1161.5,1176,1190.6,1205.2,1219.9,1234.6,1249.4,1264.3,1279.1,1294.1,1309.1,1324.1,1339.2,1354.3,1369.4,1384.6,1399.9,1415.1,1430.5,1445.8,1461.2,1476.6,1492.1,1507.6,1523.1,1538.7,1554.3,1569.9,1585.6,1601.2,1617,1632.7,1648.5,1664.2,1680.1,1695.9,1711.7,1727.6,1743.5,1759.4,1775.4,1791.3,1807.3,1823.3,1839.3,1855.3,1871.3,1887.3,1903.4,1919.4,1935.5,1951.5,1967.6,1983.7,1999.7,2015.8,2031.9,2048,2064.1,2080.2,2096.3,2112.3,2128.4,2144.5,2160.5,2176.6,2192.6,2208.7,2224.7,2240.7,2256.7,2272.7,2288.7,2304.7,2320.6,2336.6,2352.5,2368.4,2384.3,2400.1,2415.9,2431.8,2447.5,2463.3,2479,2494.8,2510.4,2526.1,2541.7,2557.3,2572.9,2588.4,2603.9,2619.4,2634.8,2650.2,2665.5,2680.9,2696.1,2711.4,2726.6,2741.7,2756.8,2771.9,2786.9,2801.9,2816.9,2831.7,2846.6,2861.4,2876.1,2890.8,2905.4,2920,2934.5,2949,2963.4,2977.8,2992.1,3006.3,3020.5,3034.6,3048.7,3062.7,3076.6,3090.5,3104.3,3118.1,3131.8,3145.4,3158.9,3172.4,3185.8,3199.1,3212.4,3225.6,3238.7,3251.8,3264.8,3277.7,3290.5,3303.2,3315.9,3328.5,3341,3353.4,3365.8,3378.1,3390.3,3402.4,3414.4,3426.3,3438.2,3450,3461.6,3473.2,3484.7,3496.2,3507.5,3518.7,3529.9,3540.9,3551.9,3562.8,3573.5,3584.2,3594.8,3605.3,3615.7,3626,3636.2,3646.3,3656.3,3666.2,3676,3685.8,3695.4,3704.9,3714.3,3723.6,3732.8,3741.9,3750.8,3759.7,3768.5,3777.2,3785.7,3794.2,3802.6,3810.8,3818.9,3827,3834.9,3842.7,3850.4,3858,3865.4,3872.8,3880,3887.2,3894.2,3901.1,3907.9,3914.6,3921.1,3927.6,3933.9,3940.1,3946.2,3952.2,3958,3963.8,3969.4,3974.9,3980.3,3985.6,3990.7,3995.8,4000.7,4005.5,4010.1,4014.7,4019.1,4023.4,4027.6,4031.7,4035.6,4039.4,4043.1,4046.7,4050.1,4053.4,4056.6,4059.7,4062.7,4065.5,4068.2,4070.8,4073.2,4075.6,4077.8,4079.9,4081.8,4083.6,4085.3,4086.9,4088.4,4089.7,4090.9,4092,4092.9,4093.7,4094.4,4095,4095.4,4095.7,4095.9,4096,4095.9,4095.7,4095.4,4095,4094.4,4093.7,4092.9,4092,4090.9,4089.7,4088.4,4086.9,4085.3,4083.6,4081.8,4079.9,4077.8,4075.6,4073.2,4070.8,4068.2,4065.5,4062.7,4059.7,4056.6,4053.4,4050.1,4046.7,4043.1,4039.4,4035.6,4031.7,4027.6,4023.4,4019.1,4014.7,4010.1,4005.5,4000.7,3995.8,3990.7,3985.6,3980.3,3974.9,3969.4,3963.8,3958,3952.2,3946.2,3940.1,3933.9,3927.6,3921.1,3914.6,3907.9,3901.1,3894.2,3887.2,3880,3872.8,3865.4,3858,3850.4,3842.7,3834.9,3827,3818.9,3810.8,3802.6,3794.2,3785.7,3777.2,3768.5,3759.7,3750.8,3741.9,3732.8,3723.6,3714.3,3704.9,3695.4,3685.8,3676,3666.2,3656.3,3646.3,3636.2,3626,3615.7,3605.3,3594.8,3584.2,3573.5,3562.8,3551.9,3540.9,3529.9,3518.7,3507.5,3496.2,3484.7,3473.2,3461.6,3450,3438.2,3426.3,3414.4,3402.4,3390.3,3378.1,3365.8,3353.4,3341,3328.5,3315.9,3303.2,3290.5,3277.7,3264.8,3251.8,3238.7,3225.6,3212.4,3199.1,3185.8,3172.4,3158.9,3145.4,3131.8,3118.1,3104.3,3090.5,3076.6,3062.7,3048.7,3034.6,3020.5,3006.3,2992.1,2977.8,2963.4,2949,2934.5,2920,2905.4,2890.8,2876.1,2861.4,2846.6,2831.7,2816.9,2801.9,2786.9,2771.9,2756.8,2741.7,2726.6,2711.4,2696.1,2680.9,2665.5,2650.2,2634.8,2619.4,2603.9,2588.4,2572.9,2557.3,2541.7,2526.1,2510.4,2494.8,2479,2463.3,2447.5,2431.8,2415.9,2400.1,2384.3,2368.4,2352.5,2336.6,2320.6,2304.7,2288.7,2272.7,2256.7,2240.7,2224.7,2208.7,2192.6,2176.6,2160.5,2144.5,2128.4,2112.3,2096.3,2080.2,2064.1,2048,2031.9,2015.8,1999.7,1983.7,1967.6,1951.5,1935.5,1919.4,1903.4,1887.3,1871.3,1855.3,1839.3,1823.3,1807.3,1791.3,1775.4,1759.4,1743.5,1727.6,1711.7,1695.9,1680.1,1664.2,1648.5,1632.7,1617,1601.2,1585.6,1569.9,1554.3,1538.7,1523.1,1507.6,1492.1,1476.6,1461.2,1445.8,1430.5,1415.1,1399.9,1384.6,1369.4,1354.3,1339.2,1324.1,1309.1,1294.1,1279.1,1264.3,1249.4,1234.6,1219.9,1205.2,1190.6,1176,1161.5,1147,1132.6,1118.2,1103.9,1089.7,1075.5,1061.4,1047.3,1033.3,1019.4,1005.5,991.67,977.92,964.24,950.63,937.08,923.6,910.19,896.85,883.59,870.39,857.27,844.22,831.24,818.34,805.51,792.77,780.1,767.5,754.99,742.56,730.2,717.93,705.74,693.63,681.61,669.67,657.82,646.05,634.37,622.77,611.26,599.85,588.52,577.28,566.13,555.07,544.11,533.24,522.46,511.77,501.18,490.69,480.29,469.99,459.78,449.68,439.67,429.76,419.95,410.25,400.64,391.13,381.73,372.43,363.23,354.14,345.15,336.27,327.49,318.82,310.25,301.79,293.44,285.2,277.07,269.04,261.13,253.32,245.63,238.05,230.58,223.22,215.97,208.84,201.82,194.91,188.12,181.45,174.88,168.44,162.11,155.89,149.8,143.82,137.96,132.21,126.58,121.08,115.69,110.42,105.27,100.24,95.326,90.536,85.867,81.319,76.892,72.586,68.403,64.342,60.403,56.586,52.893,49.322,45.875,42.552,39.352,36.276,33.324,30.496,27.793,25.214,22.76,20.432,18.228,16.149,14.196,12.368,10.666,9.0891,7.6383,6.3133,5.1143,4.0413,3.0943,2.2735,1.5789,1.0106,0.56846,0.25266,0.063165,0,};
    //Vertical hanning window
    double hanning_2[512] = {0,3.7797e-05,0.00015118,0.00034013,0.00060463,0.00094463,0.0013601,0.0018509,0.0024171,0.0030584,0.0037749,0.0045665,0.0054329,0.0063741,0.0073899,0.0084803,0.0096449,0.010884,0.012196,0.013583,0.015043,0.016576,0.018182,0.019862,0.021614,0.023438,0.025334,0.027302,0.029341,0.031452,0.033633,0.035885,0.038207,0.040599,0.043061,0.045591,0.04819,0.050858,0.053593,0.056396,0.059266,0.062203,0.065205,0.068274,0.071408,0.074606,0.077869,0.081196,0.084586,0.088038,0.091554,0.09513,0.098769,0.10247,0.10623,0.11004,0.11392,0.11786,0.12185,0.1259,0.13001,0.13417,0.13839,0.14266,0.14699,0.15137,0.1558,0.16029,0.16483,0.16941,0.17405,0.17874,0.18347,0.18826,0.19309,0.19796,0.20288,0.20785,0.21286,0.21792,0.22301,0.22815,0.23333,0.23855,0.24381,0.24911,0.25445,0.25982,0.26523,0.27068,0.27616,0.28167,0.28722,0.2928,0.29841,0.30405,0.30972,0.31542,0.32115,0.32691,0.33269,0.33849,0.34432,0.35018,0.35605,0.36195,0.36787,0.37381,0.37977,0.38574,0.39174,0.39775,0.40377,0.40981,0.41587,0.42193,0.42801,0.4341,0.4402,0.44631,0.45243,0.45855,0.46468,0.47081,0.47695,0.4831,0.48924,0.49539,0.50154,0.50768,0.51383,0.51998,0.52612,0.53225,0.53839,0.54451,0.55063,0.55675,0.56285,0.56894,0.57503,0.5811,0.58716,0.59321,0.59924,0.60526,0.61126,0.61725,0.62321,0.62916,0.63509,0.641,0.64689,0.65275,0.6586,0.66441,0.67021,0.67598,0.68172,0.68743,0.69312,0.69877,0.7044,0.70999,0.71556,0.72109,0.72658,0.73205,0.73748,0.74287,0.74822,0.75354,0.75882,0.76406,0.76926,0.77442,0.77954,0.78462,0.78965,0.79464,0.79958,0.80448,0.80934,0.81414,0.8189,0.82361,0.82827,0.83289,0.83745,0.84196,0.84642,0.85083,0.85518,0.85948,0.86373,0.86792,0.87205,0.87613,0.88015,0.88412,0.88802,0.89187,0.89566,0.89939,0.90306,0.90667,0.91021,0.9137,0.91712,0.92048,0.92377,0.927,0.93017,0.93327,0.9363,0.93927,0.94218,0.94501,0.94778,0.95048,0.95312,0.95568,0.95818,0.96061,0.96296,0.96525,0.96747,0.96961,0.97169,0.97369,0.97562,0.97748,0.97927,0.98099,0.98263,0.9842,0.9857,0.98712,0.98847,0.98975,0.99095,0.99207,0.99313,0.99411,0.99501,0.99584,0.99659,0.99727,0.99788,0.9984,0.99886,0.99923,0.99954,0.99976,0.99991,0.99999,0.99999,0.99991,0.99976,0.99954,0.99923,0.99886,0.9984,0.99788,0.99727,0.99659,0.99584,0.99501,0.99411,0.99313,0.99207,0.99095,0.98975,0.98847,0.98712,0.9857,0.9842,0.98263,0.98099,0.97927,0.97748,0.97562,0.97369,0.97169,0.96961,0.96747,0.96525,0.96296,0.96061,0.95818,0.95568,0.95312,0.95048,0.94778,0.94501,0.94218,0.93927,0.9363,0.93327,0.93017,0.927,0.92377,0.92048,0.91712,0.9137,0.91021,0.90667,0.90306,0.89939,0.89566,0.89187,0.88802,0.88412,0.88015,0.87613,0.87205,0.86792,0.86373,0.85948,0.85518,0.85083,0.84642,0.84196,0.83745,0.83289,0.82827,0.82361,0.8189,0.81414,0.80934,0.80448,0.79958,0.79464,0.78965,0.78462,0.77954,0.77442,0.76926,0.76406,0.75882,0.75354,0.74822,0.74287,0.73748,0.73205,0.72658,0.72109,0.71556,0.70999,0.7044,0.69877,0.69312,0.68743,0.68172,0.67598,0.67021,0.66441,0.6586,0.65275,0.64689,0.641,0.63509,0.62916,0.62321,0.61725,0.61126,0.60526,0.59924,0.59321,0.58716,0.5811,0.57503,0.56894,0.56285,0.55675,0.55063,0.54451,0.53839,0.53225,0.52612,0.51998,0.51383,0.50768,0.50154,0.49539,0.48924,0.4831,0.47695,0.47081,0.46468,0.45855,0.45243,0.44631,0.4402,0.4341,0.42801,0.42193,0.41587,0.40981,0.40377,0.39775,0.39174,0.38574,0.37977,0.37381,0.36787,0.36195,0.35605,0.35018,0.34432,0.33849,0.33269,0.32691,0.32115,0.31542,0.30972,0.30405,0.29841,0.2928,0.28722,0.28167,0.27616,0.27068,0.26523,0.25982,0.25445,0.24911,0.24381,0.23855,0.23333,0.22815,0.22301,0.21792,0.21286,0.20785,0.20288,0.19796,0.19309,0.18826,0.18347,0.17874,0.17405,0.16941,0.16483,0.16029,0.1558,0.15137,0.14699,0.14266,0.13839,0.13417,0.13001,0.1259,0.12185,0.11786,0.11392,0.11004,0.10623,0.10247,0.098769,0.09513,0.091554,0.088038,0.084586,0.081196,0.077869,0.074606,0.071408,0.068274,0.065205,0.062203,0.059266,0.056396,0.053593,0.050858,0.04819,0.045591,0.043061,0.040599,0.038207,0.035885,0.033633,0.031452,0.029341,0.027302,0.025334,0.023438,0.021614,0.019862,0.018182,0.016576,0.015043,0.013583,0.012196,0.010884,0.0096449,0.0084803,0.0073899,0.0063741,0.0054329,0.0045665,0.0037749,0.0030584,0.0024171,0.0018509,0.0013601,0.00094463,0.00060463,0.00034013,0.00015118,3.7797e-05,0,};

    
    //strings for iterating file output names, not needed after ROS implenemtation
    //may be useful if timestamps on frames become a thing.
    std::string file_prefix = "Output/mtx_out_2Dfft_28shifted";
    std::string file_type = ".csv";
    int file_num =0;

    vector<double> valr(1024);//vector for real+cplx pairs
    vector<double> valc(2);//vector for real+cplx pairs

    vector< vector<double> > cols;//vector for columns
    vector< vector<double> > matrix_2d_in_a;//input buffer 1
    vector< vector<double> > matrix_2d_in_b;//input buffer 2
    vector< vector<double> > matrix_2d_src;//radar input sim array

    vector< vector< vector<double> > > matrix_2d_re;//mid calculation 2d array
    vector< vector< vector<double> > > matrix_2d_fi;//final 2d complex array

    struct timespec start, end, tstart, tend,istart, iend, itotal, total;//timing structs

    fftw_plan ltr_trans, ttb_trans;//left to right and top to bottom
    fftw_complex *out, *final, *final2;//FFTW in/out arrays

    ///////////////////////////////////////////////////////////
    // Filestreams
    ///////////////////////////////////////////////////////////

    ofstream mtx_out;
	
    ///////////////////////////////////////////////////////////
    // ROS Functions
    ///////////////////////////////////////////////////////////
	
    //The ros::init() function needs to see argc and argv so that it can perform any ROS arguments and name remapping that were provided at the command line.
    ros::init(argc, argv, TALKER_NAME);
	
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;
	 
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000/*this is the message buffer size in # of messages saved before rejection*/);
    ros::Rate loop_rate(10/*This number is in Hz*/);
	 	 
	 
    ///////////////////////////////////////////////////////////
    // Initializing USB
    ///////////////////////////////////////////////////////////

    usb_init();						// initializing libusb
    usb_find_busses();					// ... finding busses
    usb_find_devices();					// ... and devices
    device = find_device();				// find the device (hopefully the correct one)

    if ( device == NULL ) {				// nothing found
	fprintf(stderr, "Cannot find ucecho device\n");
	cout<<"The program is about to seg fault. Press any key to continue...\n";
	cin.ignore();
    }
    int cfg = usb_set_configuration(handle,1);
    if (usb_claim_interface(handle, 0) < 0) {
	fprintf(stderr, "Error claiming interface 0: %s\n", usb_strerror());
	cout<<"Press any key to continue...\n";
	cin.ignore();
    }

    ///////////////////////////////////////////////////////////
    // Initializing Data Structures
    ///////////////////////////////////////////////////////////

    in = fftw_alloc_real(1024);
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * ((1024/2)+1));
    final = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (512));
    final2 = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (512));

    //init testarr here
    //init buf here

    for(int i=(BUFSIZE/2)+2;i--;){
	testarr[i].n=0;
	}

    for(int i=0;i<512;i++) {
        matrix_2d_in_a.push_back(valr);
        matrix_2d_in_b.push_back(valr);
        matrix_2d_src.push_back(valr);
        cols.clear();
    }

    for(int i=0;i<512;i++) {
        for(int i=0;i<512;i++) {
            cols.push_back(valc);
        }
        matrix_2d_re.push_back(cols);
        cols.clear();
    }

    for(int i=0;i<256;i++) {
        for(int i=0;i<256;i++) {
            cols.push_back(valc);
        }
        matrix_2d_fi.push_back(cols);
        cols.clear();
    }

    ///////////////////////////////////////////////////////////
    // Initializing first IO thread
    ///////////////////////////////////////////////////////////


    //prepare the first input array
    boost::thread IO_Thread(IO_func, boost::ref(matrix_2d_src), boost::ref(matrix_2d_in_a), boost::ref(hanning_1), boost::ref(hanning_2),boost::ref(buf),boost::ref(testarr),boost::ref(handle));
    IO_Thread.join();//this call is supposed to wait for the IO_Thread to finish populating the first array in the double buffer and prepare the pipeline for threaded processing of the radar data

    #if USE_TIMING_ITER
    //The final program will replace this for loop with a while loop that exits on a keystroke command.
    for(int p=0; p < USE_TIMING_ITER; p++) {
        std::cout<<"Trial number "<<p<<"!\n";
        // Start the performance timing here///

        #endif // USE_TIMING_ITER

        clock_gettime(CLOCK_REALTIME, &tstart);

        ///////////////////////////////////////////////////////////
        // FFT Process
        ///////////////////////////////////////////////////////////

        //switches between transforming the A buffer or the B buffer
	//the "id" variable keeps track of the read/write order
        if(id==0){
	    //Thread to prepare the B buffer
	    boost::thread IO_Thread(IO_func, boost::ref(matrix_2d_src), boost::ref(matrix_2d_in_b), boost::ref(hanning_1), boost::ref(hanning_2),boost::ref(buf),boost::ref(testarr),boost::ref(handle));
		
	    //Transforms using the A Buffer as input
	    FFT_func(boost::ref(matrix_2d_in_a), boost::ref(matrix_2d_re), boost::ref(in), boost::ref(out),ltr_trans);
	id=1;
        }

        else{
	    //Thread to prepare the A buffer
	    boost::thread IO_Thread(IO_func, boost::ref(matrix_2d_src), boost::ref(matrix_2d_in_a), boost::ref(hanning_1), boost::ref(hanning_2),boost::ref(buf),boost::ref(testarr),boost::ref(handle));

	    //Transforms using the B Buffer as input
	    FFT_func(boost::ref(matrix_2d_in_b), boost::ref(matrix_2d_re), boost::ref(in), boost::ref(out),ltr_trans);
	id=0;
        }

        // Copy data into the input array, final[][], and perform the second fft
        for(int j=201;j--;){
	    for(int i=512;i--;){
	        final[i][0] = matrix_2d_re[i][j][0];
	        final[i][1] = matrix_2d_re[i][j][1];
	    }
	    fftw_execute(ttb_trans);
	    /*The loop below takes the first and last quarters of  
	     *the output row and places them in the bottom and top
	     *halves of the output array columns repectively.
             */
	    for(int l=511, k=181, n=91; n--;l--, k--){
	        matrix_2d_fi[n][j][0] = final2[l][0];
	        matrix_2d_fi[n][j][1] = final2[l][1];
	        matrix_2d_fi[k][j][0] = final2[n][0];
	        matrix_2d_fi[k][j][1] = final2[n][1];
	    }
        }
        clock_gettime(CLOCK_REALTIME, &tend);
        total = diff(tstart, tend);
        totals+=total.tv_sec;
        totalns+=total.tv_nsec;

        #if USE_TIMING_ITER
        //this is where the timing loop used to end.

	///////////////////////////////////////////////////////////////
	//Prepare and publish the output matrix to ROS
	///////////////////////////////////////////////////////////////
	if(ros::ok()){//here, if the ROS is not ok it will jsut drop the frame
	cout<<"outputting results"<<endl;
	std_msgs::String msg;//declaring the message object
	std::stringstream mtx_stream;//declaring a string stream

	    for(int i=0;i<182;i++){
		for(int j=0; j<201;j++){
		    mtx_stream<<matrix_2d_fi[i][j][0]<<","<<matrix_2d_fi[i][j][1]<<",";
		}
		mtx_stream<<",\n";
	    }

	    msg.data = mtx_stream.str();/*places the stringstream of the output matrix 
					 *into the msg object
					 */

	    //ROS_INFO("%s",msg.data.c_str());//I don't know exactly what this does aside from give the size of the string to ROS_INFO

	    chatter_pub.publish(msg);//This is supposed to publish the message to the subscriber(s)

	    ros::spinOnce();/*This is something that has to do with more complex
			     *iterations and is explained in the documentation
			     *excerpt up north.
			     */
            loop_rate.sleep();/*This works with the ros framerate function to 
			       *ensure this particular line executes no more 
			       *often than x times per second.
			       */
            }

        IO_Thread.join();/*This call is to ensure that the FFT processing thread doesn't 
			  *try to read the buffer arrays or spawn another IO thread before 
			  *the current working one is done. This almost never happens 
			  *because the IO thread isn't spawned until the FFT processing 
			  *is finished. Additionally the IO Thread is almost always faster 
			  *than the FFT processing thread(Note: The FFT processing thread 
			  *has never finished before the IO_Thread under"normal" operating 
			  *conditions.
			  */

    }   
    // end of timing loop

//This is still stuff used for timing on a fixed number of loops.
//the final program will replace teh for loop with a while loop that
//continuously reads FPGA input
    cout<<"Averaged total time w/ timing over " << USE_TIMING_ITER << " trials: "<< (totals)/(USE_TIMING_ITER) <<"sec " << (totalns/(USE_TIMING_ITER))/1000000 <<"msec\n";
    #endif // USE_TIMING_ITER

    ///////////////////////////////////////////////////////////
    // Cleaning up
    ///////////////////////////////////////////////////////////

    usb_release_interface(handle, 0);
    usb_close(handle);

    delete buf;
    delete testarr;

    fftw_destroy_plan(ltr_trans);
    fftw_destroy_plan(ttb_trans);
    fftw_free(in);
    fftw_free(out);
    fftw_free(final);
    fftw_free(final2);

    return(0);
}

