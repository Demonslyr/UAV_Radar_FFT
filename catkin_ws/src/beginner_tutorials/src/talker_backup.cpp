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
	unsigned char o[4];//change this to an unsigned short. I'll use half as much memmory to hold the FPGA values
	int n;
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
		for(int j=0;j<2;j++){
			testarr[i].o[j]=buf[(2*i)+j];
		}
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

    double hanning_1 [801];//Horizontal hanning window
    double hanning_2 [512];//Vertical hanning window
    
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

    ifstream hann_1;
    ifstream hann_2;
    ifstream Matrix_in;
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
    // Opening filestreams
    ///////////////////////////////////////////////////////////

    hann_1.open("../../../../../../../../../home/dan/radar-fft/fourier_2D/hann1_801.csv");
    hann_2.open("../../../../../../../../../home/dan/radar-fft/fourier_2D/hann2_512.csv");

    ///////////////////////////////////////////////////////////
    // FFTW planning
    ///////////////////////////////////////////////////////////

    ltr_trans = fftw_plan_dft_r2c_1d(1024, in, out, FFTW_EXHAUSTIVE);
    ttb_trans = fftw_plan_dft_1d(512, final, final2, FFTW_FORWARD, FFTW_EXHAUSTIVE);

    // Check to see if all three files opened correctly
    if (hann_1.is_open()&&hann_2.is_open()/*&&Matrix_in.is_open()*/) {

        // Read in hanning vector 1 into the hanning_1 array

        for (i = 0; i < 801; ++i) {
            assert(hann_1.good());
            hann_1.getline(bkt,14,'\n');
            hanning_1[i]= atof(bkt);
        }

        // Read in hanning vector 2 into the hanning_2 array
        for (h = 0; h < 512; h++) {
            assert(hann_2.good());
            hann_2.getline(bkt,14,'\n');
            hanning_2[h]= atof(bkt);
        }

    } else {
        cout<<"Error opening the hanning vector files.";
        exit(1);
    }

    ///////////////////////////////////////////////////////////
    // Closing unnecessary filestreams
    ///////////////////////////////////////////////////////////
    
    hann_1.close();
    hann_2.close();
    Matrix_in.close();

    ///////////////////////////////////////////////////////////
    // Initializing first IO thread
    ///////////////////////////////////////////////////////////


    //prepare the first input array
    boost::thread IO_Thread(IO_func, boost::ref(matrix_2d_src), boost::ref(matrix_2d_in_a), boost::ref(hanning_1), boost::ref(hanning_2),boost::ref(buf),boost::ref(testarr),boost::ref(handle));
    IO_Thread.join();//this call is supposed to wait for the IO_Thread to finish populating the first array in the double buffer and prepare the pipeline for threaded processing of the radar data

    #if USE_TIMING_ITER
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
    cout<<"outputting results"<<endl;
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

