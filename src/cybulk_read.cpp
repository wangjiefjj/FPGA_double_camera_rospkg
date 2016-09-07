//
// Created by topplusvision on 16-9-7.
//

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "../include/cyusb.h"
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static cyusb_handle *h1 = NULL;
unsigned short int gl_ep_in, gl_ep_out;
bool readGo= true;

void bulk_reader(int num_bytes)
{
    int rStatus, nbr;
    unsigned char *buf;
    int transferred = 0;

    buf = (unsigned char*)(malloc(num_bytes * 2));
    if (buf == NULL){
        printf ("Error in Memory allocation ... Try a smaller size\n");
        return;
    }

    rStatus = cyusb_bulk_transfer(h1, gl_ep_in, buf, num_bytes, &transferred, 2000);
    if (rStatus == LIBUSB_ERROR_TIMEOUT){
        printf ("\nRead only %d bytes \n", transferred);
        printf ("\nError: Timed out while reading %d bytes ..\n\n", (num_bytes - transferred));
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return;
    }
    else if (rStatus == LIBUSB_ERROR_OVERFLOW){
        printf ("\nBuffer overflow occurred read only %d bytes \n\n", transferred);
        printf ("Clearing ep %d \n",gl_ep_in);
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return;
    }
    else if (rStatus != LIBUSB_SUCCESS){
        printf ("\nError in doing bulk read ..read %d bytes only %d \n", transferred, rStatus);
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return;
    }

    //printf ("\nSuccessfully read %d bytes \n\n", transferred);
    free (buf);
}

unsigned char* bulk_read_buff(int num_bytes,int* my_transferred)
{
    int rStatus, nbr;
    unsigned char *buf;
    int transferred = 0;

    buf = (unsigned char*)(malloc(num_bytes * 2));
    if (buf == NULL){
        printf ("Error in Memory allocation ... Try a smaller size\n");
        return NULL;
    }

    rStatus = cyusb_bulk_transfer(h1, gl_ep_in, buf, num_bytes, &transferred, 2000);
    if (rStatus == LIBUSB_ERROR_TIMEOUT){
        printf ("\nRead only %d bytes \n", transferred);
        printf ("\nError: Timed out while reading %d bytes ..\n\n", (num_bytes - transferred));
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return NULL;
    }
    else if (rStatus == LIBUSB_ERROR_OVERFLOW){
        printf ("\nBuffer overflow occurred read only %d bytes \n\n", transferred);
        printf ("Clearing ep %d \n",gl_ep_in);
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return NULL;
    }
    else if (rStatus != LIBUSB_SUCCESS){
        printf ("\nError in doing bulk read ..read %d bytes only %d \n", transferred, rStatus);
        cyusb_clear_halt (h1, gl_ep_in);
        free (buf);
        return NULL;
    }

    //printf ("\nSuccessfully read %d bytes \n\n", transferred);
    *my_transferred=transferred;
    return buf;
}


void getImage(image_transport::Publisher image_pub_right,image_transport::Publisher image_pub_left,cv_bridge::CvImage cvi){
    printf("begin getImage!");
    long nBufSize= 640*480*2 + 2048;
    long nBulkLen=nBufSize;
    int transferred=0;
    //cvi.image = image;

    sensor_msgs::Image im;
    readGo=true;
    while(ros::ok()){
        unsigned char* data;
        data=bulk_read_buff(nBulkLen,&transferred);

        if(transferred<640*480*2){
            printf("\nerror：　read data less than that wanted!");
            break;
        }else{
            int HeaderOffset=0;
            while (!((*(data + HeaderOffset)==0xfe) & (*(data+1 + HeaderOffset)==0x01) & (*(data+2 + HeaderOffset)==0xfe) & (*(data+3 + HeaderOffset)==0x01)))	//帧头
            {
                HeaderOffset++;
            }
            unsigned char* pcS = (data + HeaderOffset + 16);
            cv::Mat img1,img2;
            img1.create(480,640,CV_8UC1);
            img2.create(480,640,CV_8UC1);
            int r=0,l1=0,l2=0;
            for(int cnt_y=0; cnt_y<480; cnt_y++)
            {
                //printf("\n");
                l1=0;
                l2=0;
                for(int cnt_x=0; cnt_x<1280; cnt_x++)
                {
                    if(cnt_x<640)
                    {
                        // cmos0
                        //*(pcD+(cnt_y*1280+cnt_x)*3)  = *(pcS+cnt_y*1280+cnt_x*2); // Blue
                        //*(pcD+(cnt_y*1280+cnt_x)*3+1)= *(pcS+cnt_y*1280+cnt_x*2); // Green
                        //*(pcD+(cnt_y*1280+cnt_x)*3+2)= *(pcS+cnt_y*1280+cnt_x*2); // Red
                        unsigned char temp;
                        temp=*(pcS+cnt_y*1280+cnt_x*2);
                        //printf("%d--",(int)temp);
                        img1.at<uchar>(r,l1)=temp;
                        l1++;
                    }
                    else
                    {
                        // cmos1
                        //*(pcD+(cnt_y*1280+cnt_x)*3)  = *(pcS+cnt_y*1280+cnt_x*2+1); // Blue
                        //*(pcD+(cnt_y*1280+cnt_x)*3+1)= *(pcS+cnt_y*1280+cnt_x*2+1); // Green
                        //*(pcD+(cnt_y*1280+cnt_x)*3+2)= *(pcS+cnt_y*1280+cnt_x*2+1); // Red
                        unsigned char temp;
                        temp=*(pcS+cnt_y*1280+cnt_x*2+1);
                        //printf("%d~~",(int)temp);
                        img2.at<uchar>(r,l2)=temp;
                        l2++;
                    }

                }
                r++;
            }

            ros::Time time=ros::Time::now();
            cvi.header.stamp = time;
            cvi.image=img1;
            cvi.toImageMsg(im);
            image_pub_right.publish(im);
            cvi.image=img2;
            cvi.toImageMsg(im);
            image_pub_left.publish(im);
            //cv::imshow("img1",img1);
            //cv::waitKey(20);
            //cv::imshow("img2",img1);
            //cv::waitKey(20);
        }
        free(data);
    }

}


int main(int argc, char **argv)
{
    int rStatus;
    ros::init(argc,argv,"fpga_double_camera");
    libusb_config_descriptor *configDesc;
    const struct libusb_endpoint_descriptor *endpoint;
    const struct libusb_interface *interface0 ;
    int num_bytes=0, maxPacketSize, numEndpoints, ep_index_out = 0, ep_index_in = 0;
    unsigned short int temp, ep_in[8], ep_out[8];

    /*if (argc != 2){
        printf ("Usage is ./cybulk_reader <num of bytes> \n Ex ./cybulk_reader 1024 \n");
        return -1;
    }*/

    rStatus = cyusb_open();
    if ( rStatus < 0 ) {
        printf("Error opening library\n");
        cyusb_close();
        return -1;
    }
    else if ( rStatus == 0 ) {
        printf("No device found\n");
        cyusb_close();
        return 0;
    }

    //num_bytes = atoi (argv[1]);
    printf ("Number of bytes is %d ", num_bytes);
    h1 = cyusb_gethandle(0);
    rStatus = cyusb_kernel_driver_active(h1, 0);
    if ( rStatus != 0 ) {
        printf("kernel driver active. Exitting\n");
        cyusb_close();
        return 0;
    }
    rStatus = cyusb_claim_interface(h1, 0);
    if ( rStatus != 0 ) {
        printf("Error in claiming interface\n");
        cyusb_close();
        return 0;
    }
    rStatus = cyusb_get_config_descriptor (h1, 0, &configDesc);
    if (rStatus != 0){
        printf ("Could not get Config descriptor \n");
        cyusb_close();
        return -1;
    }

    //Finding the endpoint address
    interface0 = configDesc->interface;
    numEndpoints = interface0->altsetting->bNumEndpoints;
    endpoint = interface0->altsetting->endpoint;

    while (numEndpoints){
        if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN){
            ep_in [ep_index_in] = endpoint->bEndpointAddress;
            ep_index_in++;
        }
        else{
            ep_out [ep_index_out] = endpoint->bEndpointAddress;
            ep_index_out++;
        }
        numEndpoints --;
        endpoint = ((endpoint) + 1);
    }
    //Choosing 1 endpoint for read and one for write
    if (ep_in[0] == 0){
        printf ("No IN endpoint in the device ... Cannot do bulk write\n");
        cyusb_free_config_descriptor (configDesc);
        cyusb_close();
        return -1;
    }

    gl_ep_in = ep_in [0];
    gl_ep_out = ep_out [0];

    //printf("\nbegin ros se\n");

    //printf ("The Endpoint address is  0x%x \n", gl_ep_in);
    //printf("begin ros setup\n");
    maxPacketSize = cyusb_get_max_packet_size (h1, gl_ep_in);
    //printf ("Number of bytes to read should be multiple of %d--The EP max packetsize \n", maxPacketSize);
    //printf("begin ros setup!\n");
    ros::NodeHandle node;
    //printf("begin ros setup2222!\n");
    image_transport::ImageTransport transport(node);
    //printf("1");
    image_transport::Publisher image_pub_right,image_pub_left;
    //printf("2");
    image_pub_right=transport.advertise("fpga_camera_right", 1);
    image_pub_left=transport.advertise("fpga_camera_left", 1);
    //printf("begin cv_bridge!");
    cv_bridge::CvImage cvi;
    //printf("3");
    cvi.header.frame_id = "image";
    //printf("4");
    cvi.encoding = "mono8";

    getImage(image_pub_right,image_pub_left,cvi);

    ros::spin();
    while(!ros::ok()){
        printf("safe finished!");
        readGo=false;
        cyusb_free_config_descriptor (configDesc);
        cyusb_close();
        cv::destroyAllWindows();
        break;
    }

    return 0;
}
