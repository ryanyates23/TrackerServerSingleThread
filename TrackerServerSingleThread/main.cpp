#define __CL_ENABLE_EXCEPTIONS

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "cl.hpp"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>

#define WIDTH 1280
#define HEIGHT 720

#define ROIWIDTH 128
#define ROIHEIGHT 128

#define IDL 0
#define ACQ 1
#define TRK 2

#define SHOWDEBUG 0

#define MINCENTWIDTH 5
#define MINCENTHEIGHT 5



//#define WIDTH 1024
//#define HEIGHT 576

//using namespace std;

struct ROI_t {
    int x;
    int y;
    unsigned char size;
    unsigned char xLSB;
    unsigned char xMSB;
    unsigned char yLSB;
    unsigned char yMSB;
    unsigned char mode;
    unsigned char reqMode;
    unsigned char xTGT;
    unsigned char yTGT;
    unsigned char wTGT;
    unsigned char hTGT;
} ROI;

struct Centroid_t {
    int x;
    int y;
    int width;
    int height;
    int sum;
    int size;
    int present;
};

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void TCPSend(int socket, unsigned char* buffer)
{
    int totalBytesWritten = 0;
    int vecSize = WIDTH*HEIGHT;
    int n;
    while(totalBytesWritten != vecSize)
    {
        n = (int)write(socket,&buffer[totalBytesWritten],vecSize);
        //cout << "Bytes Written: " << n << endl;
        if (n < 0)
            error("ERROR writing to socket");
        totalBytesWritten += n;
    }
}

void TCPReceive(int socket, unsigned char* buffer)
{
    int totalBytesRead = 0;
    int vecSize = WIDTH*HEIGHT;
    bzero(buffer,vecSize);
    int n;
    totalBytesRead = 0;
    while(totalBytesRead != vecSize)
    {
        n = (int)read(socket,&buffer[totalBytesRead],vecSize - totalBytesRead);
        if (n < 0) error("ERROR reading from socket");
        
        totalBytesRead += n;
    }
}

std::vector<Centroid_t> detectCentroids(std::vector<unsigned char> ROI)
{
    std::vector<Centroid_t> centroids;
    Centroid_t currentCentroid = {};
    int centroidGrown = 1;
    int centroidSum = 0;
    int centroidSumLast = 0;
    int dispPix = 0;
    
    int growLeft = 1;
    int growRight = 1;
    int growUp = 1;
    int growDown = 1;
    
    int xBoundLow = 0;
    int xBoundHigh = 0;
    int yBoundLow = 0;
    int yBoundHigh = 0;
    int tmp = 0;
    
    if(SHOWDEBUG)
    {
        for(int y = 0; y < ROIHEIGHT; y++)
        {
            for(int x = 0; x < ROIWIDTH; x++)
            {
                if (ROI[y*ROIWIDTH+x] > 0) dispPix = 1; else dispPix = 0;
                std::cout << dispPix;
            }
            std::cout << std::endl;
        }
    }
    
    for(int y = 0; y < ROIHEIGHT; y++)
    {
        for(int x = 0; x < ROIWIDTH; x++)
        {
            if((y < currentCentroid.y || y > currentCentroid.y + currentCentroid.height) && (x < currentCentroid.x || x > currentCentroid.x + currentCentroid.width))
            {
                //get each positive pixel
                if(ROI[y*ROIWIDTH + x] > 0)
                {
                    centroidSum = 0;
                    centroidSumLast = 0;
                    xBoundLow = x;
                    xBoundHigh = x;
                    yBoundLow = y;
                    yBoundHigh = y;
                    growLeft = 1;
                    growRight = 1;
                    growUp = 1;
                    growDown = 1;
                    centroidGrown = 1;
                    tmp++;
                    while(centroidGrown == 1)
                    {
                        if(xBoundLow > 0 && growLeft == 1) xBoundLow--;
                        if(xBoundHigh < ROIWIDTH && growRight == 1) xBoundHigh++;
                        if(yBoundLow > 0 && growUp == 1) yBoundLow--;
                        if(yBoundHigh < ROIHEIGHT && growDown == 1) yBoundHigh++;
                        growLeft = 0;
                        growRight = 0;
                        growUp = 0;
                        growDown = 0;
                        //tmp = 0;
                        centroidSum = 0;
                        //get surrounding pixels
                        for(int j = yBoundLow; j <= yBoundHigh; j++)
                        {
                            for(int i = xBoundLow; i <= xBoundHigh; i++)
                            {
                                if(ROI[j*ROIWIDTH + i] > 0)
                                {
                                    centroidSum++;
                                    if(i == xBoundLow) growLeft = 1;
                                    if(i == xBoundHigh) growRight = 1;
                                    if(j == yBoundLow) growUp = 1;
                                    if(j == yBoundHigh) growDown = 1;
                                }
                                
                            }
                        }
                        if(centroidSum > centroidSumLast)
                        {
                            centroidGrown = 1;
                        }
                        else centroidGrown = 0;
                        centroidSumLast = centroidSum;
                    }
                    if(centroidSum > 0)
                    {
                        currentCentroid.sum = centroidSum;
                        currentCentroid.width = xBoundHigh - xBoundLow - 1;
                        currentCentroid.height = yBoundHigh - yBoundLow - 1;
                        currentCentroid.x = xBoundLow;// - currentCentroid.width/2;
                        currentCentroid.y = yBoundLow;// - currentCentroid.height/2;
                        currentCentroid.size = currentCentroid.width * currentCentroid.height;
                        currentCentroid.present = 1;
                        
                        centroids.push_back(currentCentroid);
                    }
                    
                }
            }
            
        }
        
    }
    std::cout << "Centroids Processed: " << tmp << std::endl;
    return centroids;
}

Centroid_t getBestCentroid(std::vector<unsigned char> ROI, Centroid_t tgtCentroid, int newTrack)
{
    std::vector<Centroid_t> centroids;
    Centroid_t bestCentroid = {};
    float wTolerance = 0.25;
    float hTolerance = 0.25;
    float wTolHigh = tgtCentroid.width + (float)tgtCentroid.width*wTolerance;
    float wTolLow = tgtCentroid.width - (float)tgtCentroid.width*wTolerance;
    float hTolHigh = tgtCentroid.height + (float)tgtCentroid.height*hTolerance;
    float hTolLow = tgtCentroid.height - (float)tgtCentroid.height*hTolerance;

    
    centroids = detectCentroids(ROI);
    
    if(!centroids.empty())
    {
        if(newTrack || tgtCentroid.size == 0)
        {
            //return biggest centroid
            for(int i = 0; i < centroids.size(); i++)
            {
                if(centroids[i].size > bestCentroid.size)
                {
                    bestCentroid = centroids[i];
                }
            }
        }
        else
        {
            bestCentroid = {};
            for(int i = 0; i < centroids.size(); i++)
            {
                if((centroids[i].width >= wTolLow && centroids[i].width <= wTolHigh) && (centroids[i].height >= hTolLow && centroids[i].height <= hTolHigh))
                {
                    bestCentroid = centroids[i];
                }
            }
            
        }
    }
    
    return bestCentroid;
}

int main(int argc, char *argv[])
{
    //-------------BASIC INIT--------------------//
    //    int argc = 51717;
    //    char *argv = "localhost";
    int x, y;
    
    //int displayFrame = 0;
    unsigned char configByte;
    int TRKmode = IDL;
    int reqTRKmode = IDL;
    int filterType = 2; //0 = none, 1 = 3x3mean, 2 = 3x3 median, 3 = 5x5 median, 4 = 3x3median 2 pass
    int enSobelEdge = 1;
    int enBinarise = 1;
    int enCombine = 0;
    int enCombineFiltering = 0;
    int width = WIDTH;
    int height = HEIGHT;
    int abort = 0;
    
    int sum, max;
    float mean, thresh;
    float threshFactor = 0.70;
    float fps;
    int frame = 0;
    
    Centroid_t tgtCentroid = {};
    Centroid_t trackedCentroid = {};
    int newTrack = 1;
    
    
    
    
    //--------------OPENCV INIT---------------//
    unsigned char receiveBuffer[WIDTH*HEIGHT];
    unsigned char sendBuffer[WIDTH*HEIGHT];
    int vecSize = WIDTH*HEIGHT;
    int ROISize = ROIWIDTH*ROIHEIGHT;
    std::vector<unsigned char> h_frame_in(vecSize);
    std::vector<unsigned char> h_frame_out(vecSize);
    std::vector<unsigned char> h_ROI_data(ROISize);
    
    //--------------OPENCL INIT--------------------//
    cl::Program program;
    cl::Context context;
    
    //This section sets up a variable to accept platform info, then gets it
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    
    //This selects the default platform
    cl::Platform platform = platforms.front();
    printf("Generated Platform\n");
    
    //This sets up a variable for the devices and then gets them
    std::vector<cl::Device> devices;
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    
    //This selects the default device
    cl::Device device = devices.front();
    printf("Established Device\n");
    
    std::cout << device.getInfo<CL_DEVICE_TYPE>();
    std::cout << CL_DEVICE_TYPE_GPU << std::endl;
    std::cout << "CL_Image Support: " << device.getInfo<CL_DEVICE_IMAGE_SUPPORT>() << std::endl;
    std::cout << "Number of Compute Units: " << device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>() << std::endl;
    std::cout << "Global Memory Size (MB): " <<device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>()/1000000 << std::endl;
    std::cout << "Local Memory Size (KB): " << device.getInfo<CL_DEVICE_LOCAL_MEM_SIZE>()/1024 << std::endl;
    std::cout << "Max Clock Frequency (MHz): " <<device.getInfo<CL_DEVICE_MAX_CLOCK_FREQUENCY>() << std::endl;
    
    //This reads in the kernel file
    std::ifstream kernelFile("kernels.cl");
    std::string src(std::istreambuf_iterator<char>(kernelFile), (std::istreambuf_iterator<char>()));
    printf("Read Kernel File\n");
    
    //This turns the string input into a source for kernels
    cl::Program::Sources sources(1, std::make_pair(src.c_str(), src.length() + 1));
    
    //This creates an OpenCL context and a program from the sources
    context = cl::Context(device);
    printf("Created Context\n");
    program = cl::Program(context, sources);
    
    //This tries to build the kernels and outputs a build log if it fails miserably
    try {
        cl::Error err = program.build();
        printf("Built Program\n");
    }
    catch (cl::Error& e)
    {
        if (e.err() == CL_BUILD_PROGRAM_FAILURE)
        {
            for (cl::Device dev : devices)
            {
                // Check the build status
                cl_build_status status = program.getBuildInfo<CL_PROGRAM_BUILD_STATUS>(dev);
                if (status != CL_BUILD_ERROR)
                    continue;
                
                // Get the build log
                std::string name = dev.getInfo<CL_DEVICE_NAME>();
                std::string buildlog = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(dev);
                std::cerr << "Build log for " << name << ":" << std::endl
                << buildlog << std::endl;
            }
        }
        else
        {
            throw e;
        }
        
    }
    
    cl::CommandQueue queue(context, device);
    
    cl::Buffer d_frame_in(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(unsigned char) * h_frame_in.size(), h_frame_in.data());
    cl::Buffer d_original_frame(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, sizeof(unsigned char)*h_frame_in.size(), h_frame_in.data());
    cl::Buffer d_frame_out(context, CL_MEM_READ_WRITE, sizeof(unsigned char)*h_frame_out.size());
    cl::Buffer d_ROI(context, CL_MEM_READ_WRITE, sizeof(unsigned char) * h_ROI_data.size());
    
    cl::Kernel meanKernel(program, "MeanFilter");
    cl::Kernel medianKernel3(program, "MedianFilter3");
    cl::Kernel medianKernel5(program, "MedianFilter5");
    cl::Kernel sobelKernel(program, "SobelEdge");
    cl::Kernel binKernel(program, "Binarise");
    cl::Kernel combineKernel(program, "CombineImages");
    cl::Kernel extractKernel(program, "ExtractROI");
    cl::Kernel insertKernel(program, "InsertROI");
    
    
    //------------TCP SERVER INIT--------------//
    int sockfd, newsockfd, portno;
    socklen_t clilen;
    //char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd,
                       (struct sockaddr *) &cli_addr,
                       &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");
    
    
    
    while(1)
    {
        
        //--------------------TCP RECEIVE----------------//
        TCPReceive(newsockfd, receiveBuffer);
        
        for (y = 0; y < HEIGHT; y++)
        {
            for (x = 0; x < WIDTH-1; x++)
            {
                //std::cout << "Accessing Element: " << (y*WIDTH+x) << std::endl;
                h_frame_in[y*WIDTH + x] = receiveBuffer[y*WIDTH + x];
            }
        }
        //std::cout << "Frame Received" << std::endl;
        
        //----------Get and decode decode bytes-------------//
        configByte = h_frame_in[0];
        filterType = (0x03) & configByte;
        enSobelEdge = ((0x01 << 2) & configByte) >> 2;
        enBinarise = ((0x01 << 3) & configByte) >> 3;
        enCombine = ((0x01 << 4) & configByte) >> 4;
        enCombineFiltering = ((0x01 << 5) & configByte) >> 5;
        abort = ((0x01 << 6) & configByte) >> 6;
        //std::cout << (int)configByte << std::endl;
        
        ROI.size = h_frame_in[1];
        ROI.xMSB = h_frame_in[2];
        ROI.xLSB = h_frame_in[3];
        ROI.yMSB = h_frame_in[4];
        ROI.yLSB = h_frame_in[5];
        //ROI.mode = h_frame_in[6];
        reqTRKmode = h_frame_in[7];
        
        TRKmode = reqTRKmode;
        
        
        //        std::cout << "Mode: " << TRKmode << std::endl;
        
        ROI.x = (int)ROI.xMSB*256 + (int)ROI.xLSB;
        ROI.y = (int)ROI.yMSB*256 + (int)ROI.yLSB;

        
        //----------------PROCESSING----------------------//
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        

        
        
        
        queue.enqueueWriteBuffer(d_frame_in, CL_TRUE, 0, sizeof(unsigned char)*h_frame_in.size(), h_frame_in.data());
        queue.enqueueWriteBuffer(d_original_frame, CL_FALSE, 0, sizeof(unsigned char)*h_frame_in.size(), h_frame_in.data());
        
        if(filterType == 0)
        {
            queue.enqueueCopyBuffer(d_frame_in, d_frame_out, 0, 0, sizeof(unsigned char)*h_frame_in.size());
        }
        else if(filterType == 1)
        {
            {

                meanKernel.setArg(0, d_frame_in);
                meanKernel.setArg(1, d_frame_out);
                meanKernel.setArg(2, width);
                meanKernel.setArg(3, height);

                queue.enqueueNDRangeKernel(meanKernel, cl::NullRange, cl::NDRange(width, height));
            }

        }
        else if(filterType == 2)
        {
            medianKernel3.setArg(0, d_frame_in);
            medianKernel3.setArg(1, d_frame_out);
            medianKernel3.setArg(2, width);
            medianKernel3.setArg(3, height);

            queue.enqueueNDRangeKernel(medianKernel3, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
        }
        else if(filterType == 3)
        {
            medianKernel5.setArg(0, d_frame_in);
            medianKernel5.setArg(1, d_frame_out);
            medianKernel5.setArg(2, width);
            medianKernel5.setArg(3, height);

            queue.enqueueNDRangeKernel(medianKernel5, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
            
        }
        else
        {
            queue.enqueueCopyBuffer(d_frame_in, d_frame_out, 0, 0, sizeof(unsigned char)*h_frame_in.size());
        }
        
        if(enSobelEdge == 1)
        {
            sobelKernel.setArg(0, d_frame_out);
            sobelKernel.setArg(1, d_frame_in);
            sobelKernel.setArg(2, width);
            sobelKernel.setArg(3, height);

            queue.enqueueNDRangeKernel(sobelKernel, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
        }
        else
        {
            queue.enqueueCopyBuffer(d_frame_out, d_frame_in, 0, 0, sizeof(unsigned char)*h_frame_in.size());
        }
            
        if(enBinarise == 1)
        {
            queue.enqueueReadBuffer(d_frame_in, CL_TRUE, 0, sizeof(unsigned char)*h_frame_in.size(), h_frame_in.data());

            sum = 0;
            max = 0;
            for (y = 0; y < height; y++)
            {
                for (x = 0; x < width; x++)
                {
                    sum += h_frame_in[y*width + x];
                    if(h_frame_in[y*width + x] > max) max = h_frame_in[y*width + x];
                }
            }

            mean = sum/(width*height);
            thresh = mean+((max-mean)*threshFactor);


            binKernel.setArg(0, d_frame_in);
            binKernel.setArg(1, d_frame_out);
            binKernel.setArg(2, width);
            binKernel.setArg(3, height);
            binKernel.setArg(4, (unsigned char)thresh);
            
            queue.enqueueNDRangeKernel(binKernel, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
        }
        else
        {
            queue.enqueueCopyBuffer(d_frame_in, d_frame_out, 0, 0, sizeof(unsigned char)*h_frame_in.size());
        }

        if(enCombine == 1)
        {
            combineKernel.setArg(0, d_frame_out);
            combineKernel.setArg(1, d_original_frame);
            combineKernel.setArg(2, d_frame_in);
            combineKernel.setArg(3, width);
            combineKernel.setArg(4, height);
        }
        else
        {
            queue.enqueueCopyBuffer(d_frame_out, d_frame_in, 0, 0, sizeof(unsigned char)*h_frame_in.size());
        }
        
        
        queue.enqueueReadBuffer(d_frame_in, CL_TRUE, 0, sizeof(unsigned char)*h_frame_out.size(), h_frame_out.data());

        
        //-----------EXTRACT THE ROI---------------//
        if(TRKmode == ACQ || TRKmode == TRK)
        {
            //EXTRACT ROI FOR CENTROID DETECTION
            //std::cout << "ROI - x: " << ROI.x << " y: " << ROI.y << " size: " << (int)ROI.size << std::endl;
            extractKernel.setArg(0, d_frame_out);
            extractKernel.setArg(1, d_ROI);
            extractKernel.setArg(2, width);
            extractKernel.setArg(3, height);
            extractKernel.setArg(4, ROI.x);
            extractKernel.setArg(5, ROI.y);
//            extractKernel.setArg(6, ROI.size);
//            extractKernel.setArg(7, ROI.size);
            extractKernel.setArg(6, ROIWIDTH);
            extractKernel.setArg(7, ROIHEIGHT);
            
            queue.enqueueNDRangeKernel(extractKernel, cl::NullRange, cl::NDRange(width, height), cl::NullRange, NULL, NULL);
            queue.finish();
            queue.enqueueReadBuffer(d_ROI, CL_TRUE, 0, sizeof(unsigned char) * h_ROI_data.size(), h_ROI_data.data());
            
            //Centroid_t trackedCentroid = detectCentroids(h_ROI_data);
            trackedCentroid = getBestCentroid(h_ROI_data, tgtCentroid, newTrack);
            if(trackedCentroid.present)
            {
               TRKmode = TRK;
                ROI.xTGT = trackedCentroid.x;// - trackedCentroid.width/2;
                ROI.yTGT = trackedCentroid.y;// - trackedCentroid.height/2;
                ROI.wTGT = trackedCentroid.width;
                ROI.hTGT = trackedCentroid.height;
                ROI.x = (trackedCentroid.x + ROI.x) - ROI.size/2 + ROI.wTGT/2;
                ROI.y = (trackedCentroid.y + ROI.y) - ROI.size/2 + ROI.hTGT/2;
                if(ROI.x > WIDTH-ROI.size) ROI.x = WIDTH-ROI.size;
                if(ROI.x < 0) ROI.x = 0;
                if(ROI.y > HEIGHT - ROI.size) ROI.y = HEIGHT - ROI.size;
                if(ROI.y < 0) ROI.y = 0;
                if(trackedCentroid.width > MINCENTWIDTH && trackedCentroid.height > MINCENTHEIGHT)
                {
                    newTrack = 0;
                    tgtCentroid = trackedCentroid;
                }
                
            }
            
            
            else TRKmode = ACQ;
        }
        
        if(TRKmode == IDL)
        {
            trackedCentroid = {};
            tgtCentroid = {};
            newTrack = 1;
        }
        
        //------------Detect the centroids-----------//
        


        
        
        for (y = 0; y < HEIGHT; y++)
        {
            for (x = 0; x < WIDTH-1; x++)
            {
                sendBuffer[y*WIDTH + x] = h_frame_out[y*WIDTH + x];
            }
        }
        
        ROI.xMSB = (ROI.x & 0xff00) >> 8;
        ROI.xLSB = (ROI.x & 0x00ff);
        ROI.yMSB = (ROI.y & 0xff00) >> 8;
        ROI.yLSB = (ROI.y & 0x00ff);
        
        sendBuffer[1] = ROI.size;
        sendBuffer[2] = ROI.xMSB;
        sendBuffer[3] = ROI.xLSB;
        sendBuffer[4] = ROI.yMSB;
        sendBuffer[5] = ROI.yLSB;
        sendBuffer[6] = TRKmode;
        sendBuffer[7] = ROI.xTGT;
        sendBuffer[8] = ROI.yTGT;
        sendBuffer[9] = ROI.wTGT;
        sendBuffer[10] = ROI.hTGT;
        
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        fps = 1000000/duration;
        
        //std::cout << fps << std::endl;
        
        //----------------TCP Send------------------------//
        TCPSend(newsockfd, sendBuffer);
        
        
        //------------------------------------------------//
        frame++;
    }
    close(newsockfd);
    close(sockfd);
    while(1) x = 0;
    return 0;
}
