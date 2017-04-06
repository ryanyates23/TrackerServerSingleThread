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

#define ROIWIDTH 64
#define ROIHEIGHT 64

#define IDL 0
#define ACQ 1
#define TRK 2



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
} ROI;

struct Centroid_t {
    int x;
    int y;
    int width;
    int height;
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
        std::cout << (int)configByte << std::endl;
        
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
        
        if (filterType == 0)
        {
            queue.enqueueWriteBuffer(d_frame_out, CL_TRUE, 0, sizeof(unsigned char)*h_frame_in.size(), h_frame_in.data());
        }
        
        if (filterType == 1)
        {
            
            meanKernel.setArg(0, d_frame_in);
            meanKernel.setArg(1, d_frame_out);
            meanKernel.setArg(2, width);
            meanKernel.setArg(3, height);
            
            queue.enqueueNDRangeKernel(meanKernel, cl::NullRange, cl::NDRange(width, height));
            
            //queue.enqueueReadBuffer(d_frame_out, CL_TRUE, 0, sizeof(unsigned char)*h_frame_out.size(), h_frame_out.data());
            //queue.finish();
            //h_frame_in = h_frame_out;
        }
        else
        {
            //h_frame_out = h_frame_in;
        }
        if (filterType == 2)
        {
            
            medianKernel3.setArg(0, d_frame_in);
            medianKernel3.setArg(1, d_frame_out);
            medianKernel3.setArg(2, width);
            medianKernel3.setArg(3, height);
            
            queue.enqueueNDRangeKernel(medianKernel3, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            
            //queue.enqueueReadBuffer(d_frame_out, CL_TRUE, 0, sizeof(unsigned char)*h_frame_out.size(), h_frame_out.data());
            queue.finish();
            //h_frame_in = h_frame_out;
        }
        else
        {
            //h_frame_out = h_frame_in;
        }
        if(filterType == 3)
        {
            
            medianKernel5.setArg(0, d_frame_in);
            medianKernel5.setArg(1, d_frame_out);
            medianKernel5.setArg(2, width);
            medianKernel5.setArg(3, height);
            
            queue.enqueueNDRangeKernel(medianKernel5, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
            if(enCombine == 1 && enCombineFiltering == 1)
            {
                queue.enqueueCopyBuffer(d_frame_out, d_original_frame, 0, 0, vecSize);
            }
        }
        else
        {
            //h_frame_out = h_frame_in;
        }
        if (enSobelEdge == 1)
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
            h_frame_out = h_frame_in;
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
            //h_frame_out = h_frame_in;
        }
        
        if (enCombine == 1 && enBinarise == 1)
        {
            
            combineKernel.setArg(0, d_frame_out);
            combineKernel.setArg(1, d_original_frame);
            combineKernel.setArg(2, d_frame_in);
            combineKernel.setArg(3, width);
            combineKernel.setArg(4, height);
            
            queue.enqueueNDRangeKernel(combineKernel, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
        }
        else if (enCombine == 1 && enBinarise == 0)
        {
            
            combineKernel.setArg(0, d_frame_in);
            combineKernel.setArg(1, d_original_frame);
            combineKernel.setArg(2, d_frame_out);
            combineKernel.setArg(3, width);
            combineKernel.setArg(4, height);
            
            queue.enqueueNDRangeKernel(combineKernel, cl::NullRange, cl::NDRange(width, height),cl::NullRange, NULL, NULL);
            queue.finish();
        }
        
        if ((enBinarise == 0 && enCombine == 0) || (enBinarise == 1 && enCombine == 1))
        {
            queue.enqueueReadBuffer(d_frame_in, CL_TRUE, 0, sizeof(unsigned char)*h_frame_out.size(), h_frame_out.data());
        }
        else
        {
            queue.enqueueReadBuffer(d_frame_out, CL_TRUE, 0, sizeof(unsigned char)*h_frame_out.size(), h_frame_out.data());
        }
        
        
        if(TRKmode == ACQ || TRKmode == TRK)
        {
            //EXTRACT ROI FOR CENTROID DETECTION
            extractKernel.setArg(0, d_frame_out);
            extractKernel.setArg(1, d_ROI);
            extractKernel.setArg(2, width);
            extractKernel.setArg(3, height);
            extractKernel.setArg(4, ROI.x);
            extractKernel.setArg(5, ROI.y);
            extractKernel.setArg(6, ROI.size);
            extractKernel.setArg(7, ROI.size);
            
            queue.enqueueNDRangeKernel(extractKernel, cl::NullRange, cl::NDRange(width, height), cl::NullRange, NULL, NULL);
            queue.finish();
            //queue.enqueueReadBuffer(d_ROI, CL_TRUE, 0, sizeof(unsigned char) * h_ROI_data.size(), h_ROI_data.data());
            
            insertKernel.setArg(0, d_frame_out);
            insertKernel.setArg(1, d_ROI);
            insertKernel.setArg(2, d_frame_in);
            insertKernel.setArg(3, ROI.x);
            insertKernel.setArg(4, ROI.y);
            insertKernel.setArg(5, ROI.size);
            insertKernel.setArg(6, ROI.size);
            insertKernel.setArg(7, width);
            insertKernel.setArg(8, height);
            
            queue.enqueueNDRangeKernel(insertKernel, cl::NullRange, cl::NDRange(width, height), cl::NullRange, NULL, NULL);
            queue.finish();
            queue.enqueueReadBuffer(d_frame_in, CL_TRUE, 0, sizeof(unsigned char) * h_frame_out.size(), h_frame_out.data());
        }
        
        if(TRKmode == ACQ)
        {
            //Centroid_t centroid = detectCentroids(h_ROI_data);
            //if(centroid.present) TRKmode = TRK;
        }

        
        
        for (y = 0; y < HEIGHT; y++)
        {
            for (x = 0; x < WIDTH-1; x++)
            {
                sendBuffer[y*WIDTH + x] = h_frame_out[y*WIDTH + x];
            }
        }
        
        sendBuffer[1] = ROI.size;
        sendBuffer[2] = ROI.xMSB;
        sendBuffer[3] = ROI.xLSB;
        sendBuffer[4] = ROI.yMSB;
        sendBuffer[5] = ROI.yLSB;
        sendBuffer[6] = TRKmode;
        
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
