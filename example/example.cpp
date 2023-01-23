#include <iostream>
#include <sys/time.h>
#include <dynpick.hpp>
#include <fstream>
#include <sstream>

DynPick dynpick;

int main(int argc, char **argv)
{   
    if(argc < 2){
        printf("Error: please argument!\n");
        return 0;
    }


    double s_time;
    double start, time1, time2=0.0, times;
    char port[] = "/dev/ttyUSB0";
    vector<float> wrench(3);


    // setting limit_wrench
    float limit_wrench = 0.0;
    limit_wrench = atof(argv[1]);

    // connecte force-torque sensor
    dynpick.set(port);

    // default process time: 1[s] 
    dynpick.auto_offset();

    
    // time start
    start = static_cast<double>(clock()) / CLOCKS_PER_SEC * 10000.0;
    printf("\nlimit wrench: %f Nm\n", limit_wrench);


    
    // // file output
    //  time_t t = time(0);   // get time now
    //  struct tm * now = localtime( & t );

    //  char buffer [80];
    //  strftime (buffer,80,"/home/CSV/%Y-%m-%d-%H:%M:%S.csv",now);

    // std::ofstream myfile;
    // myfile.open (buffer);
    

    do
    {
	wrench = dynpick.read_3axis(); // [FZ, MX, MY]
	printf("Fz:%.3lf[N] Mx:%.3lf[Nm] My:%.3lf[Nm] time:%.1lf[s]\n", wrench[0], wrench[1], wrench[2], (times-start)/1000);  
    	usleep(1000); // sleep: 100ms
	times = static_cast<double>(clock()) / CLOCKS_PER_SEC * 10000.0;
    }while(wrench[0] > -limit_wrench);

    sleep(2);


    
    // s_time=times+100000;
    
    // time1 = static_cast<double>(clock()) / CLOCKS_PER_SEC * 10000.0;
    // for (int i = 0; i < 1000; i++)  {
	// wrench = dynpick.read_3axis(); // [FZ, MX, MY]
    //     printf("Fz:%.3lf[N] Mx:%.3lf[Nm] My:%.3lf[Nm] time:%.1lf[s]\n", wrench[0], wrench[1], wrench[2], (time2-time1)/1000);
    //     // save to file
    //     myfile << wrench[0] << "," << wrench[1] << "," << wrench[2] << "," << (time2-time1)/1000 << "\n";

    //     usleep(1000); // sleep: 100ms
    //     time2 = static_cast<double>(clock()) / CLOCKS_PER_SEC * 10000.0;
    // }
   
    // myfile.close();
    
    
    return 0;
}

