#ifndef DYNPICK_H
#define DYNPICK_H

#include    <iostream>
#include    <fcntl.h>
#include    <time.h>
#include    <termios.h>
#include    <string.h>
#include    <unistd.h>
#include    <vector>

using namespace std;

// TODO: Find out why and fix the multiple try approach
#define RESET_COMMAND_TRY   3       // It only works when send several times.

#define DATA_LENGTH 27
#define CALIB_DATA_LENGTH 45


class DynPick
{
    private:

        int fdc;

        int clock = 0;
        double rate;
        bool auto_adjust = true;
        int frq_div = 1;

        float calib[6] = {1, 1, 1, 1, 1, 1};

        float offsets[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        float force_z, torque_x, torque_y;

        int SetComAttr(int fdc);

        //bool offsetRequest(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        bool clearSocket(const int& fdc, char* leftover);

        bool readCharFromSocket(const int& fdc, const int& length, char* reply);

    public:

        bool set(const char* port);

        void offset(float _FX, float _FY, float _FZ, float _MX, float _MY, float _MZ);

        void auto_offset(int ave_count=100);

        vector<float> read_3axis();

        vector<float> read_axis();
};

#endif
