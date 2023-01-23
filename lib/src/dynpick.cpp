#include "dynpick.hpp"

bool gmoredata = false;
bool gnomoredata = false;
bool gneedmore = false;


int DynPick::SetComAttr(int fdc)
{
    int n;

    struct termios term;


    // Set baud rate
    n = tcgetattr(fdc, &term);
    if (n < 0)
        return 1;

    bzero(&term, sizeof(term));

    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;/*ICANON;*/

    term.c_cc[VINTR]    = 0;     /* Ctrl-c */
    term.c_cc[VQUIT]    = 0;     /* Ctrl-? */
    term.c_cc[VERASE]   = 0;     /* del */
    term.c_cc[VKILL]    = 0;     /* @ */
    term.c_cc[VEOF]     = 4;     /* Ctrl-d */
    term.c_cc[VTIME]    = 0;
    term.c_cc[VMIN]     = 0;
    term.c_cc[VSWTC]    = 0;     /* '?0' */
    term.c_cc[VSTART]   = 0;     /* Ctrl-q */
    term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    term.c_cc[VEOL]     = 0;     /* '?0' */
    term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    term.c_cc[VEOL2]    = 0;     /* '?0' */

    //  tcflush(fdc, TCIFLUSH);
    n = tcsetattr(fdc, TCSANOW, &term);
    
    return 0;
}

/*bool DynPick::offsetRequest(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    std::unique_lock<std::mutex> lock(m_);
    offset_reset_ = RESET_COMMAND_TRY;
    cv_.wait(lock, []{ return offset_reset_ <= 0; });
    lock.unlock();
    res.message = "Reset offset command was send " + std::to_string(RESET_COMMAND_TRY) + " times to the sensor.";
    res.success = true;
    return true;
}*/

bool DynPick::clearSocket(const int& _fdc, char* leftover)
{
  int len = 0;
  int c = 0;
  int length = 255;
  while ( len < length ) {
    c = read(_fdc, leftover+len, length-len);
    if (c > 0) {
      len += c;
      if(!gmoredata){
        printf("= dynpick: More data to clean up; n = %d (%d) ===", c, len);
        gmoredata = true;
      }
    } else {
      if(!gmoredata){      
        cout << "= dynpick: No more data on socket" << endl;
        gnomoredata = true;
      }
      break;
    }
  }
  // This could actually check if data was received and may return on timeout with false
  return true;
}

bool DynPick::readCharFromSocket(const int& _fdc, const int& length, char* reply)
{
  int len = 0;
  int c = 0;
  while ( len < length ) {
    c = read(_fdc, reply+len, length-len);
    if (c >= 0) {
      len += c;
    } else {
      if(!gneedmore){
        printf("=== need to read more data ... n = %d (%d) ===", c, len);
        gneedmore = true;
      }
      continue;
    }
  }
  // This could actually check if data was received and may return on timeout with false
  return true;
}

bool DynPick::DynPick::set(const char* port)
{
  // Open COM port
  fdc = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fdc < 0) {
    cout << "could not open" << port << endl;
    return 1;
  }

  // Set baud rate of COM port
  SetComAttr(fdc);

  // Clean up
  char trash[255];
  clearSocket(fdc, trash);

  // Autoadjust
  if (auto_adjust){
    write(fdc, "p", 1);
    char reply[CALIB_DATA_LENGTH];
    readCharFromSocket(fdc, CALIB_DATA_LENGTH, reply);
    sscanf(reply,"%f,%f,%f,%f,%f,%f", &calib[0], &calib[1], &calib[2], &calib[3], &calib[4], &calib[5]);
    printf("Calibration from sensor:\n%.3f LSB/N, %.3f LSB/N, %.3f LSB/N, %.3f LSB/Nm, %.3f LSB/Nm, %.3f LSB/Nm", calib[0], calib[1], calib[2], calib[3], calib[4], calib[5]);
    clearSocket(fdc, trash);
  }
    
  // Set frequncy divider filter
  if (frq_div == 1 || frq_div == 2 || frq_div == 4 || frq_div == 8) {
    char cmd[2];
    sprintf(cmd, "%d", frq_div);
    // sprintf(cmd, "%F", frq_div);    
    write(fdc, cmd, 2);
    printf("Set the frequency divider to %s", cmd);

    // check if successful
    write(fdc, "0F", 2);
    char repl[3];
    readCharFromSocket(fdc, 3, repl);
    //printf(repl[0]-'0' != frq_div, "Response by sensor is not as expected! Current Filter: %dF", repl[0]-'0');
    clearSocket(fdc, trash);
  } else {
    printf("Not setting frequency divider. Parameter out of acceptable values {1,2,4,8}: %d", frq_div);
  }

  return 0;
}

void DynPick::offset(float _FZ, float _MX, float _MY)
{
    offsets[0] = _FZ;
    offsets[1] = _MX;
    offsets[2] = _MY;
}

void DynPick::auto_offset(int ave_count)
{
    char str[256];
    int tick;
    unsigned short data[6];

    for(int count=0; count<ave_count; count++){
        // Request for initial data (2nd round)
        write(fdc, "R", 1);

        // Obtain single data
        readCharFromSocket(fdc, DATA_LENGTH, str);

        sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

        offsets[0] += -(data[2]-8192)/calib[2];
        offsets[1] += -(data[3]-8192)/calib[3];
        offsets[2] += -(data[4]-8192)/calib[4];
        usleep(10000);
    }
    for(int i=0; i<3; i++) offsets[i] /= ave_count;
}

vector<float> DynPick::read_3axis()
{
    char str[256];
    int tick;
    unsigned short data[6];
    vector<float> wrench(3);
    
    // Request for initial data (2nd round)
    write(fdc, "R", 1);

    // Obtain single data
    readCharFromSocket(fdc, DATA_LENGTH, str);

    sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

    wrench[0] = ((data[2]-8192)/calib[2]) + offsets[0]; // force_x
    wrench[1] = ((data[3]-8192)/calib[3]) + offsets[1]; // torque_x
    wrench[2] = ((data[4]-8192)/calib[4]) + offsets[2]; // torque_y
    
    return wrench;
}
