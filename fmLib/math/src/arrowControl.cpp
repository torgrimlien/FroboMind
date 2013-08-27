#include <ros/ros.h>
#include <ntnu_fieldflux/motorVal.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class ArrowControl
{
public:
  ArrowControl();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  int linear_, angular_, l_scale_, a_scale_;
  ros::Publisher pub_;
  
};

ArrowControl::ArrowControl():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  pub_ = nh_.advertise<ntnu_fieldflux::motorVal>("throttle_input", 100);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "arrow_control");
  ArrowControl arrow_control;

  signal(SIGINT,quit);

  arrow_control.keyLoop();
  
  return(0);
}


void ArrowControl::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the motors.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
    }
   

    ntnu_fieldflux::motorVal mot;
    mot.leftSpeed = abs(linear_*l_scale_ + angular_*a_scale_);
    mot.rightSpeed = abs(linear_*l_scale_ - angular_*a_scale_);
    if ((linear_*l_scale_ + angular_*a_scale_) > 0 )
	{
	mot.leftDir = 1;
	}
    if ((linear_*l_scale_ + angular_*a_scale_) <= 0 )
	{
	mot.leftDir = 5;
	}
	if ((linear_*l_scale_ - angular_*a_scale_) > 0 )
	{
	mot.rightDir = 1;
	}
    if ((linear_*l_scale_ - angular_*a_scale_) <= 0 )
	{
	mot.rightDir = 5;
	}

    if(dirty ==true)
    {
      pub_.publish(mot);
      dirty=false;
    }
  }


  return;
}


