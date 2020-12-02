#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

int key_num = 0;

int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "key_talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int64>("get_key", 100);

  ros::Rate loop_rate(100);


  int count = 0;
  while (ros::ok())
  {
    // Get the pressed key
    key_num = getch();
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Int64 msg;

    msg.data = key_num;

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);
    if (key_num == 3)
    {
        break;
    }
    ROS_INFO("%i", msg.data);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;


  }


  return 0;
}