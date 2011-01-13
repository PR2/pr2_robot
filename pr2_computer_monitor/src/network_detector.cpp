#include <netinet/ip.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <errno.h>

#include <string>

#include "ros/ros.h"
#include "std_msgs/Bool.h"

static int socket_fd = -1;

bool initSocket()
{
  socket_fd = socket( AF_INET, SOCK_DGRAM, 0 );
  if( socket_fd != -1 )
    return true;
  else
    return false;
}

bool interfaceIsRunning( std::string interface_name )
{
  struct ifreq ifr;
  
  strcpy( ifr.ifr_name, interface_name.c_str() );
  if( ioctl( socket_fd, SIOCGIFFLAGS, &ifr ) < 0 )
  {
    static std::string last_warning;
    std::string warning = "Query of interface '" + interface_name + "' failed: '" + strerror( errno ) + "'  Presuming down.";
    if( warning != last_warning )
    {
      ROS_WARN( warning.c_str() );
    }
    last_warning = warning;
    return false;
  }
  bool running = (ifr.ifr_flags & IFF_RUNNING);
  bool up = (ifr.ifr_flags & IFF_UP);

  return up && running;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "network_detector" );
  ros::NodeHandle node;
  std::string interface_name;
  if( !ros::param::get( "~interface_name", interface_name ))
  {
    ROS_FATAL( "No parameter 'interface_name' specified.  Don't know which interface to watch.  Exiting." );
    exit(1);
  }
  ros::Publisher running_pub = node.advertise<std_msgs::Bool>( "network/connected", 0, true );
  int loop_count;
  bool first_time = true;
  bool was_running = false;
  float inner_loop_hertz = 4;
  ros::Rate loop_rate( inner_loop_hertz );
  if( !initSocket() )
  {
    ROS_FATAL( "Failed to open socket for network ioctl: '%s'.  Exiting.",
	       strerror( errno ));
    exit(1);
  }
  while( ros::ok() )
  {
    bool is_running = interfaceIsRunning( interface_name );
    if( is_running != was_running || first_time || loop_count > inner_loop_hertz * 5 )
    {
      if( is_running != was_running )
      {
	ROS_INFO( "Interface '%s' %s.", interface_name.c_str(), (is_running ? "connected" : "disconnected") );
      }

      std_msgs::Bool msg;
      msg.data = is_running;
      running_pub.publish( msg );

      loop_count = 0;
      first_time = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
    loop_count++;
    was_running = is_running;
  }
}
