/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <stdio.h>
#include <getopt.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <pr2_controller_manager/controller_manager.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <tirt/tirt.h>
#include <nodelet/loader.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
using namespace boost::accumulators;

static const std::string name = "pr2_etherCAT";

static struct
{
  char *program_;
  char *interface_;
  char *xml_;
  bool allow_unprogrammed_;
  bool stats_;
} g_options;

std::string g_robot_desc;
boost::shared_ptr<tirt::ContextManager> g_tirt_manager;

void Usage(string msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
  fprintf(stderr, "  Available options\n");
  fprintf(stderr, "    -i, --interface <interface> Connect to EtherCAT devices on this interface\n");
  fprintf(stderr, "    -x, --xml <file|param>      Load the robot description from this file or parameter name\n");
  fprintf(stderr, "    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices\n");
  fprintf(stderr, "    -h, --help                  Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
  {
    exit(0);
  }
}

static int g_quit = 0;
static bool g_reset_motors = true;
static bool g_halt_motors = false;
static bool g_halt_requested = false;
static volatile bool g_publish_trace_requested = false;
static const int NSEC_PER_SEC = 1e+9;

static struct
{
  accumulator_set<double, stats<tag::max, tag::mean> > ec_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > cm_acc;
  int overruns;
  int recent_overruns;
  int last_overrun;
  int last_severe_overrun;
  double overrun_ec;
  double overrun_cm;
} g_stats;

static void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher)
{
  if (publisher.trylock())
  {
    accumulator_set<double, stats<tag::max, tag::mean> > zero;
    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    diagnostic_updater::DiagnosticStatusWrapper status;

    static double max_ec = 0, max_cm = 0;
    double avg_ec, avg_cm;

    avg_ec = extract_result<tag::mean>(g_stats.ec_acc);
    avg_cm = extract_result<tag::mean>(g_stats.cm_acc);
    max_ec = std::max(max_ec, extract_result<tag::max>(g_stats.ec_acc));
    max_cm = std::max(max_cm, extract_result<tag::max>(g_stats.cm_acc));
    g_stats.ec_acc = zero;
    g_stats.cm_acc = zero;

    static bool first = true;
    if (first)
    {
      first = false;
      status.add("Robot Description", g_robot_desc);
    }

    status.addf("Max EtherCAT roundtrip (us)", "%.2f", max_ec*1e+6);
    status.addf("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec*1e+6);
    status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm*1e+6);
    status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm*1e+6);
    status.addf("Control Loop Overruns", "%d", g_stats.overruns);
    status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
    status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus", g_stats.overrun_ec*1e6, g_stats.overrun_cm*1e6);

    status.name = "Realtime Control Loop";
    if (g_stats.overruns > 0 && g_stats.last_overrun < 30)
    {
      if (g_stats.last_severe_overrun < 30)
	status.level = 1;
      else
	status.level = 0;
      status.message = "Realtime loop used too much time in the last 30 seconds.";
    }
    else
    {
      status.level = 0;
      status.message = "OK";
    }
    g_stats.recent_overruns = 0;
    g_stats.last_overrun++;
    g_stats.last_severe_overrun++;

    statuses.push_back(status);
    publisher.msg_.set_status_vec(statuses);
    publisher.msg_.header.stamp = ros::Time::now();
    publisher.unlockAndPublish();
  }
}

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / NSEC_PER_SEC + n.tv_sec;
}


void *diagnosticLoop(void *args)
{
  EthercatHardware *ec((EthercatHardware *) args);
  struct timespec tick;
  clock_gettime(CLOCK_MONOTONIC, &tick);
  while (!g_quit) {
    ec->collectDiagnostics();
    tick.tv_sec += 1;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);
  }
  return NULL;
}

void tirtLoop()
{
  ros::Rate rate(10);
  int count = 0;
  ros::NodeHandle nh;
  ros::Publisher pub_tirt_state = nh.advertise<tirt_core::TirtState>("tirt_state", 10);

  while (!g_quit)
  {
    g_tirt_manager->updateSchedule();

    if (count % 10 == 0) {
      tirt_core::TirtState::Ptr state = g_tirt_manager->getStateMessage();
      pub_tirt_state.publish(state);
    }

    ++count;
    rate.sleep();
  }
}

static void timespecInc(struct timespec &tick, int nsec)
{
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= NSEC_PER_SEC)
  {
    tick.tv_nsec -= NSEC_PER_SEC;
    tick.tv_sec++;
  }
}

void *controlLoop(void *)
{
  int rv = 0;
  double last_published;
  int period;
  int policy;
  TiXmlElement *root;
  TiXmlElement *root_element;
  boost::thread tirt_thread;
  boost::shared_ptr<nodelet::Loader> nodelet_loader;

  ros::NodeHandle node(name);
  g_tirt_manager = tirt::ContextManager::instance();

  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> publisher(node, "/diagnostics", 2);
  realtime_tools::RealtimePublisher<std_msgs::Float64> *rtpublisher = 0;

  if (g_options.stats_){
    rtpublisher = new realtime_tools::RealtimePublisher<std_msgs::Float64>(node, "realtime", 2);
  }

  // Initialize the hardware interface
  EthercatHardware ec(name);
  ec.init(g_options.interface_, g_options.allow_unprogrammed_);

  // Create controller manager
  pr2_controller_manager::ControllerManager cm(ec.hw_);

  // Load robot description
  TiXmlDocument xml;
  struct stat st;
  if (0 == stat(g_options.xml_, &st))
  {
    xml.LoadFile(g_options.xml_);
  }
  else
  {
    ROS_INFO("Xml file not found, reading from parameter server");
    ros::NodeHandle top_level_node;
    if (top_level_node.getParam(g_options.xml_, g_robot_desc))
      xml.Parse(g_robot_desc.c_str());
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s", g_options.xml_);
      rv = -1;
      goto end;
    }
  }
  root_element = xml.RootElement();
  root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
      ROS_FATAL("Could not parse the xml from %s", g_options.xml_);
      rv = -1;
      goto end;
  }

  // Initialize the controller manager from robot description
  if (!cm.initXml(root))
  {
      ROS_FATAL("Could not initialize the controller manager");
      rv = -1;
      goto end;
  }

  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  //Start Non-realtime diagonostic thread
  static pthread_t diagnosticThread;
  if ((rv = pthread_create(&diagnosticThread, NULL, diagnosticLoop, &ec)) != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    goto end;
  }

  // Starts a nodelet loader
  nodelet_loader.reset(new nodelet::Loader(true));

  // Starts the non-realtime tirt thread
  tirt_thread = boost::thread(tirtLoop);

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);
  period = 1e+6; // 1 ms in nanoseconds

  // Snap to the nearest second
  tick.tv_sec = tick.tv_sec;
  tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
  clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  last_published = now();
  while (!g_quit)
  {
    double start = now();
    if (g_reset_motors)
    {
      ec.update(true, g_halt_motors);
      g_reset_motors = false;
    }
    else
    {
      ec.update(false, g_halt_motors);
    }
    if (g_publish_trace_requested)
    {
      g_publish_trace_requested = false;
      ec.publishTrace(-1,"",0,0);
    }
    g_halt_motors = false;
    double after_ec = now();
    cm.update();
    if (g_tirt_manager)
      g_tirt_manager->update(ros::Time(now()));
    double end = now();

    g_stats.ec_acc(after_ec - start);
    g_stats.cm_acc(end - after_ec);

    if ((end - last_published) > 1.0)
    {
      publishDiagnostics(publisher);
      last_published = end;
    }

    // Compute end of next period
    timespecInc(tick, period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + before.tv_nsec/1e9) > (tick.tv_sec + tick.tv_nsec/1e9))
    {
      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(tick, period);

      // initialize overruns
      if (g_stats.overruns == 0){
	g_stats.last_overrun = 1000;
	g_stats.last_severe_overrun = 1000;
      }
      // check for overruns
      if (g_stats.recent_overruns > 10)
	g_stats.last_severe_overrun = 0;
      g_stats.last_overrun = 0;

      g_stats.overruns++;
      g_stats.recent_overruns++;
      g_stats.overrun_ec = after_ec - start;
      g_stats.overrun_cm = end - after_ec;
    }

    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

    // Publish realtime loops statistics, if requested
    if (rtpublisher)
    {
      struct timespec after;
      clock_gettime(CLOCK_REALTIME, &after);
      double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/1e9)*1e6;
      if (rtpublisher->trylock())
      {
        rtpublisher->msg_.data  = jitter;
        rtpublisher->unlockAndPublish();
      }
    }

    // Halt the motors, if requested by a service call
    if (g_halt_requested)
    {
      g_halt_motors = true;
      g_halt_requested = false;
    }
  }

  /* Shutdown all of the motors on exit */
  for (pr2_hardware_interface::ActuatorMap::const_iterator it = ec.hw_->actuators_.begin(); it != ec.hw_->actuators_.end(); ++it)
  {
    it->second->command_.enable_ = false;
    it->second->command_.effort_ = 0;
  }
  ec.update(false, true);

  //pthread_join(diagnosticThread, 0);

end:
  publisher.stop();
  delete rtpublisher;

  ros::shutdown();

  return (void *)rv;
}

void quitRequested(int sig)
{
  g_quit = 1;
}

bool resetMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_reset_motors = true;
  return true;
}

bool haltMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_halt_requested = true;
  return true;
}

bool publishTraceService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_publish_trace_requested = true;
  return true;
}

static int
lock_fd(int fd)
{
  struct flock lock;
  int rv;

  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;

  rv = fcntl(fd, F_SETLK, &lock);
  return rv;
}

#define PIDDIR "/var/tmp/run/"
#define PIDFILE "pr2_etherCAT.pid"
static int setupPidFile(void)
{
  int rv = -1;
  pid_t pid;
  int fd;
  FILE *fp = NULL;

  umask(0);
  mkdir(PIDDIR, 0777);
  fd = open(PIDDIR PIDFILE, O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);
  if (fd == -1)
  {
    if (errno != EEXIST)
    {
      ROS_FATAL("Unable to create pid file '%s': %s", PIDDIR PIDFILE, strerror(errno));
      goto end;
    }

    if ((fd = open(PIDDIR PIDFILE, O_RDWR)) < 0)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", PIDDIR PIDFILE, strerror(errno));
      goto end;
    }

    if ((fp = fdopen(fd, "rw")) == NULL)
    {
      ROS_FATAL("Can't read from '%s': %s", PIDDIR PIDFILE, strerror(errno));
      goto end;
    }
    pid = -1;
    if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
    {
      int rc;

      if ((rc = unlink(PIDDIR PIDFILE)) == -1)
      {
        ROS_FATAL("Can't remove stale pid file '%s': %s", PIDDIR PIDFILE, strerror(errno));
        goto end;
      }
    } else {
      ROS_FATAL("Another instance of pr2_etherCAT is already running with pid: %d", pid);
      goto end;
    }
  }

  unlink(PIDDIR PIDFILE);
  fd = open(PIDDIR PIDFILE, O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP | S_IWOTH | S_IROTH);

  if (fd == -1)
  {
    ROS_FATAL("Unable to open pid file '%s': %s", PIDDIR PIDFILE, strerror(errno));
    goto end;
  }

  if (lock_fd(fd) == -1)
  {
    ROS_FATAL("Unable to lock pid file '%s': %s", PIDDIR PIDFILE, strerror(errno));
    goto end;
  }

  if ((fp = fdopen(fd, "w")) == NULL)
  {
    ROS_FATAL("fdopen failed: %s", strerror(errno));
    goto end;
  }

  fprintf(fp, "%d\n", getpid());

  /* We do NOT close fd, since we want to keep the lock. */
  fflush(fp);
  fcntl(fd, F_SETFD, (long) 1);
  rv = 0;
end:
  return rv;
}

static void cleanupPidFile(void)
{
  unlink(PIDDIR PIDFILE);
}

#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;
int main(int argc, char *argv[])
{
  // Keep the kernel from swapping us out
  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
    perror("mlockall");
    return -1;
  }

  // Setup single instance
  if (setupPidFile() < 0) return -1;

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv, "realtime_loop");

  // Parse options
  g_options.program_ = argv[0];
  while (1)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"allow_unprogrammed", no_argument, 0, 'u'},
      {"interface", required_argument, 0, 'i'},
      {"xml", required_argument, 0, 'x'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "hi:usx:", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'u':
        g_options.allow_unprogrammed_ = 1;
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'x':
        g_options.xml_ = optarg;
        break;
      case 's':
        g_options.stats_ = 1;
        break;
    }
  }
  if (optind < argc)
  {
    Usage("Extra arguments");
  }

  if (!g_options.interface_)
    Usage("You must specify a network interface");
  if (!g_options.xml_)
    Usage("You must specify a robot description XML file");

  ros::NodeHandle node(name);

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  ros::ServiceServer reset = node.advertiseService("reset_motors", resetMotorsService);
  ros::ServiceServer halt = node.advertiseService("halt_motors", haltMotorsService);
  ros::ServiceServer publishTrace = node.advertiseService("publish_trace", publishTraceService);

  //Start thread
  int rv;
  if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d", rv);
    ROS_BREAK();
  }

  ros::spin();
  pthread_join(controlThread, (void **)&rv);

  // Cleanup pid file
  cleanupPidFile();

  return rv;
}
