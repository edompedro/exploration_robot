#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// velocidades iniciais
float speed = 0.2; // linear  (m/s)
float turn = 0.25; // angular (rad/s)

float x = 0, y = 0, z = 0, th = 0; 
char key = ' ';

std::map<char, std::vector<float>> directionMap{
  {'w', {0.2, 0, 0, 0}},
  {'a', {0, 0, 0, 0.2}},
  {'s', {-0.2, 0, 0, 0}},
  {'d', {0, 0, 0, -0.2}},
  {'z', {0, 0, 0, 0}}
};

std::map<char, std::vector<float>> velocityMap{
  {'q', {1.1, 1.1}},
  {'e', {0.9, 0.9}},
};

// Reminder message
const char* msg = R"(
---------------------------
Moving around:
    w    
a       d
    s    

q/e : increase/decrease max speeds by 10%
z : stop
CTRL-C to quit
)";

// For non-blocking keyboard inputs
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

int main(int argc, char** argv){
    ros::init(argc, argv, "teleop_twist_keyboard");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist twist;

    printf("%s", msg);
    printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r\n", speed, turn);

    while(true){

        key = getch();

        if (directionMap.count(key) > 0){
            x = directionMap[key][0];
            y = directionMap[key][1];
            z = directionMap[key][2];
            th = directionMap[key][3];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }
        else if (velocityMap.count(key) > 0){
            speed = speed * velocityMap[key][0];
            turn = turn * velocityMap[key][1];

            printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
        }
        else{
            x = 0;
            y = 0;
            z = 0;
            th = 0;
            
            //correspondecia do ctrl c
            if (key == '\x03'){ break; }

            printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
        }

        // Update the Twist message
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        pub.publish(twist);
        ros::spinOnce();
    }

  return 0;
}