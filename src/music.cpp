#include "ros/ros.h"

#define MEASURE = 160

#define HALF int(MEASURE/2)
#define Q int(MEASURE/4)
#define E int(MEASURE/8)
#define Ed int(MEASURE*3/16)
#define S int(MEASURE/16)
 
#define MEASURE_TIME MEASURE/64.

const int c4 = 60;
const int cis4 = 61; 
const int des4 = 61;
const int d4 = 62;
const int dis4 = 63;
const int ees4 = 63;
const int e4 = 64;
const int f4 = 65;
const int fis4 = 66;
const int ges4 = 66;
const int g4 = 67;
const int gis4 = 68;
const int aes4 = 68;
const int a4 = 69;
const int ais4 = 70;
const int bes4 = 70;
const int b4 = 71;
const int c5 = 72;
const int cis5 = 73;
const int des5 = 73;
const int d5 = 74;
const int dis5 = 75;
const int ees5 = 75;
const int e5 = 76;
const int f5 = 77;
const int fis5 = 78;
const int ges5 = 78;
const int g5 = 79;
const int gis5 = 80;
const int aes5 = 80;
const int a5 = 81;
const int ais5 = 82;
const int bes5 = 82;
const int b5 = 83;
const int c6 = 84;
const int cis6 = 85;
const int des6 = 85;
const int d6 = 86;
const int dis6 = 87;
const int ees6 = 87;
const int e6 = 88;
const int f6 = 89;
const int fis6 = 90;
const int ges6 = 90;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "music");
    ros::NodeHandle n;

    store_song = rospy.ServiceProxy('store_song', StoreSong)
    rospy.wait_for_service('play_song')
    play_song = rospy.ServiceProxy('play_song', PlaySong)
    print('Create driver is ready!') 

    

    return 0;
}