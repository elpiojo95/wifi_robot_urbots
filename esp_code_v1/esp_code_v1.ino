#include <Kinematics.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define MOTOR_MAX_RPM 8200        // motor's maximum rpm
#define WHEEL_DIAMETER 0.0489      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.0753   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define POLOLU_MAX 50 // max value for pololu driver


Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);
/*
  char ssid[] = "URBots_2G";
  char password[] = "URB@ts_interna";
*/
d[] = "elpiojo-u";
char password[] = "Rumble_15";

unsigned char bytes[4];

//IPAddress server(192, 168, 1, 25);
IPAddress server(192, 168, 1, 1);
const uint16_t serverPort = 11411;

void motorTwist( const geometry_msgs::Twist& msg) {
  Kinematics::output rpm;

  int16_t data[2];
  float linear_vel_x = msg.linear.x - msg.linear.y;     //  m/s
  float linear_vel_y = 0;               //   m/s
  float angular_vel_z = msg.angular.z - msg.angular.x; //    m/s
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);

  //LEFT MOTOR
  data[0] = rpm.motor1 * POLOLU_MAX / MOTOR_MAX_RPM;
  //Serial.println(data[0]);
  //RIGHT MOTOR
  data[1] = rpm.motor2 * POLOLU_MAX / MOTOR_MAX_RPM;
  //Serial.println(data[1]);

  bytes[0] = data[0] & 0x00FF;
  bytes[1] = data[0] >> 8;
  bytes[2] = data[1] & 0x00FF;
  bytes[3] = data[1] >> 8;
  Serial.write(bytes, 4);
/*
  Serial.print("rpm.motor1: ");
  Serial.println(rpm.motor1);

  Serial.print("data ");
  Serial.println(data[0]);
*/
}

ros::NodeHandle nh;
// Make a chatter publisher
//std_msgs::Int16 motor1_msg;
ros::Subscriber<geometry_msgs::Twist> sub1("r2/cmd_vel", motorTwist);



void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  //Serial.println("Wifi test!");

  WiFi.disconnect();

  WiFi.setOutputPower(0);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub1);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

}

void loop() {
  nh.spinOnce();
  delay(1);
}
