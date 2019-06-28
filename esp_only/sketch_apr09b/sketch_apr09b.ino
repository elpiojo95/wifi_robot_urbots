#include <Kinematics.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define MOTOR_MAX_RPM 8200        // motor's maximum rpm
#define WHEEL_DIAMETER 0.0489     // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0       // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.0753  // distance between left wheel and right wheel
#define PWM_BITS 8                // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define POLOLU_MAX 400            // max value for pololu driver

#define DIRA_A 0
#define PWMA_A 5
#define DIRA_B 2
#define PWMA_B 4

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

char ssid[] = "URBots_2G";
char password[] = "URB@ts_interna";

/*
char ssid[] = "DISCOVERY";
char password[] = "26bahia1507";
*/
unsigned char bytes[4];

IPAddress server(192,168,0,25);
const uint16_t serverPort = 11411;

void motorTwist( const geometry_msgs::Twist& msg){
  Kinematics::output pwm;

  float linear_vel_x = msg.linear.x - msg.linear.y;     //  m/s
  float linear_vel_y = 0;               //   m/s
  float angular_vel_z = msg.angular.z; //    m/s
  pwm = kinematics.getPWM(linear_vel_x, linear_vel_y, angular_vel_z);

  //LEFT MOTOR
  digitalWrite(DIRA_A, (pwm.motor1 & 2147483648) != 0);
  analogWrite(PWMA_A, abs(pwm.motor1));
  
  //RIGHT MOTOR
  digitalWrite(DIRA_B, (pwm.motor2 & 2147483648) != 0);
  analogWrite(PWMA_B, abs(pwm.motor2));
  
  
}

ros::NodeHandle nh;
// Make a chatter publisher
//std_msgs::Int16 motor1_msg;
ros::Subscriber<geometry_msgs::Twist> sub1("r2/cmd_vel", motorTwist);



void setup() {
  pinMode(DIRA_A, OUTPUT);
  pinMode(PWMA_A, OUTPUT);
  pinMode(DIRA_B, OUTPUT);
  pinMode(PWMA_B, OUTPUT);
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
