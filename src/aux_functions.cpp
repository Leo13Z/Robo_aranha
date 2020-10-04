#include"spider.hpp"

// void command_by_serial()
// {
//   String servo_to_move;
//   String angle_desired;
//   while (Serial.available())
//   {
//     char c = Serial.read();  //gets one byte from serial buffer
//     servo_to_move += c; //makes the string readString
//     delay(2);  //slow looping to allow buffer to fill with next character
//   }
//
//   if (servo_to_move.length() >0)
//   {
//     Serial.print("Selected Servo: ");
//     Serial.println(servo_to_move);  //so you can see the captured string
//     Serial.print("Enter the angle:");
//
//     int servo = servo_to_move.toInt();  //convert readString into a number
//
//     while(!Serial.available());
//     while (Serial.available())
//     {
//       char c = Serial.read();  //gets one byte from serial buffer
//       angle_desired += c; //makes the string readString
//       delay(2);  //slow looping to allow buffer to fill with next character
//     }
//     if (angle_desired.length() >0)
//     {
//       Serial.print("Selected Angle: ");
//       Serial.println(angle_desired);  //so you can see the captured string
//       int angle = angle_desired.toInt();  //convert readString into a number
//
//       pwm.setPWM(servo, 0, (map(angle, 0,180,SERVOMIN,SERVOMAX)));
//     }
//   }
//   servo_to_move=""; //empty for next input
//   angle_desired=""; //empty for next input
// }


// void command_by_serial_PolarPosition(struct spiderbot aranha1)
// {
//   String _limb;
//   struct position{
//     String r;
//     String z;
//     String theta;
//   };
//   position desired;
//
//
//   while (Serial.available())
//   {
//     Serial.print("Selected Limb: ");
//     _limb = Serial.readStringUntil('\n');
//   }
//
//   if (_limb.length() >0)
//   {
//     Serial.println(_limb);  //so you can see the captured string
//     Serial.print("Enter r, z and theta :\nr:");
//
//     limb realLimb;
//     switch (_limb.toInt()) {
//       case 0:
//         realLimb.joints[0] = thigh0;
//         realLimb.joints[1] = knee0;
//         realLimb.joints[2] = foot0;
//         break;
//       case 1:
//         realLimb.joints[0] = thigh1;
//         realLimb.joints[1] = knee1;
//         realLimb.joints[2] = foot1;
//         break;
//       case 2:
//         realLimb.joints[0] = thigh2;
//         realLimb.joints[1] = knee2;
//         realLimb.joints[2] = foot2;
//         break;
//       case 3:
//         realLimb.joints[0] = thigh3;
//         realLimb.joints[1] = knee3;
//         realLimb.joints[2] = foot3;
//         break;
//     }
//
//     while(!Serial.available());
//     while (Serial.available())
//     {
//       desired.r = Serial.readStringUntil('\n');
//     }
//     if (desired.r.length() >0)
//     {
//       Serial.println(desired.r);  //so you can see the captured string
//       Serial.print("z:");
//       int r_ = desired.r.toInt();  //convert readString into a number
//
//       while(!Serial.available());
//       while (Serial.available())
//       {
//         desired.z = Serial.readStringUntil('\n');
//       }
//       if (desired.z.length() >0)
//       {
//         Serial.println(desired.z);  //so you can see the captured string
//         Serial.print("theta:");
//         int z_ = desired.z.toInt();  //convert readString into a number
//
//         while(!Serial.available());
//         while (Serial.available())
//         {
//           desired.theta = Serial.readStringUntil('\n');
//         }
//         if (desired.theta.length() >0)
//         {
//           Serial.println(desired.theta);  //so you can see the captured string
//           Serial.print("Moving leg...");
//           int theta_ = desired.theta.toInt();  //convert readString into a number
//           aranha1.moveLimbPolar(realLimb, r_, z_, theta_);
//         }
//       }
//       _limb=""; //empty for next input
//       desired.r=""; //empty for next input
//       desired.z=""; //empty for next input
//       desired.theta=""; //empty for next input
//     }
//   }
// }
