package alex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "frontLeftMotor";
            public static DcMotor.Direction frontLeftMotorDirection = DcMotor.Direction.FORWARD;
            public static String frontRightMotorName = "frontRightMotor";
            public static DcMotor.Direction frontRightMotorDirection = DcMotor.Direction.REVERSE;
            public static String backLeftMotorName = "backLeftMotor";
            public static DcMotor.Direction backLeftMotorDirection = DcMotor.Direction.FORWARD;
            public static String backRightMotorName = "backRightMotor";
            public static DcMotor.Direction backRightMotorDirection = DcMotor.Direction.REVERSE;
            public static String winchMotorName = "winchMotor";
            public static DcMotor.Direction winchMotorDirection = DcMotor.Direction.REVERSE;
            public static String armMotorName = "armMotor";
            public static DcMotor.Direction armMotorDirection = DcMotor.Direction.REVERSE;
            public static String slideMotorName = "slideMotor";
            public static DcMotor.Direction slideMotorDirection = DcMotor.Direction.FORWARD;

            public static double driveMotorPPR =  ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double strafeMultiplier = 100d/89d;

        }

        public static class Servo {
            public static String clawServoName = "clawServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
            public static double clawOpenPostion = 0.1;
            public static double clawClosedPosition = 0.325;
            public static String wristServoName = "wristServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawTiltServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double clawTiltServoLow = .47;
            public static double clawTiltServoHigh = .3;
            static int clawAutoTiltHeight = (int) (1900/(100/17.5));

            public static String calibrateServoName = "calibrate_servo";

        }

        public static class Wheel {
            public static double wheelRadius = 0.048;
            static double gearRatio = 1d;
            static double distanceFromCenter = 0.1703;
            //42.6 degrees for 1 wheel rotation
            //1111 ticks per full rotation of robot
            //4.5 in to center on y
            //5 in to center x
            //0.048/(0.1703)*360*.42
        }
    }

    public static class Software{
        public static class AprilTags{
            static int ZONE_1_ID = 213;
            static int ZONE_2_ID = 214;
            static int ZONE_3_ID = 215;
        }

        public static class Motor {
            public static double verticalCoefficient = 1d;
            public static double horizontalCoefficient = 1d;
            public static double rotationCoefficient = 1d;

        }
    }
}
