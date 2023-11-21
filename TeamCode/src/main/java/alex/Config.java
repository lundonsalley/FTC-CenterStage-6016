package alex;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "leftFront";
            public static DcMotor.Direction frontLeftMotorDirection = DcMotor.Direction.REVERSE;
            public static String frontRightMotorName = "rightFront";
            public static DcMotor.Direction frontRightMotorDirection = DcMotor.Direction.FORWARD;
            public static String backLeftMotorName = "leftRear";
            public static DcMotor.Direction backLeftMotorDirection = DcMotor.Direction.REVERSE;
            public static String backRightMotorName = "rightRear";
            public static DcMotor.Direction backRightMotorDirection = DcMotor.Direction.FORWARD;
            public static String winchMotorName = "winch_motor";
            public static DcMotor.Direction winchMotorDirection = DcMotor.Direction.FORWARD;

            public static double driveMotorPPR =  ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double strafeMultiplier = 100d/89d;

        }

        public static class Servo {
            public static String clawOpenServoName = "claw_open_servo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
            static double clawOpenPostion = 0.68;
            static double clawClosedPosition = 0.5;
            static String clawTiltServoName = "claw_tilt_servo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawTiltServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
            static double clawTiltServoLow = .55;
            static double clawTiltServoHigh = .8;
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
