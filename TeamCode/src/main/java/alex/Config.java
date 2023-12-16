package alex;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    public static class Hardware {
        public static class Motor {
            public static String frontLeftMotorName = "frontLeftMotor";
            public static DcMotorEx.Direction frontLeftMotorDirection = DcMotorEx.Direction.FORWARD;
            public static String frontRightMotorName = "frontRightMotor";
            public static DcMotorEx.Direction frontRightMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String backLeftMotorName = "backLeftMotor";
            public static DcMotorEx.Direction backLeftMotorDirection = DcMotorEx.Direction.FORWARD;
            public static String backRightMotorName = "backRightMotor";
            public static DcMotorEx.Direction backRightMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String winchMotorName = "winchMotor";
            public static DcMotorEx.Direction winchMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String armMotorName = "armMotor";
            public static DcMotorEx.Direction armMotorDirection = DcMotorEx.Direction.REVERSE;
            public static String slideMotorName = "slideMotor";
            public static DcMotorEx.Direction slideMotorDirection = DcMotorEx.Direction.FORWARD;

            public static double driveMotorPPR =  ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double strafeMultiplier = 100d/89d;

        }

        public static class Servo {
            public static String clawServoName = "clawServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double clawOpenPosition = 0.0;
            public static double clawClosedPosition = 0.13;
            public static String wristServoName = "wristServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction wristServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double wristServoLow = 0.4;
            public static double wristServoHigh = 0.3;
            public static double wristServoZero = 0.0;
            static int clawAutoTiltHeight = (int) (1900/(100/17.5));

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
        public static class PawnPosition{
            public static int centerLeft = 230;
            public static int centerRight = 430;
        }

        public static class Motor {
            public static double verticalCoefficient = 1d;
            public static double horizontalCoefficient = 1d;
            public static double rotationCoefficient = 1d;

        }
    }
}
