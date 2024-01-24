package alex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
            public static DcMotorEx.Direction winchMotorDirection = DcMotorEx.Direction.FORWARD;
            public static String armMotorName = "armMotor";
            public static DcMotorEx.Direction armMotorDirection = DcMotorEx.Direction.REVERSE;
            public static double armMoveVelo = 0.4;
            public static int armStoredPos = 0;
            public static int armUpPos = 750;
            public static int armDownPos = 330;
            public static String elbowMotorName = "elbowMotor";
            public static DcMotorSimple.Direction elbowMotorDirection = DcMotorSimple.Direction.FORWARD;
            public static double elbowMoveVelo = 0.2;
            public static int elbowStoredPos = 0;
            public static int elbowDownPos = 560;
            public static int elbowUpPos = 440; //to imp
            
            
            
            public static double driveMotorPPR =  ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double winchMotorPPR = ((((1+(46d/17))) * (1+(46d/11))) * 28);
            public static double strafeMultiplier = 100d/89d;

        }

        public static class Servo {
            public static String clawServoLName = "clawServoL"; //x
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoLDirection = com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
            public static double clawLOpenPosition = 0.0;
            public static double clawLClosedPosition = 0.083;
            public static double clawLFullClosedPosition = 0.2;
            public static String clawServoRName = "clawServoR"; //b
            public static com.qualcomm.robotcore.hardware.Servo.Direction clawServoRDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double clawROpenPosition = 0.0;
            public static double clawRClosedPosition = 0.0838;
            public static double clawRFullClosedPosition = 0.2;
            public static String leftLimitServoName = "leftLimitServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction leftLimitServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double leftLimitStowed = 0.0;
            public static double leftLimitDeployed = 0.0; //to imp
            public static String rightLimitServoName = "rightLimitServo";
            public static com.qualcomm.robotcore.hardware.Servo.Direction rightLimitServoDirection = com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
            public static double rightLimitStowed = 0.0;
            public static double rightLimitDeployed = 0.0; //to imp
            
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


        }
    }
}
