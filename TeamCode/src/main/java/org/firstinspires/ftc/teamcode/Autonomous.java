package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import alex.AutonomousNavigator;
import alex.Config;
import alex.PositionCalculator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Linear Opmode")
//@Disabled


public class Autonomous extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor winchMotor;
    private DcMotor slideMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo clawServo;
    private boolean targetClawOpen = true;
    private boolean targetWristUp = true;
    private boolean targetWinchDown = true;
    private double targetClawPosition = 0;
    private double targetWristPosition = 0;
    private int targetWinchPosition = 0;
    PositionCalculator posCalc;
    AutonomousNavigator autoNav;

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor"); //E port 2
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"); //E port 3
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor"); //E port 0
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor"); //E port 1

        winchMotor = hardwareMap.dcMotor.get("winchMotor"); //port 0
        slideMotor = hardwareMap.dcMotor.get("slideMotor"); //port 1
        armMotor = hardwareMap.dcMotor.get("armMotor"); //port 2

        wristServo = hardwareMap.servo.get("wristServo"); //servo port 0
        clawServo = hardwareMap.servo.get("clawServo"); //servo port 1


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        winchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.REVERSE);

        posCalc = new PositionCalculator(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        autoNav = new AutonomousNavigator(posCalc);

        ////////////////////////////////////////////////////////////////////////////////////////////

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

        //autoNav.move(new Position(DistanceUnit.METER, 0, .02, 0, 500), frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        boolean go = true;

        if (isStopRequested()) return;
        while(opModeIsActive()) {
            if(go) {
                wrist(true);
                Thread.sleep(100);
                drive(0.0, 1.0, 0.0, 0.7);
                Thread.sleep(850);
                claw(false);
                go = false;
            }else
                drive(0.0,0.0,0.0,0.7);
        }


    }
    public void drive(double x, double y, double rx,double power){
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * power);
        backLeftMotor.setPower(backLeftPower * power);
        frontRightMotor.setPower(frontRightPower * power);
        backRightMotor.setPower(backRightPower * power);
    }

    public void claw(boolean open){
        if(open){
            targetClawPosition = 0.325; //if the button was pressed down, toggle the claw
        }else{
            targetClawPosition = 0.1;
        }
        clawServo.setPosition(targetClawPosition);
    }

    public void wrist(boolean up){
        if(up){
            targetWristPosition = 0.47; //if the button was pressed down, toggle the claw
        }else{
            targetWristPosition = 0.0;
        }
        clawServo.setPosition(targetClawPosition);
    }
}