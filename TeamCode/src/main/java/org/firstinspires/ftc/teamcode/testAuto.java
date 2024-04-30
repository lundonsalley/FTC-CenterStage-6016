package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import alex.AutonomousNavigator;
import alex.Config;
import alex.PositionCalculator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Test Auto", group = "Linear Opmode")
//@Disabled


public class testAuto extends LinearOpMode {

    //region VARIABLE DECLARATIONS
    private final ElapsedTime runtime = new ElapsedTime();

    PositionCalculator posCalc;
    AutonomousNavigator autoNav;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx winchMotor;
    private DcMotorEx armMotor;
    private DcMotorEx elbowMotor;
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo leftLimitServo;
    private Servo rightLimitServo;
    private TouchSensor whiskerL;
    private TouchSensor whiskerR;
    private GyroSensor gyro;
    private boolean targetClawROpen = false;
    private boolean targetClawLOpen = false;
    private boolean armUp = false;
    private boolean fullArmStored = true;
    private boolean winchDown = true;
    private enum MarkerPositions {
        LEFT,
        RIGHT,
        CENTER
    }
    private MarkerPositions markerPos = MarkerPositions.CENTER;

    private boolean setup = true;

    private double convert = 0.0254; //converts meters to inches
    //endregion

    @Override
    public void runOpMode() throws InterruptedException {

        //region CONFIG
        posCalc = new PositionCalculator(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        autoNav = new AutonomousNavigator(posCalc);

        //motor config
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontLeftMotorName);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backLeftMotorName);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontRightMotorName);
        backRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backRightMotorName);

        winchMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.winchMotorName);
        armMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.armMotorName);
        elbowMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.elbowMotorName);

        //servo config
        clawServoL = hardwareMap.servo.get(Config.Hardware.Servo.clawServoLName);
        clawServoR = hardwareMap.servo.get(Config.Hardware.Servo.clawServoRName);
        leftLimitServo = hardwareMap.servo.get(Config.Hardware.Servo.leftLimitServoName);
        rightLimitServo = hardwareMap.servo.get(Config.Hardware.Servo.rightLimitServoName);

        //digital config
        whiskerL = hardwareMap.touchSensor.get(Config.Hardware.Digital.whiskerLName);
        whiskerR = hardwareMap.touchSensor.get(Config.Hardware.Digital.whiskerRName);

        //direction config
        frontLeftMotor.setDirection(Config.Hardware.Motor.frontLeftMotorDirection);
        frontRightMotor.setDirection(Config.Hardware.Motor.frontRightMotorDirection);
        backLeftMotor.setDirection(Config.Hardware.Motor.backLeftMotorDirection);
        backRightMotor.setDirection(Config.Hardware.Motor.backRightMotorDirection);

        winchMotor.setDirection(Config.Hardware.Motor.winchMotorDirection);
        armMotor.setDirection(Config.Hardware.Motor.armMotorDirection);
        elbowMotor.setDirection(Config.Hardware.Motor.elbowMotorDirection);

        clawServoL.setDirection(Config.Hardware.Servo.clawServoLDirection);
        clawServoR.setDirection(Config.Hardware.Servo.clawServoRDirection);
        leftLimitServo.setDirection(Config.Hardware.Servo.leftLimitServoDirection);
        rightLimitServo.setDirection(Config.Hardware.Servo.rightLimitServoDirection);


        //zero power behavior setup
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //starting out with zeroed out encoders

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

        clawServoL.setPosition(Config.Hardware.Servo.clawLClosedPosition);
        clawServoR.setPosition(Config.Hardware.Servo.clawRClosedPosition);
        gyro = hardwareMap.gyroSensor.get(Config.Hardware.Digital.gyroName);
        //endregion

        runtime.reset();

        waitForStart();

        if (isStopRequested()) return;

        if(opModeIsActive()){
            runtime.reset();
            setupNav();
        }

        while(opModeIsActive()) {
            telemetry.addData("runtime", runtime.seconds());
            telemetry.addData("gyro", gyro.getHeading());
            autoNav.run();
            telemetry.update();
        }
    }

    public void setupNav(){
        autoNav.rotate(180, gyro,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

}