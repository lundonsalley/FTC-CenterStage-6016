package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorExSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

import alex.AutonomousNavigator;
import alex.Config;
import alex.PositionCalculator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Autonomous Far", group = "Linear Opmode")
//@Disabled


public class BlueAutonomousFar extends LinearOpMode {

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
        whisker("stowed");
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
            telemetry.update();
            if(setup) {
                autoNav.run();
                whiskerDetection();
                if(markerPos != MarkerPositions.CENTER || runtime.seconds()>4){
                    configAutoNav();
                    setup = false;
                }
            }else{
                autoNav.run();
                telemetry.update();
            }

        }
    }


    public void configAutoNav(){  //starting with front wheels on the center spike marker
        whisker("stowed");
        switch (markerPos){
            case LEFT:
                //store arm to avoid the metal bar
                autoNav.arm(Config.Hardware.Motor.armStoredPos, Config.Hardware.Motor.elbowStoredPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //rotate 90deg left
                autoNav.rotate(Math.PI/2,frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //move backwards so the pixel is aligned with the spike marker
                autoNav.move(new Position(DistanceUnit.METER,0,-8.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //move arm down
                autoNav.arm(Config.Hardware.Motor.armDownPos, Config.Hardware.Motor.elbowDownPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //move to spike marker
                autoNav.move(new Position(DistanceUnit.METER,0,3.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //drop purple pixel on the spike marker
                autoNav.claw(Config.Hardware.Servo.clawLOpenPosition,clawServoL);
                //wait for the pixel to be placed
                autoNav.sleep(750);
                //move backwards to avoid bumping pixel
                autoNav.move(new Position(DistanceUnit.METER,0,-2.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //set arm to stored position
                autoNav.arm(Config.Hardware.Motor.armStoredPos, Config.Hardware.Motor.elbowStoredPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //close claw
                autoNav.claw(Config.Hardware.Servo.clawLClosedPosition, clawServoL);
                //end
                break;
            case CENTER:
                //move backwards so the claw is centered with the spike marker
                autoNav.move(new Position(DistanceUnit.METER,0,-7.5*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //drop purple pixel on the spike marker
                autoNav.claw(Config.Hardware.Servo.clawLOpenPosition,clawServoL);
                //wait for the pixel to be placed
                autoNav.sleep(750);
                //set arm to stored position
                autoNav.arm(Config.Hardware.Motor.armStoredPos, Config.Hardware.Motor.elbowStoredPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //close both claws
                autoNav.claw(Config.Hardware.Servo.clawLClosedPosition,clawServoL);
                //end
                break;
            case RIGHT:
                //store arm to avoid the metal bar (metal bar doesn't exist on this side)
                autoNav.arm(Config.Hardware.Motor.armStoredPos, Config.Hardware.Motor.elbowStoredPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //rotate 90deg right
                autoNav.rotate(-Math.PI/2,frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //move backwards so the pixel is aligned with the spike marker
                autoNav.move(new Position(DistanceUnit.METER,0,-12.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //move arm down
                autoNav.arm(Config.Hardware.Motor.armDownPos, Config.Hardware.Motor.elbowDownPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //realign with spike marker
                autoNav.move(new Position(DistanceUnit.METER,0,4.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //drop purple pixel on the spike marker
                autoNav.claw(Config.Hardware.Servo.clawLOpenPosition,clawServoL);
                //wait for the pixel to be placed
                autoNav.sleep(750);
                //move back to avoid bumping the pixel
                autoNav.move(new Position(DistanceUnit.METER,0,-5.0*convert,0, 500),
                        frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
                //set arm to stored position
                autoNav.arm(Config.Hardware.Motor.armStoredPos, Config.Hardware.Motor.elbowStoredPos,
                        Config.Hardware.Motor.armMoveVelo/2, armMotor, elbowMotor);
                //close claw
                autoNav.claw(Config.Hardware.Servo.clawLOpenPosition,clawServoL);
                //end
                break;
        }
        /*
            distances:
                center:
                    Y: (+) 2'
                    drop
                    Y: (-) 2'
                    X: (+) 4'
                left:
                    Y: (+) 1.25'
                    X: (-) (5.0/6.0)'
                    drop
                    Y: (-) 1.25'
                    X: (+) (29.0/6.0)'
                right:
                    Y: (+) 1.25'
                    X: (+) (21.0/24.0)'
                    drop
                    Y: (-) 1.25'
                    X: (+) (75.0/24.0)'
         */



    }

    public void setupNav(){ //arm down and move to center (front wheels touching the spike marker)
        arm("down");
        whisker("deployed");
        autoNav.move(new Position(DistanceUnit.METER,0,29.0*convert,0, 500),
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    public void whiskerDetection(){
        if(!whiskerL.isPressed()){
            this.markerPos = MarkerPositions.LEFT;
        }
        if(!whiskerR.isPressed()){
            this.markerPos = MarkerPositions.RIGHT;
        }
    }

    public void arm(String position) {
        if (position == "down") {
            armMotor.setTargetPosition(Config.Hardware.Motor.armDownPos);
            elbowMotor.setTargetPosition(Config.Hardware.Motor.elbowDownPos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Config.Hardware.Motor.armMoveVelo);
            elbowMotor.setPower(Config.Hardware.Motor.elbowMoveVelo);
        }
    }
    public void whisker(String position){
        if (position == "stowed") {
            rightLimitServo.setPosition(Config.Hardware.Servo.rightLimitStowed);
            leftLimitServo.setPosition(Config.Hardware.Servo.leftLimitStowed);
        }else if (position == "deployed") {
            rightLimitServo.setPosition(Config.Hardware.Servo.rightLimitDeployed);
            leftLimitServo.setPosition(Config.Hardware.Servo.leftLimitDeployed);
        }
    }

}