package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import alex.Config;

@TeleOp(name = "MainTeleOp", group = "Linear Opmode")
//@Disabled


public class MainTeleOp extends LinearOpMode {

    //region VARIABLE DECLARATIONS
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor winchMotor;
    private DcMotor armMotor;
    private DcMotor elbowMotor;
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo leftLimitServo;
    private Servo rightLimitServo;
    private boolean targetClawROpen = false;
    private boolean targetClawLOpen = false;
    private boolean armUp = false;
    private boolean fullArmStored = true;
    private boolean winchDown = true;
    private double targetClawLPosition = Config.Hardware.Servo.clawLFullClosedPosition;
    private double targetClawRPosition = Config.Hardware.Servo.clawRFullClosedPosition;
    private int targetWinchPosition = 0;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad1 = new Gamepad();
    //endregion

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration

        //region CONFIG
        //motor config
        frontLeftMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.frontLeftMotorName);
        backLeftMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.backLeftMotorName);
        frontRightMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.frontRightMotorName);
        backRightMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.backRightMotorName);

        winchMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.winchMotorName);
        armMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.armMotorName);
        elbowMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.elbowMotorName);

        //servo config
        clawServoL = hardwareMap.servo.get(Config.Hardware.Servo.clawServoLName);
        clawServoR = hardwareMap.servo.get(Config.Hardware.Servo.clawServoRName);
        //leftLimitServo = hardwareMap.servo.get(Config.Hardware.Servo.leftLimitServoName);
        //rightLimitServo = hardwareMap.servo.get(Config.Hardware.Servo.rightLimitServoName);



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
        //leftLimitServo.setDirection(Config.Hardware.Servo.leftLimitServoDirection);
        //rightLimitServo.setDirection(Config.Hardware.Servo.rightLimitServoDirection);


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
        //endregion

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //updates current gamepad to match the controller
            try {
                currentGamepad1.copy(gamepad1);
            } catch (Exception e) {
                e.printStackTrace();
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            telemetry.addData("armPos",armMotor.getCurrentPosition());
            telemetry.addData("elbowPos",elbowMotor.getCurrentPosition());
            telemetry.addData("winchPos",winchMotor.getCurrentPosition());

            hand();
            arm();
            winch();
            drive(x, y, rx, 0.7);

            telemetry.update();

            //store gamepad for next cycle
            try {
                previousGamepad1.copy(currentGamepad1);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    public void hand(){
        //CLAW
        if ((!previousGamepad1.x && currentGamepad1.x) || (!previousGamepad1.left_stick_button && currentGamepad1.left_stick_button)) {
            targetClawLOpen = !targetClawLOpen;
            targetClawLPosition = targetClawLOpen ? Config.Hardware.Servo.clawLOpenPosition : Config.Hardware.Servo.clawLClosedPosition; //if the button was pressed down, toggle the claw
        }
        clawServoL.setPosition(targetClawLPosition);

        if ((!previousGamepad1.b && currentGamepad1.b) || (!previousGamepad1.right_stick_button && currentGamepad1.right_stick_button)) {
            targetClawROpen = !targetClawROpen;
            targetClawRPosition = targetClawROpen ? Config.Hardware.Servo.clawROpenPosition : Config.Hardware.Servo.clawRClosedPosition; //if the button was pressed down, toggle the claw
        }
        clawServoR.setPosition(targetClawRPosition);
    }
    public void arm() {
        if ((!previousGamepad1.a && currentGamepad1.a)) {
            if(fullArmStored){ //init position
                fullArmStored = !fullArmStored;
                armMotor.setTargetPosition(Config.Hardware.Motor.armUpPos);
                elbowMotor.setTargetPosition(Config.Hardware.Motor.elbowUpPos);
            }else {
                armUp = !armUp;
                if (armUp) {
                    armMotor.setTargetPosition(Config.Hardware.Motor.armDownPos);
                    elbowMotor.setTargetPosition(Config.Hardware.Motor.elbowDownPos);
                }else{
                    armMotor.setTargetPosition(Config.Hardware.Motor.armUpPos);
                    elbowMotor.setTargetPosition(Config.Hardware.Motor.elbowUpPos);
                }
            }
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Config.Hardware.Motor.armMoveVelo);
            elbowMotor.setPower(Config.Hardware.Motor.elbowMoveVelo);
        }

        if ((!previousGamepad1.y && currentGamepad1.y)) { //store arm
            if (fullArmStored){
                return;
            }
            fullArmStored = !fullArmStored;
            armMotor.setTargetPosition(Config.Hardware.Motor.armStoredPos);
            elbowMotor.setTargetPosition(Config.Hardware.Motor.elbowStoredPos);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(Config.Hardware.Motor.armMoveVelo/2);
            elbowMotor.setPower(Config.Hardware.Motor.elbowMoveVelo/2);
        }
    }
    public void winch(){
        //adjust and reset the winch
        if((gamepad1.dpad_up)&& (winchMotor.getCurrentPosition()<Config.Hardware.Motor.winchUpPos)){
            winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            winchMotor.setPower(Config.Hardware.Motor.winchVelo);
        }
        if((gamepad1.dpad_down)&&(winchMotor.getCurrentPosition()>Config.Hardware.Motor.winchDownPos)){
                winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                winchMotor.setPower(-Config.Hardware.Motor.winchVelo);
        }
        if((previousGamepad1.dpad_up != currentGamepad1.dpad_up)
                ||(previousGamepad1.dpad_down != currentGamepad1.dpad_down)
                ||(winchMotor.getCurrentPosition()>Config.Hardware.Motor.winchUpPos)
                ||(winchMotor.getCurrentPosition()<Config.Hardware.Motor.winchDownPos)){
            winchMotor.setPower(0);
        }
    }
    public void drive(double x, double y, double rx, double power){
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
}