//package Testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import alex.Config;
//
//@TeleOp(name = "testOp", group = "Linear Opmode")
////@Disabled
//
//
//public class TestOp extends LinearOpMode {
//
//    private DcMotor frontLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor backRightMotor;
//    private DcMotor winchMotor;
//    private DcMotor slideMotor;
//    private DcMotor armMotor;
//    private Servo wristServo;
//    private Servo clawServo;
//    private boolean targetClawOpen = true;
//    private boolean targetWristUp = true;
//    private boolean targetWinchDown = true;
//    private double targetClawPosition = 0;
//    private double targetWristPosition = 0;
//    private int targetWinchPosition = 0;
//
//    private final Gamepad previousGamepad1 = new Gamepad();
//    private final Gamepad currentGamepad1 = new Gamepad();
//
//    @Override
//    public void runOpMode() {
//        // Declare our motors
//        // Make sure your ID's match your configuration
//
//        //motor config
//        frontLeftMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.frontLeftMotorName); //E port 2
//        backLeftMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.backLeftMotorName); //E port 3
//        frontRightMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.frontRightMotorName); //E port 0
//        backRightMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.backRightMotorName); //E port 1
//
//        winchMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.winchMotorName); //port 0
//        slideMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.slideMotorName); //port 1
//        armMotor = hardwareMap.dcMotor.get(Config.Hardware.Motor.armMotorName); //port 2
//
//        //servo config
//        wristServo = hardwareMap.servo.get(Config.Hardware.Servo.wristServoName); //servo port 0
//        clawServo = hardwareMap.servo.get(Config.Hardware.Servo.clawServoName); //servo port 1
//
//        //direction config
//        frontLeftMotor.setDirection(Config.Hardware.Motor.frontLeftMotorDirection);
//        frontRightMotor.setDirection(Config.Hardware.Motor.frontRightMotorDirection);
//        backLeftMotor.setDirection(Config.Hardware.Motor.backLeftMotorDirection);
//        backRightMotor.setDirection(Config.Hardware.Motor.backRightMotorDirection);
//
//        winchMotor.setDirection(Config.Hardware.Motor.winchMotorDirection);
//        armMotor.setDirection(Config.Hardware.Motor.armMotorDirection);
//        slideMotor.setDirection(Config.Hardware.Motor.slideMotorDirection);
//
//        clawServo.setDirection(Config.Hardware.Servo.clawServoDirection);
//        wristServo.setDirection(Config.Hardware.Servo.wristServoDirection);
//
//
//        //zero power behavior setup
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            //updates current gamepad to match the controller
//            try {
//                currentGamepad1.copy(gamepad1);
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            /*
//            DRIVER NOTES
//
//            left stick click AND (a) => open and close claw
//
//            left trigger and left bumper => rotate arm up and down
//
//            right trigger and right bumper => extend and retract slide
//
//            wrist servo => gear ratio with arm rotation / (b)
//             */
//
//            hand();
//            arm(armMotor.getCurrentPosition());
//            winch();
//            //drive(x, y, rx, 0.7);
//
//            telemetry.addData("Arm", armMotor.getCurrentPosition());
//            telemetry.addData("Winch", winchMotor.getCurrentPosition());
//            telemetry.addData("Slide", slideMotor.getCurrentPosition());
//
//            telemetry.addData("Wrist", wristServo.getPosition());
//            telemetry.addData("Claw", clawServo.getPosition());
//
//            //telemetry.addData("arm", armMotor.getCurrentPosition());
//            telemetry.update();
//
//            //store gamepad for next cycle
//            try {
//                previousGamepad1.copy(currentGamepad1);
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//        }
//    }
//    public void hand(){
//        //CLAW
//        if (currentGamepad1.dpad_up) {
//            targetClawPosition += 0.01; //if the button was pressed down, toggle the claw
//        }else if(currentGamepad1.dpad_down){
//            targetClawPosition -= 0.01;
//        }
//        clawServo.setPosition(targetClawPosition);
//
//        //WRIST
//        if (!previousGamepad1.b && currentGamepad1.b) {
//            targetWristUp = !targetWristUp;
//            targetWristPosition = targetWristUp ? Config.Hardware.Servo.wristServoHigh : Config.Hardware.Servo.wristServoLow; //if the button was pressed down, toggle the wrist
//        }
//        wristServo.setPosition(targetWristPosition);
//    }
//    public void arm(int armPos){
//        int slidePower;
//        double extendedPower = 1.0;
//
//        if(slideMotor.getCurrentPosition()>250)
//            extendedPower = 1.0;
//
//        if(gamepad1.left_trigger>0){
//            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if(armPos<200)
//                armMotor.setPower(-0.25*extendedPower);
//            else
//                armMotor.setPower(-0.15*extendedPower);
//            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }else if(gamepad1.left_bumper){
//            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if(armPos<150)
//                armMotor.setPower(0.25*extendedPower);
//            else
//                armMotor.setPower(0.3*extendedPower);
//            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }else {
//            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            if (armPos>85)
//                armMotor.setPower(0.15 * extendedPower);
//            else
//                armMotor.setPower(0.05 * extendedPower);
//            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        //slide movement
//        if(gamepad1.right_bumper){
//            slidePower = 1;
//        }else if(gamepad1.right_trigger>0){
//            slidePower = -1;
//        }else{
//            slidePower = 0;
//        }
//        slideMotor.setPower(.35*slidePower);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//    public void winch(){
//        if(!previousGamepad1.y && currentGamepad1.y){
//            targetWinchDown = !targetWinchDown;
//            targetWinchPosition = targetWinchDown ? 0 : 13500; //if the button was pressed down, toggle the winch direction
//        }
//        winchMotor.setTargetPosition(targetWinchPosition);
//        winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(targetWinchDown)
//            winchMotor.setPower(0.4);
//        else
//            winchMotor.setPower(1);
//
//        //adjust and reset the winch
//        if(gamepad1.dpad_up){
//            winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            winchMotor.setPower(1);
//        }
//        if(previousGamepad1.dpad_up != currentGamepad1.dpad_up){
//            targetWinchDown = true;
//            winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            winchMotor.setPower(0);
//        }
//        if(gamepad1.dpad_down){
//            winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            winchMotor.setPower(-1);
//        }
//        if(previousGamepad1.dpad_down != currentGamepad1.dpad_down){
//            targetWinchDown = true;
//            winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            winchMotor.setPower(0);
//        }
//    }
//    public void drive(double x, double y, double rx,double power){
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//        frontLeftMotor.setPower(frontLeftPower * power);
//        backLeftMotor.setPower(backLeftPower * power);
//        frontRightMotor.setPower(frontRightPower * power);
//        backRightMotor.setPower(backRightPower * power);
//    }
//}