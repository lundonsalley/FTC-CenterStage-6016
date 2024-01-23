//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.DcMotorExSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//import alex.AutonomousNavigator;
//import alex.Config;
//import alex.PositionCalculator;
//
//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Autonomous Backdrop", group = "Linear Opmode")
////@Disabled
//
//
//public class BlueAutonomousBackdrop extends LinearOpMode {
//    private final ElapsedTime runtime = new ElapsedTime();
//    private DcMotorEx frontLeftMotor;
//    private DcMotorEx frontRightMotor;
//    private DcMotorEx backLeftMotor;
//    private DcMotorEx backRightMotor;
//    private DcMotorEx winchMotor;
//    private DcMotorEx slideMotor;
//    private DcMotorEx armMotor;
//    private Servo elbowServo;
//    private Servo clawServoR;
//    private Servo clawServoL;
//    private boolean targetClawOpen = true;
//    private boolean targetWristUp = true;
//    private boolean targetWinchDown = true;
//    private double targetClawPosition = 0;
//    private double targetWristPosition = 0;
//    private int targetWinchPosition = 0;
//    private double convert = 0.3048;
//    private boolean setup = true;
//    PositionCalculator posCalc;
//    AutonomousNavigator autoNav;
//
//    //tfod vars
//
//    private VisionPortal visionPortal;
//    private TfodProcessor tfod;
//    final boolean USE_WEBCAM = true;
//    private static final String TFOD_MODEL_FILE = "CenterStage.tflite";
//    private static final String[] LABELS = {
//            "Pawn",
//    };
//
//    private boolean left = false;
//    private boolean center = false;
//    private boolean right = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //motor config
//        frontLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontLeftMotorName); //E port 2
//        backLeftMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backLeftMotorName); //E port 3
//        frontRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.frontRightMotorName); //E port 0
//        backRightMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.backRightMotorName); //E port 1
//
//        winchMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.winchMotorName); //port 0
//        armMotor = hardwareMap.get(DcMotorEx.class, Config.Hardware.Motor.armMotorName); //port 2
//
//        wristServo = hardwareMap.servo.get(Config.Hardware.Servo.elbowServoName); //servo port 0
//        clawServoL = hardwareMap.servo.get(Config.Hardware.Servo.clawServoLName); //servo port 1
//        clawServoR = hardwareMap.servo.get(Config.Hardware.Servo.clawServoRName); //servo port 1
//
//
//        //direction config
//        frontLeftMotor.setDirection(Config.Hardware.Motor.frontLeftMotorDirection);
//        frontRightMotor.setDirection(Config.Hardware.Motor.frontRightMotorDirection);
//        backLeftMotor.setDirection(Config.Hardware.Motor.backLeftMotorDirection);
//        backRightMotor.setDirection(Config.Hardware.Motor.backRightMotorDirection);
//
//        winchMotor.setDirection(Config.Hardware.Motor.winchMotorDirection);
//        armMotor.setDirection(Config.Hardware.Motor.armMotorDirection);
//
//        clawServoL.setDirection(Config.Hardware.Servo.clawServoLDirection);
//        clawServoR.setDirection(Config.Hardware.Servo.clawServoRDirection);
//        elbowServo.setDirection(Config.Hardware.Servo.elbowServoDirection);
//
//
//        //zero power behavior setup
//        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        winchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        posCalc = new PositionCalculator(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
//        autoNav = new AutonomousNavigator(posCalc);
//
//        ////////////////////////////////////////////////////////////////////////////////////////////
//
//        //sets the robot to its default initialized position (all motors at their zero positions)
//
//        frontLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftMotor.setTargetPosition(0);
//        frontRightMotor.setTargetPosition(0);
//        backLeftMotor.setTargetPosition(0);
//        backRightMotor.setTargetPosition(0);
//        winchMotor.setTargetPosition(0);
//        armMotor.setTargetPosition(0);
//        slideMotor.setTargetPosition(0);
//
//        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        winchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        clawServoL.setPosition(Config.Hardware.Servo.clawLClosedPosition);
//        clawServoR.setPosition(Config.Hardware.Servo.clawRClosedPosition);
//        elbowServo.setPosition(0);
//
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setPower(0.15);
//
//
//
//
//        runtime.reset();
//
//        waitForStart();
//
//        initTfod();
//        telemetryTfod();
//
//        setup = true;
//
//        left = false;
//        right = false;
//        center = false;
//
//        if (isStopRequested()) return;
//        while(opModeIsActive()) {
//            if(getRuntime()<7) {
//
//                PawnDetection();
//
//                telemetry.addData("left", left);
//                telemetry.addData("right", right);
//                telemetry.addData("center", center);
//                telemetry.addData("pawns", PawnDetection());
//                telemetryTfod();
//                telemetry.update();
//            }else {
//                if(setup) {
//                    configAutoNav();
//                    setup=false;
//                }
//                armMotor.setPower(-0.3);
//                autoNav.run();
//                telemetry.update();
//            }
//        }
//    }
//
//    public void configAutoNav(){
//        autoNav.claw(Config.Hardware.Servo.clawClosedPosition,clawServo,Config.Hardware.Servo.wristServoLow,wristServo);
//        autoNav.move(new Position(DistanceUnit.METER,-4.2*convert,0,0, 500),frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
//
//        /*
//            distances:
//                center:
//                    Y: (+) 2'
//                    drop
//                    Y: (-) 2'
//                    X: (+) 4'
//                left:
//                    Y: (+) 1.25'
//                    X: (-) (5.0/6.0)'
//                    drop
//                    Y: (-) 1.25'
//                    X: (+) (29.0/6.0)'
//                right:
//                    Y: (+) 1.25'
//                    X: (+) (21.0/24.0)'
//                    drop
//                    Y: (-) 1.25'
//                    X: (+) (75.0/24.0)'
//         */
//
//
//
//    }
//
//    //tfod methods
//    private void initTfod() {
//
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use .setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//                .setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                .setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        //tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//
//        telemetry.update();
//
//    }
//
//    private int PawnDetection() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        if(currentRecognitions.size() > 0) {
//            Recognition pawn = currentRecognitions.get(0);
//
//            double x = 0;
//
//            for (Recognition recognition : currentRecognitions) {
//                x = (recognition.getLeft() + recognition.getRight()) / 2;
//
//                if (Math.max(recognition.getConfidence(), pawn.getConfidence()) == recognition.getConfidence()) {
//                    pawn = recognition;
//                }
//            }
//
//            if (x < Config.Software.PawnPosition.centerLeft) {
//                left = true;
//                right = false;
//                center = false;
//            }else if (x < Config.Software.PawnPosition.centerRight) {
//                center = true;
//                left = false;
//                right = false;
//            }else {
//                right = true;
//                left = false;
//                center = false;
//            }
//        }else{
//            //just in case the pawn isn't detected
//            right = true;
//            left = false;
//            center = false;
//        }
//        return(currentRecognitions.size());
//
//        //center = 330
//        //right = 530
//        //left =  130
//    }
//}