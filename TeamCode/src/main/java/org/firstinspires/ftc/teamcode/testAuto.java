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
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import alex.AutonomousNavigator;
import alex.Config;
import alex.PositionCalculator;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Test Auto", group = "Linear Opmode")
//@Disabled


public class testAuto extends LinearOpMode {

    //region VARIABLE DECLARATIONS
    private final ElapsedTime runtime = new ElapsedTime();
    //april tag
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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
        initAprilTag();

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
            telemetryAprilTag();
            telemetry.update();
        }
    }

    public void setupNav(){
//        autoNav.rotate(180, gyro,
//                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        autoNav.sleep(2000);


        int angle = 0;
        autoNav.rotateToAprilTag(aprilTag,10, angle, gyro,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        autoNav.sleep(1000);
        Double XYZ[] = {5.0,5.0,null}; //inch
        autoNav.goToAprilTag(aprilTag,10, XYZ,
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    //april tag methods
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}