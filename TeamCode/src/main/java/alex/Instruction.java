package alex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.Callable;

import alex.AprilTagPositioning;
import alex.AutonomousNavigator;

public class Instruction {
    public enum Code {
        Move,
        Rotate,
        Custom,
        Claw,
        Arm,
        Slide,
        GoToAprilTag,
        RotateToAprilTag,
        Sleep
    }

    public enum Reference {
        Relative,
        Absolute
    }

    private final Code instructionCode;
    private final Object[] parameters;
    private boolean firstIteration = true;
    private boolean complete = false;
    private PositionCalculator posCalc;

    public Instruction(Code instructionCode, Object[] parameters) {
        this.instructionCode = instructionCode;
        this.parameters = parameters;
    }

    public boolean isComplete(){
        return complete;
    }

    public void execute(PositionCalculator posCalc) throws Exception {
        this.posCalc = posCalc;
        switch (instructionCode){
            case Move:
                move();
                break;
            case Rotate:
                rotate();
                break;
            case Arm:
                arm();
                break;
            case Slide:
                slide();
                break;
            case Claw:
                claw();
                break;
            case GoToAprilTag:
                goToAprilTag();
                break;
            case RotateToAprilTag:
                rotateToAprilTag();
                break;
            case Custom:
                custom();
                break;
            case Sleep:
                sleep();
                break;
        }
        firstIteration = false;
    }


    private void custom() {
        Runnable code = (Runnable) parameters[0];
        code.run();
        complete = true;
    }

    private void sleep() throws Exception {
        long sleepTime = (long) parameters[0];
        Callable<Boolean> stopCondition = (Callable<Boolean>) parameters[1];
        if (firstIteration){
            parameters[0] = System.currentTimeMillis() + (long) parameters[0];
            sleepTime = (long) parameters[0];
        }

        if ((stopCondition != null && stopCondition.call()) || (System.currentTimeMillis() - sleepTime > 0)){
            complete = true;
        }
    }

    private void claw() {
        double targetOpenPosition = (double) parameters[0];
        Servo clawServo = (Servo) parameters[1];


        if (firstIteration){
            clawServo.setPosition(targetOpenPosition);
            complete = true;
        }

    }

    private void arm() {
        int targetArmPosition = (int) parameters[0];
        int targetElbowPosition = (int) parameters[1];
        double liftSpeed = (double) parameters[2];
        DcMotorEx armMotor = (DcMotorEx) parameters[3];
        DcMotorEx elbowMotor = (DcMotorEx) parameters[4];

        if (firstIteration) {
            armMotor.setPower(liftSpeed);
            elbowMotor.setPower(liftSpeed);
            armMotor.setTargetPosition(targetArmPosition);
            elbowMotor.setTargetPosition(targetElbowPosition);
        }

        if (Math.abs(armMotor.getCurrentPosition() - targetArmPosition) < 10){
            complete = true;
        }
        if (Math.abs(armMotor.getCurrentPosition() - targetElbowPosition) < 10){
            complete = true;
        }
    }
    private void slide() {
        int targetPosition = (int) parameters[0];
        double liftSpeed = (double) parameters[1];
        DcMotorEx slideMotor = (DcMotorEx) parameters[2];

        if (firstIteration) {
            slideMotor.setVelocity(liftSpeed);
            slideMotor.setTargetPosition(targetPosition);
        }

        if (Math.abs(slideMotor.getCurrentPosition() - targetPosition) < 10){
            complete = true;
        }
    }

    private void rotate_() {
        double radians = (double) parameters[0];
        DcMotor frontLeftMotor = (DcMotor) parameters[1];
        DcMotor frontRightMotor = (DcMotor) parameters[2];
        DcMotor backLeftMotor = (DcMotor) parameters[3];
        DcMotor backRightMotor = (DcMotor) parameters[4];

        int buffer = 250;

        if (firstIteration){
            //calculate and set new position for wheel encoders

            int frontLeftEncoder = frontLeftMotor.getCurrentPosition();
            int frontRightEncoder = frontRightMotor.getCurrentPosition();
            int backLeftEncoder = backLeftMotor.getCurrentPosition();
            int backRightEncoder = backRightMotor.getCurrentPosition();

            frontLeftEncoder += radians * Config.Hardware.Motor.rotateMultiplierLeft;
            frontRightEncoder +=  radians * Config.Hardware.Motor.rotateMultiplierRight;
            backLeftEncoder +=  radians * Config.Hardware.Motor.rotateMultiplierLeft;
            backRightEncoder +=  radians * Config.Hardware.Motor.rotateMultiplierRight;

            frontLeftMotor.setTargetPosition(frontLeftEncoder);
            frontRightMotor.setTargetPosition(frontRightEncoder);
            backLeftMotor.setTargetPosition(backLeftEncoder);
            backRightMotor.setTargetPosition(backRightEncoder);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftMotor.setPower(Config.Hardware.Motor.moveVelo);
            frontRightMotor.setPower(Config.Hardware.Motor.moveVelo);
            backLeftMotor.setPower(Config.Hardware.Motor.moveVelo);
            backRightMotor.setPower(Config.Hardware.Motor.moveVelo);

        }

        if (!(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition());
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition());
            this.complete = true;
        }


    }

    private void rotate() {
        int degrees = (int) parameters[0];
        GyroSensor gyro = (GyroSensor) parameters[1];
        DcMotor frontLeftMotor = (DcMotor) parameters[2];
        DcMotor frontRightMotor = (DcMotor) parameters[3];
        DcMotor backLeftMotor = (DcMotor) parameters[4];
        DcMotor backRightMotor = (DcMotor) parameters[5];

        int dir = 1;
        double err = Config.Hardware.Motor.moveVelo * 15;

        if (firstIteration){
            if (degrees > 0)
                dir = -1;
            degrees = gyro.getHeading() - degrees;
            if(degrees < 0)
                degrees += 360;
            if(degrees > 359)
                degrees -= 360;
            Config.Software.degrees = degrees;


            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            frontLeftMotor.setPower(.5 * -dir);
            frontRightMotor.setPower(.5 * dir);
            backLeftMotor.setPower(.5 * -dir);
            backRightMotor.setPower(.5 * dir);
        }

        if (Config.Software.withinErr(gyro.getHeading(), Config.Software.degrees, (int) err)){
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition());
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition());

            this.complete = true;
        }

    }

    private void rotate(int degrees, GyroSensor gyro, DcMotor flm, DcMotor frm, DcMotor blm, DcMotor brm) {

        int dir = 1;
        double err = Config.Hardware.Motor.moveVelo * 5;

        if (firstIteration){
            if (degrees > 0)
                dir = -1;
            degrees = gyro.getHeading() - degrees;
            if(degrees < 0)
                degrees += 360;
            if(degrees > 359)
                degrees -= 360;
            Config.Software.degrees = degrees;


            flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            flm.setPower(.5 * -dir);
            frm.setPower(.5 * dir);
            blm.setPower(.5 * -dir);
            brm.setPower(.5 * dir);
        }

        if (Config.Software.withinErr(gyro.getHeading(), Config.Software.degrees, (int) err)){
            flm.setPower(0);
            frm.setPower(0);
            blm.setPower(0);
            brm.setPower(0);

            flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            flm.setTargetPosition(flm.getCurrentPosition());
            frm.setTargetPosition(frm.getCurrentPosition());
            blm.setTargetPosition(blm.getCurrentPosition());
            brm.setTargetPosition(brm.getCurrentPosition());

            this.complete = true;
        }

    }

    private void move() throws Exception {
        Position offset = (Position) parameters[0];
        Callable<Boolean> stopCondition = (Callable<Boolean>) parameters[1];
        DcMotorEx frontLeftMotor = (DcMotorEx) parameters[2];
        DcMotorEx frontRightMotor = (DcMotorEx) parameters[3];
        DcMotorEx backLeftMotor = (DcMotorEx) parameters[4];
        DcMotorEx backRightMotor = (DcMotorEx) parameters[5];

        int buffer = 250;

        if (firstIteration){
            //calculate and set new position for wheel encoders

            int frontLeftEncoder = frontLeftMotor.getCurrentPosition();
            int frontRightEncoder = frontRightMotor.getCurrentPosition();
            int backLeftEncoder = backLeftMotor.getCurrentPosition();
            int backRightEncoder = backRightMotor.getCurrentPosition();

            frontLeftEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            frontRightEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            backLeftEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            backRightEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);

            frontLeftMotor.setTargetPosition(frontLeftEncoder);
            frontRightMotor.setTargetPosition(frontRightEncoder);
            backLeftMotor.setTargetPosition(backLeftEncoder);
            backRightMotor.setTargetPosition(backRightEncoder);

            if (Math.abs(frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition()) < buffer){
                frontLeftMotor.setVelocity(buffer);
                frontRightMotor.setVelocity(buffer);
                backLeftMotor.setVelocity(buffer);
                backRightMotor.setVelocity(buffer);
            } else {
                frontLeftMotor.setVelocity(offset.acquisitionTime);
                frontRightMotor.setVelocity(offset.acquisitionTime);
                backLeftMotor.setVelocity(offset.acquisitionTime);
                backRightMotor.setVelocity(offset.acquisitionTime);
            }
        }

        if ((stopCondition != null && stopCondition.call()) | !(frontLeftMotor.isBusy() || frontRightMotor.isBusy() || backLeftMotor.isBusy() || backRightMotor.isBusy())){
            frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
            frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
            backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition());
            backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition());
            this.complete = true;
        }
    }

    private void move(Position offset, DcMotorEx flm, DcMotorEx frm, DcMotorEx blm, DcMotorEx brm){

        int buffer = 250;

        if (firstIteration){
            //calculate and set new position for wheel encoders

            int frontLeftEncoder = flm.getCurrentPosition();
            int frontRightEncoder = frm.getCurrentPosition();
            int backLeftEncoder = blm.getCurrentPosition();
            int backRightEncoder = brm.getCurrentPosition();

            frontLeftEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            frontRightEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            backLeftEncoder += posCalc.distanceToMotorRotation(offset.y) - posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);
            backRightEncoder += posCalc.distanceToMotorRotation(offset.y) + posCalc.distanceToMotorRotation(offset.x*Config.Hardware.Motor.strafeMultiplier);

            flm.setTargetPosition(frontLeftEncoder);
            frm.setTargetPosition(frontRightEncoder);
            blm.setTargetPosition(backLeftEncoder);
            brm.setTargetPosition(backRightEncoder);

            if (Math.abs(flm.getTargetPosition() - flm.getCurrentPosition()) < buffer){
                flm.setVelocity(buffer);
                frm.setVelocity(buffer);
                blm.setVelocity(buffer);
                brm.setVelocity(buffer);
            } else {
                flm.setVelocity(offset.acquisitionTime);
                frm.setVelocity(offset.acquisitionTime);
                blm.setVelocity(offset.acquisitionTime);
                brm.setVelocity(offset.acquisitionTime);
            }
        }

        if (!(flm.isBusy() || frm.isBusy() || blm.isBusy() || brm.isBusy())){
            flm.setTargetPosition(flm.getCurrentPosition());
            frm.setTargetPosition(frm.getCurrentPosition());
            blm.setTargetPosition(blm.getCurrentPosition());
            brm.setTargetPosition(brm.getCurrentPosition());
            this.complete = true;
        }
    }

    private void goToAprilTag(){
        AprilTagProcessor aprilTag = (AprilTagProcessor) parameters[0];
        int id = (int) parameters[1];
        Double XYZ[] = (Double[]) parameters[2];
        DcMotorEx frontLeftMotor = (DcMotorEx)  parameters[3];
        DcMotorEx frontRightMotor = (DcMotorEx) parameters[4];
        DcMotorEx backLeftMotor = (DcMotorEx) parameters[5];
        DcMotorEx backRightMotor = (DcMotorEx) parameters[6];
        double in = 0.0254;
        //autoNav.move(new Position(DistanceUnit.INCH, 0, 5, 0, 500),frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        if(firstIteration) {
            boolean d = false;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection atd : currentDetections) {
                if (atd.id == id) {
                    AprilTagPositioning.Pos.x = atd.ftcPose.x;
                    AprilTagPositioning.Pos.y = atd.ftcPose.y;
                    AprilTagPositioning.Pos.z = atd.ftcPose.z;
                    AprilTagPositioning.Pos.pitch = atd.ftcPose.pitch;
                    AprilTagPositioning.Pos.roll = atd.ftcPose.roll;
                    AprilTagPositioning.Pos.yaw = atd.ftcPose.yaw;
                    AprilTagPositioning.Pos.range = atd.ftcPose.range;
                    AprilTagPositioning.Pos.bearing = atd.ftcPose.bearing;
                    AprilTagPositioning.Pos.elevation = atd.ftcPose.elevation;
                    d = true;
                }
            }
            if (currentDetections.size() == 0 || !d) {
                //no tags or wrong tag detected
                this.complete = true;
                return;
            }
            AprilTagPositioning.Target.pos[0] = XYZ;
            AprilTagPositioning.updateDeltas();
        }
        double x = AprilTagPositioning.Pos.dx;
        double y = AprilTagPositioning.Pos.dy;
        
        this.move(new Position(DistanceUnit.METER, -x*in, y*in, 0, 500),frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }

    private void rotateToAprilTag(){
        AprilTagProcessor aprilTag = (AprilTagProcessor) parameters[0];
        int id = (int) parameters[1];
        int angle = (int) parameters[2];
        GyroSensor gyro = (GyroSensor) parameters[3];
        DcMotorEx frontLeftMotor = (DcMotorEx)  parameters[4];
        DcMotorEx frontRightMotor = (DcMotorEx) parameters[5];
        DcMotorEx backLeftMotor = (DcMotorEx) parameters[6];
        DcMotorEx backRightMotor = (DcMotorEx) parameters[7];
        //autoNav.move(new Position(DistanceUnit.INCH, 0, 5, 0, 500),frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        if(firstIteration) {
            boolean d = false;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection atd : currentDetections) {
                if (atd.id == id) {
                    AprilTagPositioning.Pos.x = atd.ftcPose.x;
                    AprilTagPositioning.Pos.y = atd.ftcPose.y;
                    AprilTagPositioning.Pos.z = atd.ftcPose.z;
                    AprilTagPositioning.Pos.pitch = atd.ftcPose.pitch;
                    AprilTagPositioning.Pos.roll = atd.ftcPose.roll;
                    AprilTagPositioning.Pos.yaw = atd.ftcPose.yaw;
                    AprilTagPositioning.Pos.range = atd.ftcPose.range;
                    AprilTagPositioning.Pos.bearing = atd.ftcPose.bearing;
                    AprilTagPositioning.Pos.elevation = atd.ftcPose.elevation;
                    AprilTagPositioning.Pos.angle = (int) atd.ftcPose.yaw;
                    d = true;
                }
            }
            if (currentDetections.size() == 0 || !d) {
                //no tags or wrong tag detected
                this.complete = true;
                return;
            }
            AprilTagPositioning.Target.angle = angle;
            AprilTagPositioning.updateDeltas();
        }
        int deg = AprilTagPositioning.Pos.dangle;
        this.rotate(deg, gyro, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }
}
