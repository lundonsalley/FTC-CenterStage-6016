package alex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.concurrent.Callable;

public class Instruction {
    public enum Code {
        Move,
        Rotate,
        Custom,
        Claw,
        Arm,
        Slide,
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

    private void rotate() {
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


}
