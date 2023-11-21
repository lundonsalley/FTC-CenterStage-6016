package alex;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.concurrent.Callable;

public class Instruction {
    public enum Code {
        Move,
        Rotate,
        Custom,
        Claw,
        Lift,
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
            case Lift:
                lift();
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
        double targetTiltPosition = (double) parameters[2];
        Servo clawTiltServo = (Servo) parameters[3];


        if (firstIteration){
            clawServo.setPosition(targetOpenPosition);
            clawTiltServo.setPosition(targetTiltPosition);
            complete = true;
        }

    }

    private void lift() {
        int targetPosition = (int) parameters[0];
        int liftSpeed = (int) parameters[1];
        DcMotorEx winchMotor = (DcMotorEx) parameters[2];

        if (firstIteration) {
            winchMotor.setVelocity(liftSpeed);
            winchMotor.setTargetPosition(targetPosition);
        }

        if (Math.abs(winchMotor.getCurrentPosition() - targetPosition) < 10){
            complete = true;
        }
    }

    private void rotate() {

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
