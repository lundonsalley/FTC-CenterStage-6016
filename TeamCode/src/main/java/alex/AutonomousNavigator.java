package alex;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.BlueAutonomousBackdrop;

import java.util.ArrayList;
import java.util.concurrent.Callable;


public class AutonomousNavigator {

    private final PositionCalculator posCalc;
    private final ArrayList<Instruction> instructions;

    public AutonomousNavigator(PositionCalculator _posCalc) {
        this.posCalc = _posCalc;
        this.instructions = new ArrayList<>();
    }

    public void addInstruction(Instruction i){
        instructions.add(i);
    }

    public void move(Position distance, DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4){
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{distance, null, motor1, motor2, motor3, motor4}));
    }
    public void move(Position distance, Callable<Boolean> stopCondition, DcMotorEx motor1, DcMotorEx motor2, DcMotorEx motor3, DcMotorEx motor4){
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{distance, stopCondition, motor1, motor2, motor3, motor4}));
    }

    public void claw(double openPosition, Servo clawServo){
        addInstruction(new Instruction(Instruction.Code.Claw, new Object[]{openPosition, clawServo}));
    }

    public void arm(int armPosition, int elbowPosition, double power, DcMotorEx armMotor, DcMotorEx elbowMotor){
        addInstruction(new Instruction(Instruction.Code.Arm, new Object[]{armPosition, elbowPosition, power, armMotor, elbowMotor}));
    }
    public void slide(int position, double power, DcMotorEx slideMotor){
        addInstruction(new Instruction(Instruction.Code.Slide, new Object[]{position, power, slideMotor}));
    }

    public void sleep(long sleepTimeMilliseconds){
        addInstruction(new Instruction(Instruction.Code.Sleep, new Object[]{sleepTimeMilliseconds, null}));
    }

    public void sleep(long sleepTimeMilliseconds, Callable<Boolean> stopCondition){
        addInstruction(new Instruction(Instruction.Code.Sleep, new Object[]{sleepTimeMilliseconds, stopCondition}));
    }

    public void rotate(double radians, DcMotorEx frontL, DcMotorEx frontR, DcMotorEx backL, DcMotorEx backR){
        addInstruction(new Instruction(Instruction.Code.Rotate, new Object[]{radians, frontL, frontR, backL, backR}));
    }

    public void custom(Runnable code){
        Instruction i = new Instruction(Instruction.Code.Custom, new Object[]{code});
        addInstruction(i);
//        return i;
    }



    public void run(){
        for (Instruction i : instructions) {

            if (!i.isComplete()){
                try {
                    i.execute(posCalc);
                } catch (Exception e){
                    e.printStackTrace();
                }
                break;
            }
        }
    }





}
