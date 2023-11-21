package alex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

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

    public void move(Position distance, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4){
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{distance, null, motor1, motor2, motor3, motor4}));
    }
    public void move(Position distance, Callable<Boolean> stopCondition, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4){
        addInstruction(new Instruction(Instruction.Code.Move, new Object[]{distance, stopCondition, motor1, motor2, motor3, motor4}));
    }

    public void claw(double openPosition, Servo clawServo, double tiltPosition, Servo tiltServo){
        addInstruction(new Instruction(Instruction.Code.Claw, new Object[]{openPosition, clawServo, tiltPosition, tiltServo}));
    }

    public void lift(int position, int speed, DcMotor winchMotor){
        addInstruction(new Instruction(Instruction.Code.Lift, new Object[]{position, speed, winchMotor}));
    }

    public void sleep(long sleepTimeMilliseconds){
        addInstruction(new Instruction(Instruction.Code.Sleep, new Object[]{sleepTimeMilliseconds, null}));
    }

    public void sleep(long sleepTimeMilliseconds, Callable<Boolean> stopCondition){
        addInstruction(new Instruction(Instruction.Code.Sleep, new Object[]{sleepTimeMilliseconds, stopCondition}));
    }

    public void rotate(){
        throw new RuntimeException("Rotation Not yet implemented");
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
