package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Linear Opmode")
//@Disabled


public class Autonomous extends MainTeleOp {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor winchMotor;
    private DcMotor slideMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo clawServo;
    private boolean targetClawOpen = true;
    private boolean targetWristUp = true;
    private boolean targetWinchDown = true;
    private double targetClawPosition = 0;
    private double targetWristPosition = 0;
    private int targetWinchPosition = 0;
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {



    }
}