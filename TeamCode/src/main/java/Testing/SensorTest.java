package Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp(name = "SensorTest", group = "Linear Opmode")
@Disabled
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our Sensors
        // Make sure your ID's match your configuration
        //DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        ColorSensor colorSensor = hardwareMap.colorSensor.get("colorSensor");
        //UltrasonicSensor ultraSensor = hardwareMap.ultrasonicSensor.get("ultraSensor");



        telemetry.addData("Color Sensor Value", colorSensor);
        //telemetry.addData("Ultrasonic Sensor Value", ultraSensor);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {







        }
    }
}