package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class magnetTEst extends LinearOpMode {
    // Define variables for our touch sensor
    TouchSensor limit;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        limit = hardwareMap.get(TouchSensor.class, "limit");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Switch is pressed, stop the motor
            if (limit.isPressed()) {
                telemetry.addLine("isPressed");
            }
            else { // Otherwise, run the motor
                telemetry.addLine("notPressed");
            }
            telemetry.update();
        }
    }
}