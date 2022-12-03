package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Imu Test", group="test")
public class ImuTest extends LinearOpMode {

    Orientation lastAngles = new Orientation();

    BNO055IMU imu;

    public void runOpMode(){

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()){
            lastAngles = imu.getAngularOrientation() ;
            telemetry.addData("Angles", lastAngles);
            telemetry.update();

        }
    }
}
