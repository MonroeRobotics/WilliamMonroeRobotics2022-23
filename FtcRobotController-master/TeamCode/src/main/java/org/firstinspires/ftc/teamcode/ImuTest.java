package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Imu Test", group="test")
public class ImuTest extends LinearOpMode {

    Orientation lastAngles = new Orientation();

    BNO055IMU imu;

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    public void turnAngle(float angle, double turnPower){

        double motorSpeed = 2500;
        double startingAngle = imu.getAngularOrientation().firstAngle;


        // Encoders to set motors to either actively hold position or move freely based on the usage
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(10);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (turnPower > 0){
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
        }


        while (Math.abs(startingAngle - imu.getAngularOrientation().firstAngle) <= angle) {
            if(Math.abs(startingAngle - imu.getAngularOrientation().firstAngle) > angle - 20){
                backLeft.setVelocity(Math.abs(motorSpeed * turnPower / 2));
                frontRight.setVelocity(Math.abs(motorSpeed * turnPower / 2));
                backRight.setVelocity(Math.abs(motorSpeed * turnPower / 2));
                frontLeft.setVelocity(Math.abs(motorSpeed * turnPower / 2));
            }
            else{
                backLeft.setVelocity(Math.abs(motorSpeed * turnPower));
                frontRight.setVelocity(Math.abs(motorSpeed * turnPower));
                backRight.setVelocity(Math.abs(motorSpeed * turnPower));
                frontLeft.setVelocity(Math.abs(motorSpeed * turnPower));
            }


        }

        if (turnPower < 0){
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
        }

        while (Math.abs(startingAngle - imu.getAngularOrientation().firstAngle) >= angle){
            backLeft.setVelocity(Math.abs(motorSpeed * turnPower / 4));
            frontRight.setVelocity(Math.abs(motorSpeed * turnPower / 4));
            backRight.setVelocity(Math.abs(motorSpeed * turnPower / 4));
            frontLeft.setVelocity(Math.abs(motorSpeed * turnPower / 4));
        }

        if (turnPower > 0){
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
        }


        while (Math.abs(startingAngle - imu.getAngularOrientation().firstAngle) <= angle) {

                backLeft.setVelocity(Math.abs(motorSpeed * turnPower / 8));
                frontRight.setVelocity(Math.abs(motorSpeed * turnPower / 8));
                backRight.setVelocity(Math.abs(motorSpeed * turnPower / 8));
                frontLeft.setVelocity(Math.abs(motorSpeed * turnPower / 8));
        }

        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
    }

    public void runOpMode(){

        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");

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

        turnAngle(90, .5);

        while(opModeIsActive()){

            lastAngles = imu.getAngularOrientation() ;
            telemetry.addData("Angles", lastAngles.firstAngle);
            telemetry.update();

        }
    }
}
