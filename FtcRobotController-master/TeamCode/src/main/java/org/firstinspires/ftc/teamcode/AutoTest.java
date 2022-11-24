package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Auto Test", group="Concept")
public class AutoTest extends OpMode {
    OpenCvWebcam webcam = null;

    @Override
    public void init(){
        //gets webcam name and Id from configfile and initializes OpenCvWebcam object "webcam"
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //sets webcam Computer Vision Pipeline to examplePipeline
        webcam.setPipeline(new examplePipeline());

        //Starts streaming camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop(){
        telemetry.update();
    }

    class examplePipeline extends OpenCvPipeline {

        //Initializing Variables
        Mat HSV = new Mat();
        Mat outPut = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;
        double leftavgfin;
        double rightavgfin;
        double centeravgfin;
        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Scalar rectColor2 = new Scalar(255.0, 255.0, 0.0);
        Scalar rectColor3 = new Scalar(255.0, 0.0, 255.0);

        //Loops and processes every frame and returns desired changed
        public Mat processFrame(Mat input) {

            //changes Mat "input" from RGB to HSV and saves under MAT HSV
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            //Creates New rectangle objects for 3 regions, Left, Right, and Center
            Rect leftRect = new Rect(250, 140, 19, 79);
            Rect rightRect = new Rect(370, 140, 19, 79);
            Rect centerRect = new Rect(270, 140, 99, 79);

            //Creates the upper and lower range for the accepted HSV values for color of pole
            Scalar lowHSV = new Scalar(17,50,50);
            Scalar highHSV = new Scalar(22,255,255);

            //Returns Output Mat "thresh" that only contains pixels that are within low and high boundaries (lowHSV, highHSV)
            Mat thresh = new Mat();
            Core.inRange(HSV, lowHSV, highHSV, thresh);

            //creates submats(subsections) of MAT HSV using the rectangle regions
            leftCrop = HSV.submat(leftRect);
            rightCrop = HSV.submat(rightRect);
            centerCrop = HSV.submat(centerRect);

            //copies thresh Mat to outPut and draws rectangles regions on the image for user to see
            thresh.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor2, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor3, 2);

            //Returns to display outPut Mat
            return outPut;
        }
    }
}