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
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new examplePipeline());

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

        Mat HSV = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat centerCrop;
        double leftavgfin;
        double rightavgfin;
        double centeravgfin;
        Mat outPut = new Mat();
        Scalar rectColor1 = new Scalar(255.0, 0.0, 0.0);
        Scalar rectColor2 = new Scalar(255.0, 255.0, 0.0);
        Scalar rectColor3 = new Scalar(255.0, 0.0, 255.0);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 140, 269, 79);
            Rect rightRect = new Rect(370, 140, 269, 79);
            Rect centerRect = new Rect(270, 140, 99, 79);



            Scalar lowHSV = new Scalar(0.0,0.0,0.0);
            Scalar highHSV = new Scalar(255.0,255.0,255.0);
            Mat thresh = new Mat();

            Core.inRange(input, new Scalar(255.0,255.0,255.0), new Scalar(0.0,0.0,0.0), thresh);

            leftCrop = HSV.submat(leftRect);
            rightCrop = HSV.submat(rightRect);
            centerCrop = HSV.submat(centerRect);


            thresh.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor2, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor3, 2);

            return outPut;
        }
    }
}