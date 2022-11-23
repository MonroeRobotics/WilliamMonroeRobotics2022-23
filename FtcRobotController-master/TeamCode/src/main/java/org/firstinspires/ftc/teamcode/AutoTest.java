package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        public Mat processFrame(Mat input){

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipline running");

            Rect leftRect = new Rect(1, 140, 269, 79);
            Rect rightRect = new Rect(370, 140, 269, 79);
            Rect centerRect = new Rect(270, 140, 99, 79);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor1, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor2, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor3, 2);
            Imgproc.cvtColor(outPut, HSV, Imgproc.COLOR_RGB2HSV);

            leftCrop = HSV.submat(leftRect);
            rightCrop = HSV.submat(rightRect);
            centerCrop = HSV.submat(centerRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar centeravg = Core.mean(centerCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            centeravgfin = centeravg.val[0];

            telemetry.addData("Left avg:", leftavgfin);
            telemetry.addData("Right avg:", rightavgfin);
            telemetry.addData("Center avg:", centeravgfin);


            if (centeravgfin < leftavgfin || centeravgfin < rightavgfin) {
                if (leftavgfin > rightavgfin) {
                    telemetry.addLine("Left");
                } else {
                    telemetry.addLine("right");
                }
            }
            else{
                telemetry.addLine("center");
            }

            return(outPut);
        }
    }
}