package org.firstinspires.ftc.teamcode.helpers;

import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.vuforia.CameraCalibration;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.Tool;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.teamcode.RC;
//import org.opencv.core.Core;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;

import java.util.Arrays;

/**
 * Created by FIXIT on 16-11-20.
 */
public class VortexUtils {

    public final static int NOT_VISIBLE = 0;

    public final static int BEACON_BLUE_RED = 1;
    public final static int BEACON_RED_BLUE = 2;
    public final static int BEACON_ALL_BLUE = 3;
    public final static int BEACON_NO_BLUE = 4;

    public final static int OBJECT_BLUE = 1;
    public final static int OBJECT_RED = 2;

    //hsv blue beacon range colours
    //DON'T CHANGE THESE NUMBERS
//    public final static Scalar BEACON_BLUE_LOW = new Scalar(108, 0, 220);
//    public final static Scalar BEACON_BLUE_HIGH = new Scalar(178, 255, 255);
//
//    public final static Scalar OTHER_BLUE_LOW = new Scalar(105, 120, 110);
//    public final static Scalar OTHER_BLUE_HIGH = new Scalar(185, 255, 255);
//    public final static Scalar OTHER_RED_LOW = new Scalar(222, 101, 192);
//    public final static Scalar OTHER_RED_HIGH = new Scalar(47, 251, 255);

    static int RGB888to565(int red, int green, int blue){
        return (((red >> 3) & 0xFF) << 11) &
                (((green >> 2) & 0xFF) << 5) &
                ((blue >> 3) & 0xFF);
    }

    public final static int BEACON_BLUE_LOW = RGB888to565(108, 0, 220);
    public final static int BEACON_BLUE_HIGH = RGB888to565(178, 255, 255);

    public final static int OTHER_BLUE_LOW = RGB888to565(105, 120, 110);
    public final static int OTHER_BLUE_HIGH = RGB888to565(185, 255, 255);
    public final static int OTHER_RED_LOW =  RGB888to565(222, 101, 192);
    public final static int OTHER_RED_HIGH = RGB888to565(47, 251, 255);

//    public static Mat applyMask(Mat img, Scalar low, Scalar high) {
//
//        Mat mask = new Mat(img.size(), CvType.CV_8UC3);
//
//        if (high.val[0] < low.val[0]) {
//            Scalar lowMed = new Scalar(255, high.val[1], high.val[2]);
//            Scalar medHigh = new Scalar(0, low.val[1], low.val[2]);
//
//            Mat maskLow = new Mat(img.size(), CvType.CV_8UC3);
//            Mat maskHigh = new Mat(img.size(), CvType.CV_8UC3);
//
//            Core.inRange(img, low, lowMed, maskLow);
//            Core.inRange(img, medHigh, high, maskHigh);
//
//            Core.bitwise_or(maskLow, maskHigh, mask);
//        } else {
//            Core.inRange(img, low, high, mask);
//        }//else
//
//        return mask;
//    }

    public static int waitForBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal, long timeOut) {
        //DbgLog.msg("starting wait for");

        int config = NOT_VISIBLE;
        long beginTime = System.currentTimeMillis();
        //DbgLog.msg("inside wait for");
       // while (config == NOT_VISIBLE && System.currentTimeMillis() - beginTime < timeOut /*&& RC.l.opModeIsActive()*/) {
            config = getBeaconConfig(img, beacon, camCal);

           // RC.l.idle();
       // }//while

        return config;
    }

    public static int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {

        int leftp, rightp;

        OpenGLMatrix pose = beacon.getRawPose();
        DbgLog.msg("get raw pose"+pose);

        if (pose != null && img != null && img.getPixels() != null) {
            DbgLog.msg("inside if");

            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);

            rawPose.setData(poseData);
            //DbgLog.msg("raw pose data");

            //calculating pixel coordinates of beacon corners
            float[][] corners = new float[4][2];
            float[] leftbc = new float[2];
            float[] rightbc = new float[2];
            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 150, 0)).getData(); //lower right of beacon
            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 150, 0)).getData(); //lower left of beacon
            leftbc = Tool.projectPoint(camCal, rawPose, new Vec3F(-70, 230, 0)).getData(); //mid left of beacon
            rightbc = Tool.projectPoint(camCal, rawPose, new Vec3F(70, 230, 0)).getData(); //mid right of beacon
            //DbgLog.msg("corners[0] %f,%f", corners[0][0],corners[0][1]);
            //DbgLog.msg("corners[1] %f,%f", corners[1][0],corners[1][1]);
            //DbgLog.msg("corners[2] %f,%f", corners[2][0],corners[2][1]);
            //DbgLog.msg("corners[3] %f,%f", corners[3][0],corners[3][1]);


            //getting camera image...
            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            leftp = bm.getPixel(Math.round(leftbc[0]), Math.round(leftbc[1]));
            rightp = bm.getPixel(Math.round(rightbc[0]), Math.round(rightbc[1]));

            int leftpcol = isBlueOrRedPixel(leftp);
            int rightpcol = isBlueOrRedPixel(rightp);

            DbgLog.msg("leftp: %x", leftp);
            DbgLog.msg("rightp: %x", rightp);

            if (leftpcol == NOT_VISIBLE || rightpcol == NOT_VISIBLE)
                return NOT_VISIBLE;
            if (leftpcol == OBJECT_BLUE && rightpcol == OBJECT_RED)
                return BEACON_BLUE_RED;
            if (leftpcol == OBJECT_RED && rightpcol == OBJECT_BLUE)
                return BEACON_RED_BLUE;
            if (leftpcol == OBJECT_BLUE && rightpcol == OBJECT_BLUE)
                return BEACON_ALL_BLUE;
            if (leftpcol == OBJECT_RED && rightpcol == OBJECT_RED)
                return BEACON_NO_BLUE;
        }

        return NOT_VISIBLE;

    }//getBeaconConfig

    public static int isBlueOrRedPixel(int pixel) {
        int red = (pixel >> 16) & 0xff;
        int blue = pixel & 0xff;

        if (red == 0xff && blue == 0xff)
            return NOT_VISIBLE;
        if (red == 0 && blue == 0)
            return NOT_VISIBLE;
        if (red > blue)
            return OBJECT_RED;
        if (blue > red)
            return OBJECT_BLUE;
        return NOT_VISIBLE;
    }


//    public static int isBlueOrRed(Image img, VuforiaTrackableDefaultListener object, CameraCalibration camCal, VectorF sizeOfObject, int objectType) {
//
//        OpenGLMatrix pose = object.getPose();
//        if (pose != null && img != null && img.getPixels() != null) {
//
//            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(img.getPixels());
//
//            //turning the corner pixel coordinates into a proper bounding box
//           // Mat analyse = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);
//
//            Matrix34F rawPose = new Matrix34F();
//            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
//
//            rawPose.setData(poseData);
//
//            //calculating pixel coordinates of beacon corners
//            float[][] corners = new float[6][2];
//
//            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(sizeOfObject.get(0) / 2, sizeOfObject.get(1) / 2, sizeOfObject.get(2) / 2)).getData(); //upper left of beacon
//            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(-sizeOfObject.get(0) / 2, sizeOfObject.get(1) / 2, sizeOfObject.get(2) / 2)).getData(); //upper right of beacon
//            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(-sizeOfObject.get(0) / 2, -sizeOfObject.get(1) / 2, sizeOfObject.get(2) / 2)).getData(); //lower right of beacon
//            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(sizeOfObject.get(0) / 2, -sizeOfObject.get(1) / 2, -sizeOfObject.get(2) / 2)).getData(); //lower left of beacon
//            corners[4] = Tool.projectPoint(camCal, rawPose, new Vec3F(-sizeOfObject.get(0) / 2, sizeOfObject.get(1) / 2, -sizeOfObject.get(2) / 2)).getData(); //lower left of beacon
//            corners[5] = Tool.projectPoint(camCal, rawPose, new Vec3F(-sizeOfObject.get(0) / 2, -sizeOfObject.get(1) / 2, -sizeOfObject.get(2) / 2)).getData(); //lower left of beacon
//
//            double x = MathUtils.min(corners[0][0], corners[1][0], corners[2][0], corners[3][0], corners[4][0], corners[5][0]);
//            double y = MathUtils.min(corners[0][1], corners[1][1], corners[2][1], corners[3][1], corners[4][1], corners[5][1]);
//            double width = MathUtils.range(corners[0][0], corners[1][0], corners[2][0], corners[3][0], corners[4][0], corners[5][0]);
//            double height = MathUtils.range(corners[0][1], corners[1][1], corners[2][1], corners[3][1], corners[4][1], corners[5][1]);
//
//            x = Math.max(0, x);
//            y = Math.max(0, y);
//
//            width = (x + width > analyse.cols())? analyse.cols() - x : width;
//            height = (y + height > analyse.rows())? analyse.rows() - y : height;
//
//            Mat cropped = new Mat(analyse, new Rect((int) x, (int) y, (int) width, (int) height));
//
//            Mat maskBlue = applyMask(cropped, OTHER_BLUE_LOW, OTHER_BLUE_HIGH);
//
//            Mat maskRed = applyMask(cropped, OTHER_RED_LOW, OTHER_RED_HIGH);
//
//            if (Imgproc.moments(maskBlue).get_m00() > Imgproc.moments(maskRed).get_m00()) {
//                return OBJECT_BLUE;
//            } else {
//                return OBJECT_RED;
//            }//else
//
//        }//if
//
//        return NOT_VISIBLE;
//    }//isBlueOrRed

    @Nullable
    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    //this assumes the horizontal axis is the y-axis since the phone is vertical
    //robot angle is relative to "parallel with the beacon wall"
    public static VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF(
                (float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1) + offWall.get(1),
                (float) (-trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle)))
        );
    }

    public static VectorF navOffWall2(VectorF trans, double robotAngle, VectorF offWall){
        double theta = Math.toDegrees(Math.atan2(offWall.get(0), offWall.get(2)));

        return new VectorF(
                (float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1),
                (float) (-trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle)))
        );
    }
}
