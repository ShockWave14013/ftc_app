/*
Copyright (c) 2016 Naam en Van.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Autonomous program for Shockie for Velocity Vortex.
 * Strategy: XXX
 * - Option 1: XXX
 * - - Drive to first beacon and press the correct button
 * - - - Quickly drive to close to the beacon.
 * - - Drive to second beacon and press the correct button
 * - - Finished
 * - Option 2: XXX
 * - - Shoot particles into centre vortex
 * - - Finished
 *
 * Config options:
 * - Alliance - red or blue
 * - Delay before starting
 * - Different starting positions?
 * - Try to score particles in centre vortex?
 * - Do both beacons?
 *
 * Keep in mind:
 * - Vuforia needs to have the whole target picture in the camera view to be able to
 *   lock on. That seems to be about 1.5 tile.
 *
 *   https://github.com/cheer4ftc/OpModeConfig
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.helpers.FtcConfig;
import org.firstinspires.ftc.teamcode.helpers.ImgProc;
import org.firstinspires.ftc.teamcode.helpers.VortexUtils;

import static org.firstinspires.ftc.teamcode.helpers.VortexUtils.BEACON_BLUE_RED;
import static org.firstinspires.ftc.teamcode.helpers.VortexUtils.BEACON_RED_BLUE;

@Autonomous(name="Ry met vision", group ="Robot")
//@Disabled
public class ryvision extends LinearOpMode {

    public static final String TAG = "SW Autonomous testing";

    OpenGLMatrix lastLocation = null;

    // Robot specific info - move into Hardware?
    final static double ticspmm = 10000.0/2820.0;
    float maxspeed = (float)0.78;   // We are using NeverRest motors

    SWHardware robot = new SWHardware();

    VuforiaLocalizer vuforia;

    VuforiaTrackableDefaultListener first_beacon_listener = null;

    float Perfectplace = 1500;


    class mecGR {
        double yspd, xspd; // x and y componets of speed
        double where; // Orientation of robot at the start of a move, fi. mecGR()
        double finorient; // Final orientation
        double ticsF, ticsB; // ticks needed for Front and Back wheels

        // dir is looking down at robot, x axis going through its sides and y axis through its
        // front and back. 0 is on the x axis to the right and moving CCW.
        public void init(int afst, float spd, double dir, int orientation) {
            spd = Math.abs(spd);
            //where = robot.gyroc.getHeading() + orientation;
            where = robot.gyroc.getHeading();
            finorient = orientation;

            robot.RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // XXX - Compensation for different slippage of wheels in X and Y directions.
            yspd = spd * Math.sin(Math.toRadians(dir)) * 0.91; // 0.91
            xspd = spd * Math.cos(Math.toRadians(dir)) * 1.28; // 1.33
            int tics = (int) (afst * ticspmm);
            ticsF = tics * ((yspd - xspd) / spd);
            ticsB = tics * ((yspd + xspd) / spd);

            DbgLog.msg("afst:%d,spd:%.2f,dir:%.2f", afst, spd, dir);
            DbgLog.msg("xspd:%.3f,yspd:%.3f,ticsF:%.3f,ticsB:%.3f,RB.pos:%d,RF.pos:%d", xspd, yspd, ticsF, ticsB,
                    robot.RB.getCurrentPosition(), robot.RF.getCurrentPosition());
        }

        public boolean busy() {
            double rf, lf, rb, lb;
            double Ferror; // Front error correction
            double Berror; // Back error correction. Different because of weight distribution
            double currorient = 0.0; // Orientation to aim for now
            if (Math.abs(ticsF) > Math.abs(ticsB)) {
                currorient =  finorient * 1.2 * Math.abs(robot.RF.getCurrentPosition()) / Math.abs(ticsF);

                if ((Math.abs(robot.RF.getCurrentPosition()) > Math.abs(ticsF))) {
                    DbgLog.msg("Using RF");
                    return false;
                }
            } else {
                currorient =  finorient * 1.2 * Math.abs(robot.RB.getCurrentPosition()) / Math.abs(ticsB);
                if (Math.abs(robot.RB.getCurrentPosition()) > Math.abs(ticsB)) {
                    DbgLog.msg("Using RB");
                    return false;
                }
            }
            currorient = Range.clip(currorient, -Math.abs(finorient), Math.abs(finorient));

            // XXX Weight compensation 200 ... 100?
            Ferror = currorient + ((where - robot.gyroc.getHeading()) / 100); // 200
            Berror = currorient + ((where - robot.gyroc.getHeading()) / 100);

            DbgLog.msg("xspd:%.3f,yspd:%.3f,ticsF:%.3f,ticsB:%.3f,RB.pos:%d,RF.pos:%d", xspd, yspd, ticsF, ticsB,
                    robot.RB.getCurrentPosition(), robot.RF.getCurrentPosition());

            rf = yspd;
            lf = yspd;
            rb = yspd;
            lb = yspd;

            rf -= xspd;
            lf += xspd;
            rb += xspd;
            lb -= xspd;

            rf += Ferror;
            lf -= Ferror;
            rb += Berror;
            lb -= Berror;

            // scale the values so that none is bigger than 1
            double scale = 1.0;
            scale = Math.max(scale, Math.abs(rf));
            scale = Math.max(scale, Math.abs(lf));
            scale = Math.max(scale, Math.abs(rb));
            scale = Math.max(scale,Math.abs(lb));
            if (scale != 1.0) {
                rf = rf / scale;
                lf = lf / scale;
                rb = rb / scale;
                lb = lb / scale;
            }

            //rf = Range.clip(rf, -maxspeed, maxspeed);
            //lf = Range.clip(lf, -maxspeed, maxspeed);
            //rb = Range.clip(rb, -maxspeed, maxspeed);
            //lb = Range.clip(lb, -maxspeed, maxspeed);

            robot.RF.setPower(rf);
            robot.LF.setPower(lf);
            robot.RB.setPower(rb);
            robot.LB.setPower(lb);
            return true;
        }

        public void stop() {
            robot.RF.setPower(0);
            robot.LF.setPower(0);
            robot.RB.setPower(0);
            robot.LB.setPower(0);
        }

        public void all(int afst, float spd, double dir, int orientation) {
            init(afst, spd, dir, orientation);
            while (opModeIsActive()) {
                if (busy() == false)
                    break;
            }
            stop();
        }

        public void turn(float spd, int degrees){
            where = robot.gyroc.getHeading() + degrees;

            double rf=0, lf=0, rb=0, lb=0;

            double Ferror; // Front error correction
            double Berror; // Back error correction. Different because of weight distribution

            while(opModeIsActive() && Math.abs(where - robot.gyroc.getHeading()) > 3 ) {
                Ferror = ((where - robot.gyroc.getHeading()) / 100); // 200
                Berror = ((where - robot.gyroc.getHeading()) / 100);

                rf += Ferror * spd;
                lf -= Ferror * spd;
                rb += Berror * spd;
                lb -= Berror * spd;

                robot.RF.setPower(rf);
                robot.LF.setPower(lf);
                robot.RB.setPower(rb);
                robot.LB.setPower(lb);
            }

            stop();

        }
    }

    @Override public void runOpMode() {

        float Xposisie, Yposisie;
        float helling;

        robot.init(hardwareMap);
        telemetry.addData(">", "Busy initializing Vuforia");
        telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcvR5Sn/////AAAAGQwS8PRgWkyFgpvvz2d3BaMngxk2ZmS2alish2+5HG1YD+3YTzOn/9gMWN5KtthsKuymriEd5CJCV2pS8Caf8IbGhmmiGkGzFkd+BklL11xPvHEVoN5NbPVd6SkJZxRxm78ncJoDQj/YrR8vLX1LqBaHBr4G5xs8fBs7dlbdEhBf+mt0E8Bf7GjKb7VPRgKi3V0aVES65/RsCNc+LEBofLVKx5NI85F/3UsBM8Mg85jqKm7CHDJt0ppyY03RZDyCIYcj68ZR5St5fSGBrvoiij/TG2+UxdW+ZE2+ka/5L6lC1JGNebHQo+H/NZ1hOQfC2J/f+u1CeEBqzmACucoHEIB+XivNfvy5MvxsjFmsR0kI";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        VuforiaTrackables FTC_comp = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTools = FTC_comp.get(1);
        redTools.setName("Tools");

        VuforiaTrackable blueWheels  = FTC_comp.get(0);
        blueWheels.setName("Wheels");

        VuforiaTrackable redGears = FTC_comp.get(3);
        redGears.setName("Gears");

        VuforiaTrackable blueLegos  = FTC_comp.get(2);
        blueLegos.setName("Legos");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(FTC_comp);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, mmFTCFieldWidth/4, 0)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTools.setLocation(redTargetLocationOnField);
        RobotLog.ii(TAG, "Tools=%s", format(redTargetLocationOnField));

        OpenGLMatrix blueTargetLocationOnField1 = OpenGLMatrix

                .translation(mmFTCFieldWidth/12, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueWheels.setLocation(blueTargetLocationOnField1);
        RobotLog.ii(TAG, "Wheels=%s", format(blueTargetLocationOnField1));

        OpenGLMatrix blueTargetLocationOnField2 = OpenGLMatrix

                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueLegos.setLocation(blueTargetLocationOnField2);
        RobotLog.ii(TAG, "Legos=%s", format(blueTargetLocationOnField2));

        OpenGLMatrix redTargetLocationOnField2 = OpenGLMatrix

                .translation(-mmFTCFieldWidth/2, -mmFTCFieldWidth/12, 0)
                .multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redGears.setLocation(redTargetLocationOnField2);
        RobotLog.ii(TAG, "Gears=%s", format(redTargetLocationOnField2));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,mmBotWidth/2,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 90, 90, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)redTools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueLegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        /** Start tracking the data sets we care about. */
        FTC_comp.activate();

        FtcConfig ftcConfig = new FtcConfig();
        telemetry.addData("-","world");
        telemetry.update();

        ftcConfig.init(hardwareMap.appContext, this);
        // waitOneFullHardwareCycle();
        sleep(50); // temporary fix to waitXXXHardwareCycle bug?

        telemetry.addData("-","hello");
        telemetry.update();
        // init_loop type functionality here
        while (!opModeIsActive() ) {
            ftcConfig.init_loop(hardwareMap.appContext, this);
            // waitOneFullHardwareCycle();
            sleep(50); // temporary fix to waitXXXHardwareCycle bug?
            telemetry.addData("-","rainbows");
            telemetry.update();
        }

        telemetry.addData("-","notOpMode");
        telemetry.update();

        waitForStart();

        first_beacon_listener = (VuforiaTrackableDefaultListener) blueWheels.getListener();

        telemetry.clearAll();
        telemetry.addData("A","Running the LinearOpMode now");
        telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
        telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
        telemetry.addData("AutonType", ftcConfig.param.autonType);

        mecGR drive = new mecGR();
        //drive.turn(0.1F,10);
        //sleep(30000);
        // We need to end up a bit more than a tile away from the target, otherwise it does not fit in the camera view
        drive.all(1000,0.2F, 270,0);
        drive.init(900,0.1F,270, 0);
        boolean needToDrive = true;
        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
            }

            if (lastLocation != null) {

                telemetry.addData("Pos", format(lastLocation));
                Xposisie = lastLocation.getTranslation().get(0);
                if (Xposisie < (mmFTCFieldWidth/12) + 55) { // 60mm delay
                    drive.stop();
                    needToDrive = false;
                    break;
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            if (needToDrive == true && drive.busy() == false) {
                telemetry.addData(">", "Missed target");
                drive.stop();
                break;
            }
            telemetry.update();
        }
        // Compensate if we did overshoot a little
        Xposisie = lastLocation.getTranslation().get(0);
        if(Xposisie < (mmFTCFieldWidth/12) - 55){
            drive.all(Math.round((mmFTCFieldWidth/12) - Xposisie ) + 55, 0.1F,90, 0);
        }
        // We should now be alligned with the target.
        // Get distance to where the robot can determine the beacon colour / orientation
        Yposisie = lastLocation.getTranslation().get(1);
        telemetry.addData("Yposisie: %.2f", Yposisie);
        helling = Yposisie - 340;
        float Perfectplace1 = Perfectplace - helling;

        drive.all(Math.round(Perfectplace1), 0.2F, 180, 0);
        for (VuforiaTrackable trackable : allTrackables) {

            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;

            }
        }
        Yposisie = lastLocation.getTranslation().get(1);
        telemetry.addData("Yposisie nuut",Yposisie);
        int config = VortexUtils.NOT_VISIBLE;
        sleep(2000); // XXX how long is really needed

        try {
            //telemetry.addData("-", "trying");
            //telemetry.update();
            DbgLog.msg("calling wait for");
            config = VortexUtils.getBeaconConfig(
                    ImgProc.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),
                    first_beacon_listener, vuforia.getCameraCalibration());
            telemetry.addData("Beacon", config);
            telemetry.update();
            Log.i(TAG, "runOp: " + config);
        } catch (Exception e) {
            telemetry.addData("Beacon", "could not not be found");
        }
        telemetry.update();
        //sleep(2000);

        //verskillende opsies vir die beacon
        drive.all(190,0.1F, 180,0);
        double GyroB = robot.gyroc.getHeading();
        double GyroE;
        if (config == 2){
            //drive.all(20,0.1F,180,20);
            drive.turn(0.07F,10);
            drive.all(30,0.1F,180, 0);
            sleep(750);
            //GyroE = robot.gyroc.getHeading();
            //telemetry.addData("Gyrowaarde", GyroE - GyroB);
            telemetry.addData("Config2", "1stturn  + sleep finish");
            telemetry.update();

            drive.all(50,0.1F,0,0);
            //drive.all(20,0.1F,180, -20);
        }

        if (config == 1){
            //drive.all(70,0.1F, 180,-10);
            drive.turn(0.07F,-10);
            drive.all(30,0.1F,180, 0);
            sleep(750);
            //GyroE = robot.gyroc.getHeading();
            //telemetry.addData("Gyrowaarde", GyroE - GyroB);
            telemetry.addData("Config1", "1stturn  + sleep finish");
            telemetry.update();
            drive.all(50, 0.1F, 0,0);
            //drive.all(20,0.1F,180, 20);
        }
        sleep(750);
        GyroE = robot.gyroc.getHeading();

        drive.turn(0.1F, (int)Math.round(GyroB - GyroE));

//        DbgLog.msg("After sleep 5000")

        //agteruit ry
        drive.all(700,0.1F,0, 0);
        sleep(1000);
        for (VuforiaTrackable trackable : allTrackables) {

            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;

            }
        }

        float zangle = Orientation.getOrientation(lastLocation,AxesReference.EXTRINSIC, AxesOrder.XYZ,AngleUnit.DEGREES).thirdAngle;
        DbgLog.msg("Z Angle: %f" , zangle);

        drive.turn(0.1F, Math.round(zangle) * -1);

        sleep(1000);

        drive.all(1100,0.2F,270,0);
        first_beacon_listener = (VuforiaTrackableDefaultListener) blueLegos.getListener();
        sleep(1000);
        drive.init(230,0.1F,270, 0);
        needToDrive = true;
        while (opModeIsActive()) {
            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                }
            }

            if (lastLocation != null) {

                telemetry.addData("Pos", format(lastLocation));
                Xposisie = lastLocation.getTranslation().get(0);
                if (Xposisie < (mmFTCFieldWidth/12*-3) + 25) { // 60mm delay
                    drive.stop();
                    needToDrive = false;
                    break;
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            if (needToDrive == true && drive.busy() == false) {
                telemetry.addData(">", "Missed target");
                drive.stop();
                break;
            }
            telemetry.update();
        }
        Yposisie = lastLocation.getTranslation().get(1);
        telemetry.addData("Yposisie: %.2f", Yposisie);
        helling = Yposisie - 340;
        float Perfectplace2 = Perfectplace - helling;

        drive.all(Math.round(Perfectplace2), 0.2F, 180, 0);
        sleep(5000);
        for (VuforiaTrackable trackable : allTrackables) {

            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;

            }
        }
        Yposisie = lastLocation.getTranslation().get(1);
        telemetry.addData("Yposisie nuut",Yposisie);
        config = VortexUtils.NOT_VISIBLE;
        sleep(2000); // XXX how long is really needed

        try {
            //telemetry.addData("-", "trying");
            //telemetry.update();
            DbgLog.msg("calling wait for");
            config = VortexUtils.getBeaconConfig(
                    ImgProc.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),
                    first_beacon_listener, vuforia.getCameraCalibration());
            telemetry.addData("Beacon", config);
            telemetry.update();
            Log.i(TAG, "runOp: " + config);
        } catch (Exception e) {
            telemetry.addData("Beacon", "could not not be found");
        }
        telemetry.update();
        //sleep(2000);

        //verskillende opsies vir die beacon
        drive.all(175,0.1F, 180,0);
        GyroB = robot.gyroc.getHeading();
        if (config == 2){
            //drive.all(20,0.1F,180,20);
            drive.turn(0.07F,10);
            drive.all(30,0.1F,180, 0);
            sleep(750);
            //GyroE = robot.gyroc.getHeading();
            //telemetry.addData("Gyrowaarde", GyroE - GyroB);
            telemetry.addData("Config2", "1stturn  + sleep finish");
            telemetry.update();

            drive.all(50,0.1F,0,0);
            //drive.all(20,0.1F,180, -20);
        }

        if (config == 1){
            //drive.all(70,0.1F, 180,-10);
            drive.turn(0.07F,-10);
            drive.all(30,0.1F,180, 0);
            sleep(750);
            //GyroE = robot.gyroc.getHeading();
            //telemetry.addData("Gyrowaarde", GyroE - GyroB);
            telemetry.addData("Config1", "1stturn  + sleep finish");
            telemetry.update();
            drive.all(50, 0.1F, 0,0);
            //drive.all(20,0.1F,180, 20);
        }
        // Compensate if we did overshoot a little
//        Xposisie = lastLocation.getTranslation().get(0);
//        if(Xposisie < (mmFTCFieldWidth/12) - 60){
//            drive.all(Math.round((mmFTCFieldWidth/12) - Xposisie ) + 60, 0.1F,90, 0);
//        }

    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
