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
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.helpers.FtcConfig;
import org.firstinspires.ftc.teamcode.helpers.ImgProc;
import org.firstinspires.ftc.teamcode.helpers.VortexUtils;


import java.util.ArrayList;
import java.util.List;

import static com.vuforia.PIXEL_FORMAT.RGBA8888;

//import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Kyk vir beacon kleur", group ="Robot")
//@Disabled
public class testbeacon extends LinearOpMode {

    public static final String TAG = "SW Autonomous testing";

    OpenGLMatrix lastLocation = null;

    // Robot specific info - move into Hardware?
    final static double ticspmm = 10000.0/2820.0;
    float maxspeed = (float)0.78;   // We are using NeverRest motors

   // SWHardware robot = new SWHardware();

    VuforiaLocalizer vuforia;


//    class mecGR {
//        double yspd, xspd; // x and y componets of speed
//        double where; // Orientation of robot at the start of a move, fi. mecGR()
//        double ticsF, ticsB; // ticks needed for Front and Back wheels
//
//        // dir is looking down at robot, x axis going through its sides and y axis through its
//        // front and back. 0 is on the x axis to the right and moving CCW.
//        public void init(int afst, float spd, double dir, int orientation) {
//            spd = Math.abs(spd);
//            where = robot.gyroc.getHeading() + orientation;
//
//            robot.RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            // XXX - Compensation for different slippage of wheels in X and Y directions.
//            yspd = spd * Math.sin(Math.toRadians(dir)) * 0.91; // 0.91
//            xspd = spd * Math.cos(Math.toRadians(dir)) * 1.28; // 1.33
//            int tics = (int) (afst * ticspmm);
//            ticsF = tics * ((yspd - xspd) / spd);
//            ticsB = tics * ((yspd + xspd) / spd);
//
//            DbgLog.msg("afst:%d,spd:%.2f,dir:%.2f", afst, spd, dir);
//            DbgLog.msg("xspd:%.3f,yspd:%.3f,ticsF:%.3f,ticsB:%.3f,RB.pos:%d,RF.pos:%d", xspd, yspd, ticsF, ticsB,
//                    robot.RB.getCurrentPosition(), robot.RF.getCurrentPosition());
//        }
//
//        public boolean busy() {
//            double rf, lf, rb, lb;
//            double Ferror; // Front error correction
//            double Berror; // Back error correction. Different because of weight distribution
//
//            if (Math.abs(ticsF) > Math.abs(ticsB)) {
//                    if ((Math.abs(robot.RF.getCurrentPosition()) > Math.abs(ticsF))) {
//                        DbgLog.msg("Using RF");
//                        return false;
//                    }
//                } else {
//                    if (Math.abs(robot.RB.getCurrentPosition()) > Math.abs(ticsB)) {
//                        DbgLog.msg("Using RB");
//                        return false;
//                    }
//                }
//
//            // XXX Weight compensation 200 ... 100?
//            Ferror = ((where - robot.gyroc.getHeading()) / 100); // 200
//            Berror = ((where - robot.gyroc.getHeading()) / 100);
//
//            DbgLog.msg("xspd:%.3f,yspd:%.3f,ticsF:%.3f,ticsB:%.3f,RB.pos:%d,RF.pos:%d", xspd, yspd, ticsF, ticsB,
//                    robot.RB.getCurrentPosition(), robot.RF.getCurrentPosition());
//
//            rf = yspd;
//            lf = yspd;
//            rb = yspd;
//            lb = yspd;
//
//            rf -= xspd;
//            lf += xspd;
//            rb += xspd;
//            lb -= xspd;
//
//            rf += Ferror;
//            lf -= Ferror;
//            rb += Berror;
//            lb -= Berror;
//
//            // scale the values so that none is bigger than 1
//            double scale = 1.0;
//            scale = Math.max(scale, Math.abs(rf));
//            scale = Math.max(scale, Math.abs(lf));
//            scale = Math.max(scale, Math.abs(rb));
//            scale = Math.max(scale,Math.abs(lb));
//            if (scale != 1.0) {
//                rf = rf / scale;
//                lf = lf / scale;
//                rb = rb / scale;
//                lb = lb / scale;
//            }
//
//            //rf = Range.clip(rf, -maxspeed, maxspeed);
//            //lf = Range.clip(lf, -maxspeed, maxspeed);
//            //rb = Range.clip(rb, -maxspeed, maxspeed);
//            //lb = Range.clip(lb, -maxspeed, maxspeed);
//
//            robot.RF.setPower(rf);
//            robot.LF.setPower(lf);
//            robot.RB.setPower(rb);
//            robot.LB.setPower(lb);
//            return true;
//        }
//
//        public void stop() {
//            robot.RF.setPower(0);
//            robot.LF.setPower(0);
//            robot.RB.setPower(0);
//            robot.LB.setPower(0);
//        }img
//
//        public void all(int afst, float spd, double dir, int orientation) {
//            init(afst, spd, dir, orientation);
//            while (opModeIsActive()) {
//                if (busy() == false)
//                    break;
//            }
//            stop();
//        }
//    }

    @Override public void runOpMode() {

        //robot.init(hardwareMap);
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
//
        FtcConfig ftcConfig = new FtcConfig();
//        telemetry.addData("-","world");
//        telemetry.update();
//
//        ftcConfig.init(hardwareMap.appContext, this);
//        // waitOneFullHardwareCycle();
//        sleep(50); // temporary fix to waitXXXHardwareCycle bug?
//
//        telemetry.addData("-","hello");
//        telemetry.update();
//        // init_loop type functionality here
//        while (!opModeIsActive() ) {
//            ftcConfig.init_loop(hardwareMap.appContext, this);
//            // waitOneFullHardwareCycle();
//            sleep(50); // temporary fix to waitXXXHardwareCycle bug?
//            telemetry.addData("-","rainbows");
//            telemetry.update();
//        }
//
//        telemetry.addData("-","notOpMode");
//        telemetry.update();

        waitForStart();

        int start_fast_distance;
        int start_fast_dir;
        float start_fast_power;
        int first_beacon_max_distance;
        int first_beacon_dir;
        float first_beacon_power;
        VuforiaTrackableDefaultListener first_beacon_listener = null;

        telemetry.clearAll();
        telemetry.addData("A","Running the LinearOpMode now");
        telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
        telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
        telemetry.addData("AutonType", ftcConfig.param.autonType);
        telemetry.update();

        ftcConfig.param.colorIsRed = false;

        if (ftcConfig.param.colorIsRed) {
            // We are red

        } else {
            // We are blue

            start_fast_distance = 900;
            start_fast_dir = 70;
            start_fast_power = maxspeed;
            first_beacon_max_distance = 400;
            first_beacon_dir = 0;
            first_beacon_power = 0.2F;

            first_beacon_listener = (VuforiaTrackableDefaultListener) blueWheels.getListener();
        }

        //mecGR drive = new mecGR();
        // We need to end up a bit more than a tile away from the target, otherwise it does not fit in the camera view
        //drive.init(1900,0.2F,360-25, 0);
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
                float Xposisie = lastLocation.getTranslation().get(0);
                if (Xposisie < (mmFTCFieldWidth/12) + 60) { // 60mm delay
                    //drive.stop();
                    needToDrive = false;
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            if (needToDrive == true /*&& drive.busy() == false*/) {
                telemetry.addData(">", "Missed target");
                //drive.stop();
                break;
            }


            int config = VortexUtils.NOT_VISIBLE;
            try {
                //telemetry.addData("-", "trying");
                //telemetry.update();
                DbgLog.msg("calling wait for");
                config = VortexUtils.getBeaconConfig(
                        ImgProc.getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565),
                        first_beacon_listener, vuforia.getCameraCalibration());
                telemetry.addData("Beacon", config);
                //telemetry.update();
                Log.i(TAG, "runOp: " + config);
            } catch (Exception e) {
                telemetry.addData("Beacon", "could not not be found");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
