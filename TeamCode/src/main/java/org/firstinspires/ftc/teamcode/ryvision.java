/*
Copyright (c) 2016 Robert Atkinson

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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="Ry met vision", group ="Robot")
//@Disabled
public class ryvision extends LinearOpMode {

    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;

    SWHardware robot       = new SWHardware();

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcvR5Sn/////AAAAGQwS8PRgWkyFgpvvz2d3BaMngxk2ZmS2alish2+5HG1YD+3YTzOn/9gMWN5KtthsKuymriEd5CJCV2pS8Caf8IbGhmmiGkGzFkd+BklL11xPvHEVoN5NbPVd6SkJZxRxm78ncJoDQj/YrR8vLX1LqBaHBr4G5xs8fBs7dlbdEhBf+mt0E8Bf7GjKb7VPRgKi3V0aVES65/RsCNc+LEBofLVKx5NI85F/3UsBM8Mg85jqKm7CHDJt0ppyY03RZDyCIYcj68ZR5St5fSGBrvoiij/TG2+UxdW+ZE2+ka/5L6lC1JGNebHQo+H/NZ1hOQfC2J/f+u1CeEBqzmACucoHEIB+XivNfvy5MvxsjFmsR0kI";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

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

        waitForStart();

        robot.LF.setPower(-0.2F);
        robot.RF.setPower(-0.2F);
        robot.LB.setPower(-0.2F);
        robot.RB.setPower(-0.2F);

        while (opModeIsActive()) {

            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    float Xposisie = lastLocation.getTranslation().get(0);
                    if (Xposisie < (mmFTCFieldWidth/12)-(mmBotWidth/2)) {
                        robot.LF.setPower(0);
                        robot.RF.setPower(0);
                        robot.LB.setPower(0);
                        robot.RB.setPower(0);
                    }
                }
            }

            if (lastLocation != null) {

                telemetry.addData("Pos", format(lastLocation));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}
