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

import android.os.SystemClock;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.helpers.PID;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autoshoot", group="Robot")
//@Disabled
public class Autoshoot extends LinearOpMode {

    /* Declare OpMode members. */
    SWHardware robot = new SWHardware();

    @Override
    public void runOpMode() {
        int prevleft = 0, prevright = 0;
        int leftdiff = 0, rightdiff = 0;
        int prevldiff = 0, prevrdiff = 0;
        int leftavg = 0, rightavg = 0;
        long lastrunTime = 0;
        PID leftpid = new PID(0.00075,0.0001,0,false);
        PID rightpid = new PID(0.00075,0.0001,0,false);
        double targetspd = 675.0;
        long waitStartTime = 0;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("-", "startin reset encoder");
        telemetry.update();








        // Laat motor onder vasdruk
//        robot.shootertilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.shootertilt.setPower(0.3F);
//        sleep(1000);
//        robot.shootertilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.shootertilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("-", "going to tiltshoot");
//        telemetry.update();
//
//        robot.shootertilt.setTargetPosition(robot.stvpos);
//        robot.shootertilt.setPower(robot.stvpow);
//        telemetry.addData("-", "shootertilt");
//        telemetry.update();
//        sleep(500);

        robot.initShooter(targetspd);
        sleep(50);
        for (int i = 0; opModeIsActive(); i++){
//            double dt = (SystemClock.elapsedRealtime() - lastrunTime)/1000.0;
//            lastrunTime = SystemClock.elapsedRealtime();
//
//            leftdiff = robot.lefts.getCurrentPosition()-prevleft;
//            rightdiff = robot.rights.getCurrentPosition()-prevright;
//            prevleft = robot.lefts.getCurrentPosition();
//            prevright = robot.rights.getCurrentPosition();
//            leftavg -= leftavg >> 3;
//            leftavg += leftdiff;
//            rightavg -= rightavg >> 3;
//            rightavg += rightdiff;
//
//            double leftspd = (leftavg >> 3) / dt;
//            double rightspd = (rightavg >> 3) / dt;
//            double leftpow = leftpid.run(targetspd - leftspd);
//            double rightpow = rightpid.run(targetspd - rightspd);
//            leftpow = Range.clip(leftpow, 0.01, 0.4);
//            rightpow = Range.clip(rightpow, 0.01, 0.4);
//
//            robot.lefts.setPower(leftpow);
//            robot.rights.setPower(rightpow);
            double shootspeed = robot.runShooter();

            telemetry.addData("x", "left :%d, %.3f", robot.leftavg >> 3, shootspeed);
            telemetry.addData("y", "right:%d, %.3f", robot.rightavg >> 3, shootspeed);
            //telemetry.addData("z", "tilt:%d", robot.shootertilt.getCurrentPosition());
            DbgLog.msg("leftavg:%d, rightavg%d",leftavg , rightavg);
            telemetry.update();
            if ((shootspeed > targetspd * 0.9) && (shootspeed > targetspd * 0.9))
                break;
            sleep(50);
        }

        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            double shootspeed = robot.runShooter();

            telemetry.addData("x", "left :%d, %.3f", robot.leftavg >> 3, shootspeed);
            telemetry.addData("y", "right:%d, %.3f", robot.rightavg >> 3, shootspeed);
            //telemetry.addData("z", "tilt:%d", robot.shootertilt.getCurrentPosition());
            DbgLog.msg("leftavg:%d, rightavg%d",leftavg , rightavg);
            telemetry.update();
            sleep(50);
        }

        robot.BallLifter.setPosition(0.0F);

        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            robot.runShooter();
            sleep(50);
        }
        robot.BallLifter.setPosition(0.72F);
        //sleep(1000);
        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            robot.runShooter();
            sleep(50);
        }
        robot.Kicker.setPosition(0.8F);
        //sleep(100);
        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            robot.runShooter();
            sleep(50);
        }
        robot.Kicker.setPosition(0.43F);


        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            double shootspeed = robot.runShooter();

            telemetry.addData("x", "left :%d, %.3f", robot.leftavg >> 3, shootspeed);
            telemetry.addData("y", "right:%d, %.3f", robot.rightavg >> 3, shootspeed);
            //telemetry.addData("z", "tilt:%d", robot.shootertilt.getCurrentPosition());
            DbgLog.msg("leftavg:%d, rightavg%d",leftavg , rightavg);
            telemetry.update();
            sleep(50);
        }
        robot.BallLifter.setPosition(0.0F);

        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            robot.runShooter();
            sleep(50);
        }
        robot.BallLifter.setPosition(0.72F);
        //sleep(1000);
        waitStartTime = SystemClock.elapsedRealtime();
        while (opModeIsActive() && (SystemClock.elapsedRealtime()- waitStartTime)<1000){
            robot.runShooter();
            sleep(50);
        }



        // wag bietjie
        // druk 2de bal in
        // wag
        // trek terug
        // pid skop wiele
        // skop 2de bal

        sleep(2000);

    }
}