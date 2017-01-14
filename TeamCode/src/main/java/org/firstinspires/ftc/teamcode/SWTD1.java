/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.helpers.PID;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="ShockWave Test", group="Robot")
//@Disabled
public class SWTD1 extends OpMode {

	SWHardware robot = new SWHardware();
	//PID rotator = new PID(0.001,0.001,0.001,true);

	double where = 0;
	double ptgyro =0;
	boolean pdpadup = false;
	boolean pdpaddown = false;
	double BallLiftServoPos = 0.7;
	boolean pdpadleft = false;
	boolean pdpadright = false;
	/**
	 * Constructor
	 */
	public SWTD1() {

	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {

        robot.init(hardwareMap);

	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		//robot.Rot.setPosition((gamepad1.left_trigger-gamepad1.right_trigger)/2.0+0.5);
		telemetry.addData("GyroT", "gyrot:%f,ptgyro:%f", robot.gyrot.getHeading(),ptgyro-robot.gyrot.getHeading());
		if (robot.gyrot.getHeading() != ptgyro) {
			double tpidout = rotator.run(robot.gyrot.getHeading());
			RobotLog.ii("GyroT", "gyroh:%f, pidout:%f", robot.gyrot.getHeading(), tpidout);
			telemetry.addData("GyroT", "pidout:%f", tpidout);
			tpidout = Range.clip(tpidout, -0.5, 0.5);
			double tspd = (gamepad1.left_trigger - gamepad1.right_trigger) / 2.0;
			telemetry.addData("GyroT", "tspd:%f, pidout:%f", tspd, tpidout);
			RobotLog.ii("GyroT", "tspd:%f, pidout:%f", tspd, tpidout);
			tspd += tpidout;
			tspd = Range.clip(tspd, -0.5, 0.5);
			RobotLog.ii("GyroT", "tspd:%f", tspd);
			if (Double.isNaN(tspd))
				tspd = 0;
			robot.Rot.setPosition(tspd + 0.5);
			ptgyro = robot.gyrot.getHeading();
		}

		if (gamepad1.dpad_up && !pdpadup){
			BallLiftServoPos += 0.05;
		}
		if (gamepad1.dpad_down && !pdpaddown){
			BallLiftServoPos -= 0.05;
		}
		if (gamepad1.dpad_left && !pdpadleft){
			BallLiftServoPos = 0.7;
		}
		if (gamepad1.dpad_right && !pdpadright){
			BallLiftServoPos = 0.0;
		}

		robot.BallLifter.setPosition(Range.clip(BallLiftServoPos, 0.0, 1.0));
		pdpadup = gamepad1.dpad_up;
		pdpaddown = gamepad1.dpad_down;
		pdpadleft = gamepad1.dpad_left;
		pdpadright = gamepad1.dpad_right;
		telemetry.addData("BallLifter", "BallLifter:%f",  BallLiftServoPos);

        // note that if y equal -1 then joystick is pushed all of the way forward.
        float lefty = -gamepad1.left_stick_y;
        float righty = -gamepad1.right_stick_y;
		float rightx = -gamepad1.right_stick_x;

		//float backwards = -gamepad1.right_stick_x;
		float rv = 0;
		float lv = 0;
		float rb = 0;
		float lb = 0;
		float maxspeed = (float)0.78;
		//float balance = (float)0.5;
		double Ferror, Berror;

		// clip the right/left values so that the values never exceed +/- 1
		righty = Range.clip(righty, -maxspeed, maxspeed);
		lefty = Range.clip(lefty, -maxspeed, maxspeed);
		rightx = Range.clip(rightx, -maxspeed, maxspeed);

		if (rightx == 0 && righty == 0 && lefty == 0){
			where = robot.gyroc.getHeading();
		}
		else if (lefty == 0) {
			//forward backward and sideways if only rightstick is pressed
			if (gamepad1.y) {

			}
			else {
				rightx *= 0.5;
				righty *= 0.5;
			}
			Ferror = ((where - robot.gyroc.getHeading()) / 200);//0.09
			Berror = ((where - robot.gyroc.getHeading()) / 100);//-0.09

			rv += righty;
			lv += righty;
			rb += righty;
			lb += righty;

			rv += rightx;
			lv -= rightx;
			rb -= rightx;
			lb += rightx;

			rv += Ferror;
			lv -= Ferror;
			rb += Berror;
			lb -= Berror;

			DbgLog.msg("where:%s,gyroHeading:%s,fe:%s,be:%s", ((Double) where).toString(), ((Double) robot.gyroc.getHeading()).toString(), ((Double) Ferror).toString(), ((Double) Berror).toString());
		}
		else{
			//tank drive if right stick is pressed
			if (gamepad1.y) {

			} else {
				righty *= 0.39;
				lefty *= 0.39;
			}
			where = robot.gyroc.getHeading();

			rv += righty;
			lv += lefty;
			rb += righty;
			lb += lefty;
		}
		rv = Range.clip(rv, -maxspeed, maxspeed);
		lv = Range.clip(lv, -maxspeed, maxspeed);
		rb = Range.clip(rb, -maxspeed, maxspeed);
		lb = Range.clip(lb, -maxspeed, maxspeed);
		//DbgLog.msg("righty:%s,rightx:%s,px:%s,py:%s,balance:%s",((Float)righty).toString(),((Float)rightx).toString(),((Float)px).toString(),((Float)py).toString(),((Float)balance).toString());
		robot.RF.setPower(rv);
		robot.LF.setPower(lv);
		robot.RB.setPower(rb);
		robot.LB.setPower(lb);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

		telemetry.addData("Text", "*** Robot Data***");
       // telemetry.addData("servo", "servo:  " + String.format("%.2f", armPosition));
        //telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));
		telemetry.addData("right pwr",  "right  pwr: " + String.format("%.2f", righty));
		telemetry.addData(" right direction", "right direction: " + String.format("%.2f", rightx));
		telemetry.addData("left tgt pwr","left pwr: " + String.format("%.2f", lefty));
		telemetry.addData("gyro", "gyroc" + String.format("%.2f", where));
		robot.waitForTick(10);
		//telemetry.addData("bwrd tgt pwr","backwards pwr: " + String.format("%.2f", backwards));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {
		robot.Rot.setPosition(0.5);
	}

	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
