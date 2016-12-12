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
import org.firstinspires.ftc.teamcode.helpers.LSM6;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class SWTD1 extends OpMode {

    SWHardware robot       = new SWHardware();

	double where = 0;

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

		//telemetry.addData("bwrd tgt pwr","backwards pwr: " + String.format("%.2f", backwards));
	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

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
