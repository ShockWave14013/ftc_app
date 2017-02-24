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

import android.os.SystemClock;

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
	double BallLiftServoPos = 0.72;
	boolean pdpadleft = false;
	boolean pdpadright = false;
	boolean kickerbut = false;
	float kickerservo = robot.KICKER_PULL;
	int kickerState = 0;
	long kickerTm = 0;

	float shup = 0;
	float shdown = 0;
	//int shootertilt = 0;
	public float buttprevup;
	public float buttnowup;
	public float buttprevdn;
	public float buttnowdn;
	public float shooterspeed;
	public boolean conva;
	public float convservo;
	public float SSValue = 675.0F;
	public boolean LTPresed = false;
	public boolean RTPessed = false;

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
		robot.initShooter(0.0F);
		kickerState = 0;
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {
		//robot.Rot.setPosition((gamepad1.left_trigger-gamepad1.right_trigger)/2.0+0.5);
		/*telemetry.addData("GyroT", "gyrot:%f,ptgyro:%f", robot.gyrot.getHeading(),ptgyro-robot.gyrot.getHeading());
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
		*/

//		if (gamepad1.x && !kickerbut){
//			kickerservo = 0.8;
//			kickerbut = true;
//		}
//		if (gamepad1.x && kickerbut == true ){
//			kickerservo = 0.43;
//			kickerbut = false;setTargetPosition(Range.clip(shootertilt, 0.0, 1.0));
//		}
		robot.runShooter();
//		if (gamepad2.y){
//			robot.shootertilt.setTargetPosition(robot.stvpos);
//			robot.shootertilt.setPower(robot.stvpow);
//		}

		if (gamepad2.left_bumper){

		}

//		if (gamepad2.a && !conva ){
//			if (convservo == 0.5F){
//				convservo = 0.0F;
//			}
//			else{
//				convservo = 0.5F;
//			}
//		}
//		conva = gamepad2.a;
//		robot.convservo.setPosition(Range.clip(convservo, 0.0, 1.0));
		if (gamepad2.a && !conva ){
			if (convservo == robot.CONVEYOR_STOP){
				convservo = robot.CONVEYOR_RUN;
			}
			else{
				convservo = robot.CONVEYOR_STOP;
			}
		}
		conva = gamepad2.a;
/*
		robot.convmotor.setPower(Range.clip(convservo, 0.0, 1.0));
		telemetry.addData("convservo", "Conveyer:%f", convservo);

		if (gamepad2.x)
			kickerservo = robot.KICKER_PUSH;
		else
			kickerservo = robot.KICKER_PULL;

		robot.Kicker.setPosition(Range.clip(kickerservo, 0.0, 1.0));
		telemetry.addData("kicker", "Kicker:%f",  kickerservo);
		telemetry.addData("kickerbutton", "Kickerbutton:%d",  kickerbut?1:0);
*/
		// State machine for kicker and conveyor
		if (gamepad2.x && !kickerbut && kickerState == 0)
			kickerState = 1;
		kickerbut = gamepad2.x;

		if (kickerState != 0)
			switch (kickerState) {
				case 1: // switch conveyer off and push kicker
					kickerservo = robot.KICKER_PUSH;
					convservo = robot.CONVEYOR_STOP;
					telemetry.addData("kicker", "conv off, kicker push");
					kickerState++;
					kickerTm = SystemClock.elapsedRealtime();
					break;
				case 2: // Wait to give servo time to push
					if (SystemClock.elapsedRealtime() - kickerTm < 500) {
						telemetry.addData("kicker", "pushing");
						break;
					}
					kickerState++;
					break;
				case 3: // pull back kicker
					kickerservo = robot.KICKER_PULL;
					telemetry.addData("kicker", "kicker pull");
					kickerState++;
					kickerTm = SystemClock.elapsedRealtime();
					break;
				case 4: // Wait for servo to pull
					if (SystemClock.elapsedRealtime() - kickerTm < 500) {
						telemetry.addData("kicker", "pulling");
						break;
					}
					kickerState++;
					break;
				case 5: // Start conveyor again
					convservo = robot.CONVEYOR_RUN;
					telemetry.addData("kicker", "starting conveyor");
					kickerState = 0;
					break;
			}
		robot.Kicker.setPosition(Range.clip(kickerservo, 0.0, 1.0));
		robot.convmotor.setPower(Range.clip(convservo, 0.0, 1.0));
		telemetry.addData("convservo", "Conveyer:%f", convservo);

		buttnowup = -gamepad2.right_stick_y;
		buttnowdn = gamepad2.right_stick_y;


			// Use gamepad buttons to move thearm up (Y) and down (A)

		if (-gamepad2.right_stick_y != 0 && buttprevup == 0 && buttnowup >= 0 ) {
				//shooterspeed = 0.2F;
			robot.initShooter(SSValue);
		} else if (gamepad2.right_stick_y != 0 && buttnowdn >= 0 && buttprevdn == 0)
			//shooterspeed = 0.0F;
			robot.initShooter(0.0F);
		else {

		}

		LTPresed = gamepad2.left_bumper;
		RTPessed = gamepad2.right_bumper;

		if (gamepad2.left_bumper == true && LTPresed == false){
			SSValue -= 10F;
			LTPresed = true;
			robot.initShooter(SSValue);
		}

		if (gamepad2.right_bumper == true && RTPessed == false ){
			SSValue += 10F;
			RTPessed = true;
			robot.initShooter(SSValue);
		}

		telemetry.addData("Shooter Value: %d", SSValue);
		telemetry.update();

//		robot.lefts.setPower(shooterspeed);
//		robot.rights.setPower(shooterspeed);

		buttprevdn = buttnowdn;
		buttprevup = buttnowup;

		//telemetry.addData("shooterspeed", "%.2f" , shooterspeed );

		if (gamepad2.dpad_up && !pdpadup){
			BallLiftServoPos += 0.05;
		}
		if (gamepad2.dpad_down && !pdpaddown){
			BallLiftServoPos -= 0.05;
		}
		if (gamepad2.dpad_left && !pdpadleft){
			BallLiftServoPos = 0.72;
		}
		if (gamepad2.dpad_right && !pdpadright){
			BallLiftServoPos = 0.0;
		}

		robot.BallLifter.setPosition(Range.clip(BallLiftServoPos, 0.0, 1.0));
		pdpadup = gamepad2.dpad_up;
		pdpaddown = gamepad2.dpad_down;
		pdpadleft = gamepad2.dpad_left;
		pdpadright = gamepad2.dpad_right;
		telemetry.addData("BallLifter", "BallLifter:%f",  BallLiftServoPos);

//		if (gamepad2.left_stick_y > 0 && shup == 0){
//			shootertilt += 10;
//		}
//		if (gamepad2.left_stick_y < 0 && shdown == 0){
//			shootertilt -= 10;
//		}
//
//		robot.shootertilt.setPower(0.7F);
//
//		robot.shootertilt.setTargetPosition(shootertilt);
//		shup = -gamepad2.left_stick_y;
//		shdown = gamepad2.left_stick_y;
//		telemetry.addData("shootertilt", "shootertilt:%d",  shootertilt);

		/**************************************************************************************
		 * Robot movement elow
		 */
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
			if (gamepad1.x) {

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
			if (gamepad1.x) {

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
		 * a legacy NXT-compatible motor controller, ythen the getPower() method
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
		//robot.Rot.setPosition(0.5);
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
