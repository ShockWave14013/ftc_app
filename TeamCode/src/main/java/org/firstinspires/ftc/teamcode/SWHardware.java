package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.LSM6;
import org.firstinspires.ftc.teamcode.helpers.PID;

import static java.lang.Thread.sleep;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class SWHardware
{
    /* Public OpMode members. */
    public DcMotor  LF = null;
    public DcMotor  RF = null;
    public DcMotor  LB = null;
    public DcMotor  RB = null;
    //public Servo    Rot= null;
    public Servo    BallLifter= null;
    public DeviceInterfaceModule cdim = null;
    LSM6 gyroc, gyrot = null;
    public Servo Kicker = null;
    public DcMotor shootertilt = null;
    public DcMotor lefts = null;
    public DcMotor rights = null;
    public DcMotor convmotor = null;
    //public Servo convservo;


    // gyroc = on chasis
    //gyrot = on turret

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final float KICKER_PULL = 0.43F;
    public static final float KICKER_PUSH = 0.8F;

    public static final float CONVEYOR_STOP = 0.0F;
    public static final float CONVEYOR_RUN = 0.5F;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    long lastrunTime = 0;
    int prevleft=0, prevright=0;
    public int leftavg = 0, rightavg = 0;
    PID leftpid = new PID(0.00075,0.0001,0,false);
    PID rightpid = new PID(0.00075,0.0001,0,false);
    double targetspd = 700.0;
    public float stvpow = 0.7F;
    public int stvpos = -100;
    //public float leftspow =


    /* Constructor */
    public SWHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        lefts = hwMap.dcMotor.get("lefts");
        rights = hwMap.dcMotor.get("rights");
        lefts.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rights.setDirection(DcMotor.Direction.FORWARD);

        lefts.setPower(0);
        rights.setPower(0);

        lefts.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rights.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Motors
        LF = hwMap.dcMotor.get("L F");
        RF = hwMap.dcMotor.get("R F");

        LF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        RF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        LB = hwMap.dcMotor.get("L B");
        RB = hwMap.dcMotor.get("R B");

        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);

        // Set all motors to run with encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootertilt = hwMap.dcMotor.get("Shootertilt");
        shootertilt.setPower(0);
        shootertilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Rot = hwMap.servo.get("Rotator");

        //Rot.setPosition(0.5);
        convmotor = hwMap.dcMotor.get("Conv");
        convmotor.setDirection(DcMotor.Direction.REVERSE);
        shootertilt.setPower(0);
        shootertilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BallLifter = hwMap.servo.get("BallLifter");
        BallLifter.setPosition(0.72);

        Kicker = hwMap.servo.get("Kicker");
        Kicker.setPosition(KICKER_PULL);

        //convservo = hwMap.servo.get("Convservo");
        //convservo.setPosition(0.505F);

        cdim = hwMap.deviceInterfaceModule.get("dim");
        gyroc = hwMap.get(LSM6.class,"gyroc");
        //gyrot =hardwareMap.get(LSM6.class,"gyro2");
        //gyrot.setI2cAddr(I2cAddr.create8bit(0xD4));


        // Define and initialize ALL installed servos.
    }

    public void initShooter(double spd){
        lastrunTime = SystemClock.elapsedRealtime();
        prevleft = lefts.getCurrentPosition();
        prevright = rights.getCurrentPosition();
        targetspd = spd;
        leftpid = new PID(0.00075,0.0003,0,false);
        rightpid = new PID(0.00075,0.0003,0,false);
    }

    public double runShooter(){
        double dt = (SystemClock.elapsedRealtime() - lastrunTime)/1000.0;
        lastrunTime = SystemClock.elapsedRealtime();

        int leftdiff = lefts.getCurrentPosition()-prevleft;
        int rightdiff = rights.getCurrentPosition()-prevright;
        prevleft = lefts.getCurrentPosition();
        prevright = rights.getCurrentPosition();
        leftavg -= leftavg >> 3;
        leftavg += leftdiff;
        rightavg -= rightavg >> 3;
        rightavg += rightdiff;

        double leftspd = (leftavg >> 3) / dt;
        double rightspd = (rightavg >> 3) / dt;
        double leftpow = leftpid.run(targetspd - leftspd);
        double rightpow = rightpid.run(targetspd - rightspd);
        leftpow = Range.clip(leftpow, 0.01, 0.4);
        rightpow = Range.clip(rightpow, 0.01, 0.4);

        lefts.setPower(leftpow);
        rights.setPower(rightpow);
        return (leftspd+rightspd)/2;
    }

    public void stopShooter(){
        lefts.setPower(0.0F);
        rights.setPower(0.0F);
    }
    /*
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

