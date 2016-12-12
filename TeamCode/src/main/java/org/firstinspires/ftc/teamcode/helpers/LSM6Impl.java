/*
 * Library for use with the LSM6DS33 3D Accelerometer and Gyro as used on the Polulo #2736 carrier.
 *
 * After initialization, the driver will keep reading the device with a callback and integrate
 * the Z axis values for use as a heading.
 */

package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Vector;
import java.util.concurrent.locks.Lock;

/**
 * Created by johnnie on 16/02/07.
 */
@I2cSensor(name = "LSM6-Gyro-Accel", xmlTag = "LSM6", description = "LSM6 Gyro and Accelerometer")
public class LSM6Impl extends I2cControllerPortDeviceImpl implements HardwareDevice, I2cController.I2cPortReadyCallback, LSM6 {
    public static final I2cAddr DS33_SA0_HIGH_ADDRESS = I2cAddr.create8bit(0xD6);
    public static final I2cAddr DS33_SA0_LOW_ADDRESS = I2cAddr.create8bit(0xD4);
    public static final I2cAddr DS33_SA0_ADDRESS = DS33_SA0_HIGH_ADDRESS;

    public static final int FUNC_CFG_ACCESS   = 0x01;

    public static final int FIFO_CTRL1        = 0x06;
    public static final int FIFO_CTRL2        = 0x07;
    public static final int FIFO_CTRL3        = 0x08;
    public static final int FIFO_CTRL4        = 0x09;
    public static final int FIFO_CTRL5        = 0x0A;
    public static final int ORIENT_CFG_G      = 0x0B;

    public static final int INT1_CTRL         = 0x0D;
    public static final int INT2_CTRL         = 0x0E;
    public static final int WHO_AM_I          = 0x0F;
    public static final int CTRL1_XL          = 0x10;
    public static final int CTRL2_G           = 0x11;
    public static final int CTRL3_C           = 0x12;
    public static final int CTRL4_C           = 0x13;
    public static final int CTRL5_C           = 0x14;
    public static final int CTRL6_C           = 0x15;
    public static final int CTRL7_G           = 0x16;
    public static final int CTRL8_XL          = 0x17;
    public static final int CTRL9_XL          = 0x18;
    public static final int CTRL10_C          = 0x19;

    public static final int WAKE_UP_SRC       = 0x1B;
    public static final int TAP_SRC           = 0x1C;
    public static final int D6D_SRC           = 0x1D;
    public static final int STATUS_REG        = 0x1E;

    public static final int OUT_TEMP_L        = 0x20;
    public static final int OUT_TEMP_H        = 0x21;
    public static final int OUTX_L_G          = 0x22;
    public static final int OUTX_H_G          = 0x23;
    public static final int OUTY_L_G          = 0x24;
    public static final int OUTY_H_G          = 0x25;
    public static final int OUTZ_L_G          = 0x26;
    public static final int OUTZ_H_G          = 0x27;
    public static final int OUTX_L_XL         = 0x28;
    public static final int OUTX_H_XL         = 0x29;
    public static final int OUTY_L_XL         = 0x2A;
    public static final int OUTY_H_XL         = 0x2B;
    public static final int OUTZ_L_XL         = 0x2C;
    public static final int OUTZ_H_XL         = 0x2D;

    public static final int FIFO_STATUS1      = 0x3A;
    public static final int FIFO_STATUS2      = 0x3B;
    public static final int FIFO_STATUS3      = 0x3C;
    public static final int FIFO_STATUS4      = 0x3D;
    public static final int FIFO_DATA_OUT_L   = 0x3E;
    public static final int FIFO_DATA_OUT_H   = 0x3F;
    public static final int TIMESTAMP0_REG    = 0x40;
    public static final int TIMESTAMP1_REG    = 0x41;
    public static final int TIMESTAMP2_REG    = 0x42;

    public static final int STEP_TIMESTAMP_L  = 0x49;
    public static final int STEP_TIMESTAMP_H  = 0x4A;
    public static final int STEP_COUNTER_L    = 0x4B;
    public static final int STEP_COUNTER_H    = 0x4C;

    public static final int FUNC_SRC          = 0x53;

    public static final int TAP_CFG           = 0x58;
    public static final int TAP_THS_6D        = 0x59;
    public static final int INT_DUR2          = 0x5A;
    public static final int WAKE_UP_THS       = 0x5B;
    public static final int WAKE_UP_DUR       = 0x5C;
    public static final int FREE_FALL         = 0x5D;
    public static final int MD1_CFG           = 0x5E;
    public static final int MD2_CFG           = 0x5F;


    private final I2cController cdim;
    public final byte[] readc;
    public byte[] readcold;
    private final Lock readlock;
    private final byte[] writec;
    private final Lock writelock;
    private I2cAddr i2cAddr;
    private final int port;
    private int initcounter = 0;
    private int datacounter = 0;
    public double zangle = 0.0, zgyrobias=0;
    private ElapsedTime dt = new ElapsedTime();

    public LSM6Impl(I2cController module, int physicalPort, I2cAddr i2cAddr) {
        super(module,physicalPort);
        this.i2cAddr = i2cAddr;
        this.port = physicalPort;
        this.cdim = module;
        this.readc = module.getI2cReadCache(physicalPort);
        this.readcold = this.readc.clone();
        this.readlock = module.getI2cReadCacheLock(physicalPort);
        this.writec = module.getI2cWriteCache(physicalPort);
        this.writelock = module.getI2cWriteCacheLock(physicalPort);
        module.registerForI2cPortReadyCallback(this, physicalPort);
        dt.reset();
    }

    public LSM6Impl(I2cController module, int physicalPort) {
        this(module,physicalPort,DS33_SA0_ADDRESS);
    }

    public byte[] getreadcache() {
        return this.readc;
    }

    private int readcache(int lowaddr) {
        int outval;
        try {
            this.readlock.lock();
            outval = this.readc[lowaddr-OUTX_L_G+5] << 8 | this.readc[lowaddr-OUTX_L_G+4] & 255;
        } finally {
            this.readlock.unlock();
        }
        return outval;
    }

    @Override
    public int getRawGyroX() {
        return readcache(OUTX_L_G);
    }

    @Override
    public int getRawGyroY() {
        return readcache(OUTY_L_G);
    }

    @Override
    public int getRawGyroZ() {
        return readcache(OUTZ_L_G);
    }

    @Override
    public double getGyroZ() {
        return getRawGyroZ() - zgyrobias;
    }

    @Override
    public int getaccelx() {
        return readcache(OUTX_L_XL);
    }

    @Override
    public int getaccely() {
        return readcache(OUTY_L_XL);
    }

    @Override
    public int getaccelz() {
        return readcache(OUTZ_L_XL);
    }

    @Override
    public double getHeading() { return zangle * 1.2 * 245.0 / (double)(1 << 15);}

    @Override
    public void setHeading(double angle) { zangle = angle; }

    @Override
    public Vector<Integer> getgyro() {
        Vector<Integer> gyrovals = new Vector<Integer>();
        gyrovals.add(getRawGyroX());
        gyrovals.add(getRawGyroY());
        gyrovals.add(getRawGyroZ());
        return gyrovals;
    }

    @Override
    public Vector<Integer> getaccel() {
        Vector<Integer> accelvals = new Vector<Integer>();
        accelvals.add(getaccelx());
        accelvals.add(getaccely());
        accelvals.add(getaccelz());
        return accelvals;
    }

    @Override
    public Vector<Integer > getimu() {
        Vector<Integer> imuvals = new Vector<Integer>();
        imuvals.addAll(getgyro());
        imuvals.addAll(getaccel());
        return imuvals;
    }

    @Override
    public void setI2cAddr(I2cAddr addr){
        i2cAddr = addr;
        initcounter = 0;
        datacounter = 0;
        zgyrobias = 0.0;
        RobotLog.i("setI2cAddr:%02X",addr.get8Bit());
    }
    public Manufacturer getManufacturer() {
        return controller.getManufacturer();
    }
    public String getDeviceName() {
        return "LSM6DS33 Gyro & Accel";
    }

    public String getConnectionInfo() {
        return this.cdim.getConnectionInfo() + "; I2C port: " + this.port;
    }

    public int getVersion() {
        return 1;
    }

    public void close() {
    }
    public void resetDeviceConfigurationForOpMode() {
    }
    @Override
    public void portIsReady(int port) {
//        RobotLog.i("In portIsReady");
//        if(!Arrays.equals(readcold, readc))
//            RobotLog.i("In portIsReady readc changed");
        switch (initcounter++){
            case 0:  this.b();   break;
            case 1:  this.c();   break;
            case 2:  this.d();   break;
            case 3:  this.a();   break;

            default:    this.cdim.readI2cCacheFromController(this.port);
                        this.cdim.setI2cPortActionFlag(this.port);
                        this.cdim.writeI2cPortFlagOnlyToController(this.port);
                        proccessData();
                        initcounter = 100;
        }
        //RobotLog.i(printcache(readc));
        //this.readcold = this.readc.clone();
      // if(initcounter <= 100)
        //    RobotLog.i("rc:%s",((Integer)getRawGyroZ()).toString());
    }

    private void proccessData() {
        if (datacounter++ > 100)
            datacounter = 100;

        if (datacounter > 10 && datacounter <= 20){
           zgyrobias += getRawGyroZ() / 10;
            RobotLog.i("zgyrobias:%s", ((Double)zgyrobias).toString());
        }
        else {
        /*if (zgyrobias == 0)
            zgyrobias = getRawGyroZ();
        else
            if (Math.abs(getRawGyroZ()-zgyrobias) < 0)
                zgyrobias = zgyrobias*9/10 + getRawGyroZ()/10;
        */
            zangle += dt.seconds() * getGyroZ();
        }
        dt.reset();
    }

    private void a() {
        this.cdim.enableI2cReadMode(this.port, this.i2cAddr, OUTX_L_G, 12);
        this.cdim.writeI2cCacheToController(this.port);
    }
    public String printcache(byte[] ab){
        String s = new String();
        for (byte by: ab) {
            s += String.format("%02X",by);
        }
        return s;
    }
    private void b() {
        RobotLog.i("In b");
        this.cdim.enableI2cWriteMode(this.port, this.i2cAddr, CTRL1_XL, 1);

        try {
            this.writelock.lock();
            this.writec[4] = 0x30;
        } finally {
            this.writelock.unlock();
        }

        this.cdim.setI2cPortActionFlag(this.port);
        this.cdim.writeI2cCacheToController(this.port);
        RobotLog.i("Out b");
    }
    private void c() {
        RobotLog.i("In c");
        this.cdim.enableI2cWriteMode(this.port, this.i2cAddr, CTRL2_G, 1);

        try {
            this.writelock.lock();
            this.writec[4] = 0x30;
        } finally {
            this.writelock.unlock();
        }

        this.cdim.setI2cPortActionFlag(this.port);
        this.cdim.writeI2cCacheToController(this.port);
        RobotLog.i("Out c");
    }
    private void d() {
        RobotLog.i("In d");
        this.cdim.enableI2cWriteMode(this.port, this.i2cAddr, CTRL3_C, 1);

        try {
            this.writelock.lock();
            this.writec[4] = 0x04;
        } finally {
            this.writelock.unlock();
        }

        this.cdim.setI2cPortActionFlag(this.port);
        this.cdim.writeI2cCacheToController(this.port);
        RobotLog.i("Out d");
    }
}
