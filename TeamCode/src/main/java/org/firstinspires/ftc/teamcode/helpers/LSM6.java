package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.Vector;

/**
 * Created by estie on 2016/12/10.
 */

public interface LSM6 {
    int getRawGyroX();

    int getRawGyroY();

    int getRawGyroZ();

    double getGyroZ();

    int getaccelx();

    int getaccely();

    int getaccelz();

    double getHeading();

    void setHeading(double angle);

    Vector<Integer> getgyro();

    Vector<Integer> getaccel();

    Vector<Integer > getimu();

    void setI2cAddr(I2cAddr addr);
}
