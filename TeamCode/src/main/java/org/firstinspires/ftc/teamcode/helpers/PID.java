package org.firstinspires.ftc.teamcode.helpers;

import android.os.SystemClock;
import android.util.Log;

import java.util.Timer;

/**
 * Created by johnnie on 2016/12/29.
 */

public class PID {
    double Kp,Ki,Kd;
    double I, pe = 0;
    long lastrunTime = 0;
    boolean IzerocrossReset = false;

    public PID(double kp,double ki,double kd,boolean Izerocross){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        I  = 0;
        pe = 0;
        IzerocrossReset = Izerocross;
        lastrunTime = SystemClock.elapsedRealtime();
    }

    public double run(double e){
        double dt = (SystemClock.elapsedRealtime()  - lastrunTime)/1000.0;
        Log.i("PID", "dt:"+dt);
        I += e*dt;
        if(IzerocrossReset && Math.signum(pe)*Math.signum(e)< 0)
            I = 0.0;
        Log.i("PID", "P:"+e*Kp+"I:"+I*Ki+"D:"+Kd*(e-pe)/dt);
        double out = e*Kp + I*Ki + Kd*(e-pe)/dt;
        pe = e;
        lastrunTime = SystemClock.elapsedRealtime();
        return out;
    }

}
