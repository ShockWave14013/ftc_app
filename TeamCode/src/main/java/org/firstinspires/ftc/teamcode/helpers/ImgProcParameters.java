package org.firstinspires.ftc.teamcode.helpers;

import java.io.Serializable;

/**
 * Created by johnnie on 2016/12/23.
 */
public class ImgProcParameters implements Serializable {

    private static final long serialVersionUID = 1L;

    public int redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh;
    public boolean filter;

    public ImgProcParameters() {
        this.redLow = 0;
        this.redHigh = 255;
        this.greenLow = 0;
        this.greenHigh = 255;
        this.blueLow = 0;
        this.blueHigh = 255;
        this.filter = true;
    }
    public String printRanges(){
        return String.format("R:%d-%d,G:%d-%d,B:%d-%d",redLow,redHigh,greenLow,greenHigh,blueLow,blueHigh);
    }
//        public ImgProcParameters(boolean filter, int redLow, int redHigh, int greenLow, int greenHigh, int blueLow, int blueHigh) {
//            this.redLow = redLow;
//            this.redHigh = redHigh;
//            this.greenLow = greenLow;
//            this.greenHigh = greenHigh;
//            this.blueLow = blueLow;
//            this.blueHigh = blueHigh;
//            this.filter = filter;
//        }

//        public ImgProcParameters(ImgProcParameters imgProcParam){
//            this.redLow = imgProcParam.redLow;
//            this.redHigh = imgProcParam.redHigh;
//            this.greenLow = imgProcParam.greenLow;
//            this.greenHigh = imgProcParam.greenHigh;
//            this.blueLow = imgProcParam.blueLow;
//            this.blueHigh = imgProcParam.blueHigh;
//            this.filter = imgProcParam.filter;
//        }


}
