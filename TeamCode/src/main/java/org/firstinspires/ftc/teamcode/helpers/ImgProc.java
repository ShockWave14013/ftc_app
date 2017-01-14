package org.firstinspires.ftc.teamcode.helpers;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InvalidClassException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.nio.ByteBuffer;
import java.util.LinkedList;
import java.util.concurrent.BlockingQueue;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.util.Log;
import android.widget.ImageView;
import android.widget.LinearLayout;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.internal.AppUtil;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;

/**
 * Created by johnnie on 2016/12/20.
 */

public class ImgProc {
    public File paramfile = new File(AppUtil.FIRST_FOLDER + "ImgProcParameters.conf");
    public ImgProcParameters parameters;

    private ImageView imageView = null;
    private Bitmap lastBmp = null;
    public BlockingQueue<CloseableFrame> frameq;

    ImgProcWorkerTask imgproctask = null;
    private final LinkedList<Integer> vispixels=new LinkedList<>();
    private Object angleLock = new Object();
    private float angle = 0.0F;

    private float fdist = 0.0F;

    public ImgProc(BlockingQueue<CloseableFrame> inframeq,float focaldist) {
        this.lastBmp = null;
        this.frameq = inframeq;
        this.fdist = focaldist;
        loadParametersFromFile();
    }

    public Bitmap getBitmap() {
        if(lastBmp == null)
            return null;
        synchronized (lastBmp) {
            return lastBmp;
        }
    }

    public void setBitmap(Bitmap bitmap) {
        if(lastBmp == null) {
            lastBmp = bitmap;
        }else{
            synchronized (lastBmp){
                this.lastBmp = bitmap;
            }
        }
    }

    public float getAngle() {
        float retangle;
        synchronized (angleLock){
            retangle=angle;
        }
        return retangle;
    }

    public void setAngle(float angleIn) {
        synchronized (angleLock) {
            angle = angleIn;
        }
    }

    public void setImageView(ImageView imageView) {
        this.imageView = imageView;
        imgproctask.setImageView(imageView);
    }

    public LinkedList<Integer> getVispixels() {
        LinkedList<Integer> templist = new LinkedList<>();
        synchronized (vispixels) {
            templist.addAll(vispixels);
        }
        return templist;
    }

    public void addVispixel(int pixel) {
        synchronized (vispixels) {
            vispixels.add(pixel);
        }
    }

    public void clearVispixel() {
        synchronized (vispixels) {
            vispixels.clear();
        }
    }

    public void startTask(){
        imgproctask = new ImgProcWorkerTask(imageView);
        imgproctask.execute(this);
    }

    public void setupImageView(){
        imageView = new ImageView(AppUtil.getInstance().getActivity());
        imageView.setImageBitmap(this.getBitmap());
        RobotLog.ii("ImViewer","matrix %s",imageView.getImageMatrix());
        AppUtil.getInstance().getActivity().runOnUiThread(
                new Runnable() {
                    @Override
                    public void run() {
                        ((LinearLayout) AppUtil.getInstance().getActivity().findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId)).addView(imageView);
                    }
                }  );
        RobotLog.ii("ImViewer","end");
        this.setImageView(imageView);
    }

    public void teardown(){
        imgproctask.terminateTask();
        //final ImageView  imageView = new ImageView(AppUtil.getInstance().getActivity());
        AppUtil.getInstance().getActivity().runOnUiThread(
            new Runnable() {
                @Override
                public void run() {
                    ((LinearLayout) AppUtil.getInstance().getActivity().findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId)).removeView(imageView);
                }
            }  );
        RobotLog.ii("ImViewer","Exitting");
        imageView.setImageBitmap(null);
        this.setImageView(null);
    }

    public void process(){

//        final ThresholdingWorkerTask thresh = new ThresholdingWorkerTask(imageView);
//        AppUtil.getInstance().getActivity().runOnUiThread(
//                    new Runnable() {
//                        @Override
//                        public void run() {
//                            thresh.execute(this);
//                        }
//                    }  );
        //imgproctask.execute(this);
        imgproctask.run();
//        while (thresh.getStatus() != AsyncTask.Status.FINISHED){
//            Thread.yield();
//        }
    }

    public static Image getImageFromFrame(CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            //RobotLog.ii("ImgProc","%d of %d  format %s",i,numImgs,frame.getImage(i).getFormat());
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    private Bitmap tempbitmap = null;
    private byte[] imgRGB888 = null;
    private int[] colors = null;

    public Bitmap addAlpha(ByteBuffer imgRGB888buf, int width, int height)
    {
        if (tempbitmap == null)
            tempbitmap = Bitmap.createBitmap  (width, height, Bitmap.Config.ARGB_8888);
        imgRGB888buf.rewind();
        if (imgRGB888 == null)
            imgRGB888 = new byte[imgRGB888buf.remaining()];
        imgRGB888buf.get(imgRGB888);
        if (colors == null)
            colors = new int[width * height];
        int samplecf=(height/2) *width;
        int sampleci=samplecf - width;
        //boolean prevvis = false;
        int prevviscount =0;
        clearVispixel();
        int r,g,b, starti=0, endi=colors.length;

        if(imageView== null){
            starti = sampleci;
            endi = samplecf;
        }
        Log.i("ImgProc", "Before Forloop w"+width+"h"+height);
        for (int ci = 0; ci < colors.length; ci++)
        {

            r = (int)(0xFF & imgRGB888[3*ci]);
            g = (int)(0xFF & imgRGB888[3*ci+1]);
            b = (int)(0xFF & imgRGB888[3*ci+2]);
            if(ci == colors.length/2){
                Log.i("ImgProc", "InLoop ir:"+ imgRGB888[3*ci]+"or:"+r);
            }

            if(     r < parameters.redLow ||
                    r > parameters.redHigh ||
                    g < parameters.greenLow ||
                    g > parameters.greenHigh ||
                    b < parameters.blueLow ||
                    b > parameters.blueHigh   ){
                r=0;
                b=0;
                g=0;
            }
            colors[ci] = Color.rgb(r, g, b);
            if (ci > sampleci  && ci < samplecf && r !=0){
                if(prevviscount == 0)
                    addVispixel(ci-sampleci);
                prevviscount++;
            }else{
                if(prevviscount != 0)
                    addVispixel(prevviscount);
                prevviscount=0;
            }

        }
        Log.i("ImgProc","visPixel:"+getVispixels().toString());
        int blockc=0,blocksize=0;
        //Integer blockarr[] = (Integer[])getVispixels().toArray();
        LinkedList<Integer> blocklist=new LinkedList<>(getVispixels());
        if(blocklist.size() !=0) {
            for (int i = 1; i < blocklist.size(); i += 2) {
                if (blocklist.get(i) > blocksize) {
                    blocksize = blocklist.get(i);
                    blockc = i - 1;
                }
            }
            Log.i("ImgProc", "visBlock:start:" + blocklist.get(blockc) + ",size:" + blocklist.get(blockc+1));
            setAngle((float)Math.toDegrees(Math.atan2(width/2-(blocklist.get(blockc)+blocksize/2),fdist)));
            Log.i("ImgProc", "visAngle:"+getAngle()+"xpixel:"+(width/2-(blocklist.get(blockc)+blocksize/2)));
        }
        Log.i("ImgProc", "After Forloop ");
        if(imageView != null) {
            tempbitmap.setPixels(colors, 0, width, 0, 0, width, height);
            Log.i("ImgProc", "After SetPixels ");
            tempbitmap.setHasAlpha(false);
            Matrix rmat = new Matrix();
            rmat.postRotate(90F);
            //bitmap = Bitmap.createBitmap(bitmap,0,0,width, height,rmat,true);
            Log.i("ImgProc", "After rotate ");
            setBitmap(tempbitmap.copy(tempbitmap.getConfig(), true));
        }
//        if(lastBmp == null) {
//            lastBmp = tempbitmap;
//        }else{
//            synchronized (lastBmp){
//                this.lastBmp = tempbitmap.copy(tempbitmap.getConfig(),true);
//            }
//        }
        return tempbitmap;
    }

    public void loadParametersFromFile(){
        //paramfile.delete();
        if(!paramfile.exists()){
            parameters = new ImgProcParameters();
            saveParametersToFile();
            return;
        }
        try {
            FileInputStream fis = new FileInputStream(paramfile);
            ObjectInputStream input = new ObjectInputStream(fis);
            parameters = ((ImgProcParameters)input.readObject());
        }catch (InvalidClassException e){
            paramfile.delete();
            parameters = new ImgProcParameters();
        }
        catch (ClassNotFoundException | IOException e){
            e.printStackTrace();
        }

    }
    public void saveParametersToFile(){
        try {
            FileOutputStream fos = new FileOutputStream(paramfile);
            ObjectOutputStream output = new ObjectOutputStream(fos);
            output.writeObject(parameters);
        }catch (IOException e){
            e.printStackTrace();
        }
    }

}
