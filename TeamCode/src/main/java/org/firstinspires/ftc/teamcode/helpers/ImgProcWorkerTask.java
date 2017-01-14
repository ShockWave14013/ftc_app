package org.firstinspires.ftc.teamcode.helpers;

import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.util.Log;
import android.widget.ImageView;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.lang.ref.WeakReference;

/**
 * Created by johnnie on 2016/12/21.
 */

public class ImgProcWorkerTask extends AsyncTask<ImgProc, ImgProc, Void> {
    private WeakReference<ImageView> imageViewReference;
    private ImgProc in = null;
    private Object lockOb = new Object();
    //private final ReentrantLock lock = new ReentrantLock();
    ///private final Condition tryAgain = lock.newCondition();
    private Integer runNow = 0;
    private volatile boolean finished = false;

    public ImgProcWorkerTask(ImageView imageView) {
        // Use a WeakReference to ensure the ImageView can be garbage collected
        imageViewReference = new WeakReference<>(imageView);
    }

    // Decode image in background.
    @Override
    protected Void doInBackground(ImgProc... params){
        in = params[0];
        int nowRunNow;
        VuforiaLocalizer.CloseableFrame frame = null;
        Log.i("ImgProcWorkerTask", "In doInBackground " );
        do {
            Log.i("ImgProcWorkerTask", "In do ");
//            synchronized (in.getFrame()) {
            Log.i("ImgProcWorkerTask", "In insync do ");

            do{
                frame = in.frameq.poll();
            }while(frame == null && !finished);

            if(frame == null)return null;

            Image img = ImgProc.getImageFromFrame(frame, PIXEL_FORMAT.RGB888);
            Log.i("ImgProcWorkerTask", "In doInBackground getImfromFrm "+ img.getPixels().toString() );

                in.addAlpha(img.getPixels(), img.getWidth(), img.getHeight());
//            }
            Log.i("ImgProcWorkerTask", "After setBm " );
            publishProgress(in);
            Log.i("ImgProcWorkerTask", "After publishPrg " );
            //tryAgain.await();
//            do {
//                synchronized (runNow) {
//                    nowRunNow = runNow;
//                }
//            } while (!nowRunNow && !finished);
//            synchronized (runNow) {
//                runNow = false;
//            }

            synchronized (lockOb) {
                Log.i("ImgProcWorkerTask", "In Lock " );
                if (runNow.compareTo(0) > 0 ) runNow--;
                //nowRunNow = runNow;

                if (runNow.compareTo(0) == 0) {
                    Log.i("ImgProcWorkerTask", "Before wait " );
                    try {
                        lockOb.wait();
                    }catch (InterruptedException e){
                        e.printStackTrace();
                    }
                    Log.i("ImgProcWorkerTask", "After wait " );
                }
                nowRunNow = runNow;
            }
            Log.i("ImgProcWorkerTask", "Out do " );
        } while (nowRunNow != 0 && !finished);

        Log.i("ImgProcWorkerTask", "Exitting" );
        return null;
    }

    @Override
    protected void onProgressUpdate(ImgProc... imgproc)
    {
        final Bitmap bitmap = imgproc[0].getBitmap();
        Log.i("ImgProcWorkerTask","In onPostExec");
        if (bitmap != null) {
            final ImageView imageView = imageViewReference.get();
            Log.i("ImgProcWorkerTask","After imviewget");
            if (imageView != null) {
//                AppUtil.getInstance().getActivity().runOnUiThread(new Runnable() {
//                    @Override
//                    public void run() {
                imageView.setImageBitmap(bitmap);
//                    }
//                });
                Log.i("ImgProcWorkerTask","After bm update");
            }
        }

    }

    public void run() {
        // Call this to request data from the server again

        synchronized (lockOb) {
            runNow = -1;
            lockOb.notifyAll();
        }

    }
    public void pause() {
        // Call this to request data from the server again

        synchronized (lockOb) {
            runNow = 0;
            lockOb.notifyAll();
        }

    }
    public void runOnce() {
        // Call this to request data from the server again

        synchronized (lockOb) {
            runNow = 1;
            lockOb.notifyAll();
        }

    }
    public void setImageView(ImageView imageView){
        imageViewReference = new WeakReference<>(imageView);
    }

    public void terminateTask() {
        // The task will only finish when we call this method
        finished = true;
        //lock.unlock();
    }

    @Override
    protected void onCancelled() {
        // Make sure we clean up if the task is killed
        terminateTask();
    }

}