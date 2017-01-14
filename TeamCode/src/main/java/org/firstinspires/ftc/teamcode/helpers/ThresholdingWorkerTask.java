package org.firstinspires.ftc.teamcode.helpers;

import android.app.Activity;
import android.graphics.Bitmap;
import android.os.AsyncTask;
import android.os.Debug;
import android.util.Log;
import android.widget.ImageView;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.ftc.robotcore.internal.network.SendOnceRunnable;

import java.lang.ref.WeakReference;

/**
 * Created by johnnie on 2016/12/21.
 */

public class ThresholdingWorkerTask extends AsyncTask<ImgProc, Void, ImgProc> {
    private final WeakReference<ImageView> imageViewReference;
    private ImgProc in = null;

    public ThresholdingWorkerTask(ImageView imageView) {
        // Use a WeakReference to ensure the ImageView can be garbage collected
        imageViewReference = new WeakReference<>(imageView);
    }

    // Decode image in background.
    @Override
    protected ImgProc doInBackground(ImgProc... params){
        in = params[0];
        Log.i("ThresholdingWorkerTask","In doInBackground "+this.toString());
        //Image img = ImgProc.getImageFromFrame(in.getFrame(), PIXEL_FORMAT.RGB888);
        //Log.i("ThresholdingWorkerTask","In doInBackground getImfromFrm "+img.toString());
        //in.addAlpha(img.getPixels(),img.getWidth(),img.getHeight());
        Log.i("ThresholdingWorkerTask","Out doInBackground "+this.toString());
        return in;
    }

    // Once complete, see if ImageView is still around and set bitmap.
    @Override
    protected void onPostExecute(ImgProc imgproc) {
        final Bitmap bitmap = imgproc.getBitmap();
        Log.i("ThresholdingWorkerTask","In onPostExec");
        if (bitmap != null) {
            final ImageView imageView = imageViewReference.get();
            Log.i("ThresholdingWorkerTask","After imviewget");
            if (imageView != null) {
//                AppUtil.getInstance().getActivity().runOnUiThread(new Runnable() {
//                    @Override
//                    public void run() {
                        imageView.setImageBitmap(bitmap);
//                    }
//                });
                Log.i("ThresholdingWorkerTask","After bm update");
            }
        }
    }
}