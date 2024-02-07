package org.firstinspires.ftc.teamcode.Utilities;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;
import androidx.annotation.NonNull;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import java.util.ArrayList;
import java.util.Locale;
import java.util.function.DoubleSupplier;

public class ColourMassDetectionProcessor implements VisionProcessor {
    private final DoubleSupplier minArea, left, right;
    private final Scalar upper;
    private final Scalar lower;
    private final TextPaint textPaint;
    private final Paint linePaint;
    private final ArrayList<MatOfPoint> contours;
    private final Mat hierarchy = new Mat();
    private double largestContourX;
    private double largestContourY;
    private double largestContourArea;
    private MatOfPoint largestContour;
    private PropPositions previousPropPosition;
    private PropPositions recordedPropPosition = PropPositions.UNFOUND;

    public ColourMassDetectionProcessor(@NonNull Scalar lower, @NonNull Scalar upper, DoubleSupplier minArea, DoubleSupplier left, DoubleSupplier right) {
        this.contours = new ArrayList<>();
        this.lower = lower;
        this.upper = upper;
        this.minArea = minArea;
        this.left = left;
        this.right = right;


        textPaint = new TextPaint();
        textPaint.setColor(Color.GREEN);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(10);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    public double getLargestContourX() {
        return largestContourX;
    }
    public double getLargestContourY() {
        return largestContourY;
    }
    public double getLargestContourArea() {
        return largestContourArea;
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        Core.inRange(frame, lower, upper, frame);

        contours.clear();

        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        largestContourArea = -1;

        largestContour = null;

        double minArea = this.minArea.getAsDouble();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > minArea) {
                largestContour = contour;
                largestContourArea = area;
            }
        }
        largestContourX = largestContourY = -1;

        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);
        }

        PropPositions propPosition;
        if (largestContour == null) {
            propPosition = PropPositions.UNFOUND;
        } else if (largestContourX < left.getAsDouble()) {
            propPosition = PropPositions.LEFT;
        } else if (largestContourX > right.getAsDouble()) {
            propPosition = PropPositions.RIGHT;
        } else {
            propPosition = PropPositions.MIDDLE;
        }

        if (propPosition != previousPropPosition && propPosition != PropPositions.UNFOUND) {
            recordedPropPosition = propPosition;
        }

        previousPropPosition = propPosition;

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if (largestContour != null) {
            Rect rect = Imgproc.boundingRect(largestContour);

            float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

            canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
            canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);

            canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
            canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);

            String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());

            canvas.drawText(text, (float) largestContourX * scaleBmpPxToCanvasPx, (float) largestContourY * scaleBmpPxToCanvasPx, textPaint);
        }
    }

    public PropPositions getRecordedPropPosition() {
        return recordedPropPosition;
    }

    public MatOfPoint getLargestContour() {
        return largestContour;
    }

    public void close() {
        hierarchy.release();
    }

    public enum PropPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}