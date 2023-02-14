package org.firstinspires.ftc.teamcode;

public class CameraThread extends _DeliveryTeleopTwoElectricBoogaloo implements Runnable {

    public Vision vision;
    public Sensors sensors;

    public CameraThread(Sensors sensors, Vision vision){
        this.sensors=sensors;
        this.vision=vision;
    }

    @Override
    public void run() {
        while(!Thread.currentThread().isInterrupted()){
            if(getSlidePos()>1250){
                
            }
        }
    }
}
