package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class CameraThread extends _DeliveryTeleopTwoElectricBoogaloo implements Runnable {

    public Vision vision;
    public Sensors sensors;

    public ArrayList<Double> dThetas = new ArrayList<>();
    public ArrayList<Double> dists = new ArrayList<>();

    public CameraThread(Sensors sensors, Vision vision){
        this.sensors=sensors;
        this.vision=vision;
    }

    double dTheta = 2;
    double dist = 0;

    public double getDTheta(){
        return dTheta;
    }

    public double getDist(){
        return dist;
    }

    @Override
    public void run() {
        while(!Thread.currentThread().isInterrupted()){

            if(getSlidePos()>1250){

                double newDTheta = vision.findClosePoleDTheta();
                double newDist = vision.findClosePoleDist();


                if(dThetas.size()>5 && newDTheta!=-1){
                    dThetas.remove(0);
                    dThetas.add(newDTheta);
                }
                else if(newDTheta!=-1){
                    dThetas.add(newDTheta);
                }


                if(dists.size()>5 && newDist!=-1){
                    dists.remove(0);
                    dists.add(newDist);
                }
                else if(newDist!=-1){
                    dists.add(newDist);
                }


                //Take mean dTheta if not empty
                dTheta = 2;
                if(dThetas.size()!=0){
                    if (dThetas.size() % 2 == 0)
                        dTheta = (dThetas.get(dThetas.size()/2) + (dThetas.get(dThetas.size()/2-1)))/2;
                    else
                        dTheta = (dThetas.get(dThetas.size()/2));
                }


                //Take mean dist if not empty
                dist = 0;
                if(dists.size()!=0){
                    if (dists.size() % 2 == 0)
                        dist = (dists.get(dists.size()/2) + (dists.get(dists.size()/2-1)))/2;
                    else
                        dist = (dists.get(dists.size()/2));
                }




                if(isEqual(dTheta, 0, Math.toRadians(3.5)) && isEqual(dist, 8, 2)){
                    sensors.setLEDState(Sensors.LED_STATE.POLE_GOOD);
                }
                else{
                    sensors.setLEDState(Sensors.LED_STATE.POLE_BAD);
                }


                pause(0.110);


            }
            else{
                dThetas.clear();
                dists.clear();
                sensors.setLEDState(Sensors.LED_STATE.DEFAULT);
            }

        }
    }

    public boolean isEqual (double x, double delta, double a) //X = sensor input, A = ideal input, delta = range/2
    {
        return Math.abs(x-a) < (delta/2);
    }

    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs){

        }
    }

}
