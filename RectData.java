package org.firstinspires.ftc.teamcode;

public class RectData {

    private double width;
    private double height;

    public RectData(double width, double height){
        this.width=width;
        this.height=height;
    }

    public double getWidth(){
        return width;
    }

    public void setWidth(double width){
        this.width = width;
    }


    public double getHeight(){
        return height;
    }

    public boolean equals(RectData rect){
        if((this.width>=rect.getWidth()-5 || this.width<=rect.getWidth()+5) && (this.height>=rect.getHeight()-25 || this.height<=rect.getHeight()+25)){
            return true;
        }
        return false;
    }
}
