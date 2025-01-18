package frc.robot.lib.util;


public class QoLUtil {
    //metodos para hacer el código más flexible y menos tedioso de manejar

    public static double square(double x){
        return x * x;
    }
    public static double inverse(double x, boolean v){
        return v? -x: x;
    }
    public static double percentageOf(double percentage, double x){
        double y = percentage * 0.01;
        return x * y;
    }

}
