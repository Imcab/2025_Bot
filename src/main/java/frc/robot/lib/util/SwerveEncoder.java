//v1.0.0
package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Libreria para los encoders que utilizamos para el swerve
 */
public class SwerveEncoder{

    /**
     * Modulos del swerve para identificarlos más fácil
     */
    public enum module{
        NONE,FL,FR,BL,BR
    }

    private AnalogInput encoder;
    private double offset;
    private module mod;
    private boolean mValue;

    /**
     * Crea un encoder del swerve
     * @param mod modulo al que está asociado el encoder (FL,FR,BL,BR)
     * @param Port puerto del encoder
     * @param offset offset (EN GRADOS)
     */
    public SwerveEncoder(module mod, int Port, double offset){
        this.encoder = new AnalogInput(Port);
        this.offset = offset;
        this.mod = mod;
        this.mValue = false;
    }
    /**
     * Crea un encoder del swerve
     * @param mod modulo al que está asociado el encoder (FL,FR,BL,BR)
     * @param Port puerto del encoder
     */
    public SwerveEncoder(module mod, int Port){
        this.encoder = new AnalogInput(Port);
        this.offset = 0;
        this.mod = mod;
        this.mValue = false;
    }
    /**
     * Crea un encoder del swerve
     * @param Port puerto del encoder
     * @param offset offset (EN GRADOS)
     */
    public SwerveEncoder(int Port, double offset){
        this.encoder = new AnalogInput(Port);
        this.offset = offset;
        this.mod = module.NONE;
        this.mValue = false;
    }
    /**
     * Crea un encoder del swerve
     * @param Port puerto del encoder
     */
     public SwerveEncoder(int Port){
        this.encoder = new AnalogInput(Port);
        this.offset = 0;
        this.mod = module.NONE;
        this.mValue = false;
    }

    /*
     * Obtiene el encoder, permitiendousar métodos de la clase Analog Input
     * @return el objeto de Analog Input
    * */
    public AnalogInput getAnalog(){
        return encoder;
    }

    /**
     * Asigna un modulo al encoder
     * @param type modulo al que asociar el encoder (FL,FR,BL,BR)
     */
    public void setModule(module type){
        this.mod = type;
    }

    /**
     * Obtiene el modulo al que esta asociado el encoder
     * @return (FL,FR,BL,BR)
     */
    public module getModule(){
        return mod;
    }

    /**
     * Quita el modulo asociado al encoder
     */
    public void resetModule(){
        this.mod = module.NONE;
    }

    /**
     * Obtiene el nombre del modulo al que esta asociado el encoder
     * @return "(FL,FR,BL,BR)""
     */
    public String getName(){
        return mod.toString();
    }

    /**
     * Obtiene el puerto del encoder
     * @return el puerto
     */
    public int getPort(){
        return encoder.getChannel();
    }

    /**
     * Ajusta automaticamente el offset
     */
    public void adjustOffset(){
        double off = getDegrees();
        setOffset(off);
    }
    /**
     * Ajusta el offset del encoder
     * @param offset offset (EN GRADOS)
     */
    public void setOffset(double offset){
        this.offset = offset;
    }
    /**
     * Obtiene el offset actual para el encoder
     * @return offset en grados
     */
    public double getOffset(){
        return offset;
    }

    /**
     * Pone el offset a 0
     */
    public void resetOffset(){
        offset = 0;
    }

    /**
     * Obtiene los grados del encoder
     * @return grados del encoder
     */
    public double getDegrees(){
        double angleEncoder = (encoder.getValue() * 360) / 4096;
        return (angleEncoder - offset);
    }
    /**
     * Obtiene los radianes del encoder
     * @return radianes del encoder
     */
    public double getRads(){
        return Units.degreesToRadians(getDegrees());
    }

    /**
     * Obtiene las rotaciones del encoder
     * @return rotaciones del encoder
     */
    public double getRotations(){
        return Units.degreesToRotations(getDegrees());
    }
    
    /**
     * Obtiene la rotación en 2D del encoder
     * @return una rotación en 2D
     */
    public Rotation2d getRotation2D(){
        return Rotation2d.fromDegrees(getDegrees());
    }
    /**
     * Checa si el encoder esta activado manualmente
     * @return falso si el encoder ha sido desactivado manualmente
     */
    public boolean isDisconnected(){
        return mValue;
    }
    /**
     * Desconecta el encoder manualmente
     * @param trigger
     */
    public void disconnectWhen(boolean trigger){
        this.mValue = trigger;
    }
}
