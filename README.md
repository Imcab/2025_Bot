# 🪼🦀WELCOME TO 2025 FRC: REEFSCAPE 3472 CODE

![heading](/images/heading.png)

Bienvenidos al código de Buluk3472 de la temporada 2025, este código se caracteriza por manejo de swerve en diferentes formas, vision integrada y uso de pathplanner.
Lo mas relevante de nuestro código 📂⬇️

[**lib:**](/src/main/java/frc/robot/lib) una amplia variedad de archivos de utilidad desde propias clases de encoders para el swerve, vision, gestor de alertas, constantes de pid hasta configuraciones para el swerve


[**RobotState**](/src/main/java/frc/robot/RobotState.java) clase de métodos estáticos donde se manejan y reportan todos los estados del robot desde la bateria, tiempo restante del juego hasta si el robot esta yendo a una velocidad muy alta o si el gyroscopio esta conectado


[**SuperStructure**](/src/main/java/frc/robot/Subsystems/Superstructure.java) clase de una superestructura que trackea los objetivos deseados del robot y hace request para operar todos los mecanismos de manera precisa y en conjunto (una superestructura es básicamente todo aquel mecanismo que se encuentre arriba del chassis)


Encuentra nuestros subsistemas [aqui](/src/main/java/frc/robot/Subsystems).

Encuentra nuestros comandos [aqui](/src/main/java/frc/robot/Commands).

🔴ChangeLog abajo:

# ✅COMMIT #15  - MAJOR FIX CHANGES

## ⚠️**IMPORTANT CHANGES**

- Corregir el método de alinear el swerve con la limelight (estaba mal al momento de mandar el setpoint)
- Cambiar límites de corriente de los motores del swerve a 20 para el motor que gira y 40 para el motor que avanza (valores anteriores: 30,30)

## 🛠️ **GENERAL CHANGES**

-Se crearon nuevos métodos para manejar el swerve en [DriveTrain](/src/main/java/frc/robot/Subsystems/DriveTrain.java), moveInX y moveInY haciendo que se mueva en cierta cantidad de metros en la coordenada X o Y
-Remover el método del voltaje de la clase [SwerveEncoder](/src/main/java/frc/robot/lib/util/SwerveEncoder.java)
-Mostrar el swerve en la dashboard (como si se viera en advantageScope)
-Pulir el código del swerve
- Se quitaron y añadieron métodos a [RobotState](/src/main/java/frc/robot/RobotState.java)
- Creación del método "toSetpoint" en [DriveTrain](/src/main/java/frc/robot/Subsystems/DriveTrain.java) que manda todos los modulos al setpoint solicitado. esto para ahorrar lineas de código
- Se añadieron mas campos a [SwerveConfig](/src/main/java/frc/robot/lib/SwerveConfig.java), uno donde se guarda el límite de corriente y otro para guardar si se debe de invertir el gyroscopio
- Se cambiaron los commandos para el swerve, modifcando la conducta de cambiar el frente para que solo corra el comando una vez al igual que el de frenar. se eliminaron parámetros para el comando de alinear

### 🔵 **OTHER FEATURES**

- Se cambió el nombre de alinear el swerve con el apriltag de "getinRange()" a "centerWithApriltag()"
- Creacion de [QoLutil](/src/main/java/frc/robot/lib/util/QoLutil.java), que cuenta con un método para sacar porcentaje, elevar al cuadrado e invertir un valor con una condición
- Eliminación de la notificación de la conexion de la navx
- Cambiar el orden de los métodos del swerve
- Eliminación de métodos inecesarios 
- Eliminacion de la constante "MAX_SPEED_RPS" en [SwerveConfig](/src/main/java/frc/robot/lib/SwerveConfig.java) ya que era la misma que "MAX_LINEAR_SPEED"
- Mostrar el timepo de la partida en la dashboard
- Cambiar la velocidad de rastreo máxima en [SwerveConfig](/src/main/java/frc/robot/lib/SwerveConfig.java) de "3" mps a el 75% de la máxima velocidad "5.79" mps
- Se cambió el nombre del comando que frena el swerve de "formX" a "brake" (que significa frenar) 
- Comentarios en diferentes funciones
- Se removieron importaciones innecesarias
- Cambiar el método de getPeriod() en robotState por uno fijo "0.02" (es el mismo valor)

#### Imad - 12/01/2025
