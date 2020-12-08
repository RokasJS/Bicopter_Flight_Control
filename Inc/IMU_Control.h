#ifdef __cplusplus
extern "C" {
#endif

// Header file for IMU_Control.c
// Variables
extern int Addr_Accl, Addr_Gyro;
extern float xAccl, yAccl, zAccl, xGyro, yGyro, zGyro;

// Function prototypes
void BMX_Setup(void);
void read_acc_gyro(void);
void fastcompaccelBMX055(void);

#ifdef __cplusplus
}
#endif
