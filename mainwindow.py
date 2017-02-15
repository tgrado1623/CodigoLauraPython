import time, os, ctypes

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

os.sys.path.append('../dynamixel_functions_py')     # Path setting

import dynamixel_funtions as dynamixel

## Elementos de la tabla de control del dynamixel AX-12
P_TORQUE_ENABLE         = 24 ## byte del habilitacion del torque
P_GOAL_POSITION_L	= 30 ## byte bajo de la posición objetivo
P_GOAL_POSITION_H	= 31 ## byte alto de la posición objetivo
P_MOVING_SPEED_L        = 32 ## byte bajo de la velocidad de movimiento
P_MOVING_SPEED_H        = 33 ## byte alto de la velocidad de movimiento
P_PRESENT_SPEED         = 38 ## byte bajo de la velocidad

## Longitud de los bytes de datos
GOAL_POSITION_LEN       = 2  ## Indica que la posicion objetivo corresponde a 2 bytes

## Version de protocolo
PROTOCOL_VERSION        = 1

## Ajustes por defecto
BAUDRATE		= 1000000   ## Tasa de transmisión
NUM_ACTUATOR		= 10        ## Número de actuadores
MUESTREO                = 10        ## Número de muestras por segundo
COMM_SUCCESS            = 0
DEVICENAME              = "/dev/ttyUSB0".encode('utf-8')

## Variables globales
MOD_ACTIVOS=[0]*NUM_ACTUATOR      ## Dejar el valor 0 para inactivos y el valor 1 para activos


## Funciones
def Choset(amplitud_par,amplitud_impar,desfase,dtheta_dn,dtheta_dt,t_time,n,offset_par,offset_impar):
    theta=(dtheta_dn*n + dtheta_dt*(t_time))
    if (n%2==0):
	return (((offset_par*3.14159)/180) + ((amplitud_par*3.14159)/180)*sin((theta*3.14159)/180))*180/3.14159
    else:
	return (((offset_impar*3.14159)/180) + ((amplitud_impar*3.14159)/180)*sin(((theta+desfase)*3.14159)/180))*180/3.14159

   

## Variables locales
ID = list(range(1,NUM_ACTUADOR+1))
tiempo=5

## Parametros de la ecuacion de Choset (linear)
amplitud_par = 30
amplitud_impar = 0
offset_par = 0
offset_impar = 0 
desfase = 0
dtheta_dn = 120
dtheta_dt = 18


port_num = dynamixel.portHandler(DEVICENAME)

dynamixel.packetHandler()

## Inicializa miembros de la estructura de puntero de datos de paquetesInicia
group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, P_GOAL_POSITION_L, GOAL_POSITION_LEN)

## Se abre el puerto para proceder al envío de paquetes
if dynamixel.openPort(port_num):    
  print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

## Establece la tasa de transmision
if dynamixel.setBaudRate(port_num, BAUDRATE):   
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()


## Una vez seleccionado un tiempo desde la interfaz, se coge este número para evaluar las posiciones de los servomotores dentro de
## cada segundo de tiempo seleccionado
for t_time in range(tiempo):
    
    ## Se recorre el número de muestras predeterminado en las variables globales (indica el número de veces por segundo que se
    ## le enviará un comando a cada servomotor para moverse)
    for muestra in range(MUESTREO):
        
        ## función para iniciar un conteo, se busca que la instrucción no se demore más de 100 ms
        t_ini=time.time()
        
        ## Se recorre el numero de servomotores
        for i in NUM_ACTUATOR:

            ## Se calcula el ángulo para el número de servomotor en el tiempo determinado
            angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)

            ## Si el módulo se seleccionó como inactivo, el ángulo se vuelve cero.
            angulo*=MOD_ACTIVOS[i]
            goalpos = 512 - angulo*3.41
            
            ## Se arma el paquete a enviar al servomotor. Es el envío del comando de posición
            ## Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), goalpos, GOAL_POSITION_LEN)).value
            print(dxl_addparam_result)
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (i+1))
                quit()
        
        ## Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(group_num)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        ## Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(group_num)

        ## Se finaliza el conteo iniciado antes del proceso de transmisión del paquete de estadosfunción para finalilzar un conteo
        t_fin=time.time()

        while ((t_fin-t_ini)*1000<100):
            t_fin=time.time()

## Close port
dynamixel.closePort(port_num)










            
                
