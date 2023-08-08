
	
						/* INICIO DEL PROGRAMA */
	
#include "support_common.h" /* include peripheral declarations and more */
#if (CONSOLE_IO_SUPPORT || ENABLE_UART_SUPPORT) 
/* Standard IO is only possible if Console or UART support is enabled. */
#include <stdio.h>
#endif


#define 	ESCLAVO 		0x50;
#define 	LEER_ESCLAVO 	0x51;
#define 	BRILLO_LCD 		80; 	// BRILLO DEL LCD ENTRE 0 MIN - 255 MAX, BRILLO.
#define 	CONTRASTE_LCD	125;	// CONTRASTE DE LCD 0 MIN - 255 MAX, CONTRASTE.

#define CFM_IPS_FLASH_ADDR 		0x44000000			// DIRECCIÓN DE INICIO DE ALMACENAJE EN FLASH.
#define FLASH_START_ADDRESS	 	CFM_IPS_FLASH_ADDR	// DIRECCIÓN DE INICIO DE ALMACENAJE EN FLASH.


// VARIABLES DEL PROGRAMA

// de recepcion del lector
unsigned char R_BYTE_INI=0, R_BYTE_COM=0, R_BYTE_ERROR=0, R_BYTE_CHKSUM=0, R_BYTE_END=0;
unsigned char R_BYTE_11=0, R_BYTE_12=0, R_BYTE_13=0, R_BYTE_14=0;
unsigned char R_BYTE_21=0, R_BYTE_22=0, R_BYTE_23=0, R_BYTE_24=0;
unsigned char CHECKSUM=0, SALIR=0, MUESTRA=0, ID_USUARIO=0, R_BYTE_MUESTA=0;

unsigned long CONTRASTE=0, CONTR1=0, CONTR2=0, CONTR3=0, CONTR4=0;
unsigned long LONG_MUES=0, LONG_M1=0, LONG_M2=0;

unsigned char CHECKSUM1=0, usuarios_guardados=0, timer=0x3A, GUARDA=0; // variables auxiliares X_REDO=0*** para probar error d desenriptacion
int AUX_MOD=0;// int con signo que se usa para hacer modulo 255 al desencriptar sobre el vector del templete
unsigned short int AUX_SAVE = 0; //*** variable que se guarda en una direccion de la ram como huella desencriptada
uint8 temp=0;

/* DECRLARACION DE VARIABLE */

// Enteros
unsigned int i, j, LONG_KEY, ITE1, ITE2, cambio, S, en_dos; //  POS
unsigned int k, PREG_NUM, SALTAR, CONVERTIR, DECIMAL, inc, a=0, X_REDO=0; //, BUS
unsigned int DATO=0,b=0;// , POS_R=0; **
unsigned int V_SUM;// para evitar modulo
unsigned short int POS_R=0, BUS=0, POS=0, NO_rep; //** variables de 2Bytes

// Caracter
char CAR_CI, OP_CI, CAR_SUM, EN, CARACT;

// Llave del sistema criptografico: 32 digitos hexadecimales.
char KEY[32] = 
{
    "11223344556677889900AABBCCDDEEFF"	
};

// Long dobles
long double SUM_KEY=0, VAL_KEY, VAL_16, VAL_X, VAL_Y, CAL_CI, VAL_A, VAL_B, VAL_C, VAL_D;
long double V_PA2, V_CI2, V_PA1, V_CI1, a2_L1, x2_L1, a1_L1, x1_L1, MOD_SUM1, MOD_SUM2, TXT_SUM, SUM_MEN;
long double MAP_X2, MAP_X1, ANT_X2, ANT_X1, X_DIFU, X_ALTE, Y1=0, d=0, Y2=0; //X_ORIG , NO_rep

/* PARA ALMACENAMIENTO DE DATOS EN MEMORIA FLASH */
unsigned int DATO_RECU = 0,DIF_PERM_P=0; 
unsigned int DIRE_REP = 0x00053000, DIRE_ENC=0x00050000, DIREC_INICIAL=0, DIREC_FINAL=0;


void cpu_pause (unsigned long);		// DECLARACION DE FUNCION A USAR
void ini_rs232();

void check_finger();
void get_image();
void process_image();
void sample_to_RAM(unsigned char NO_Muestra);
void template_to_MC();
void templete_a_FAM_RAM();
void verificacion();

/* FUNCIONES RTC */
void INIC_RTC(void);							// INICIALIZA RELOJ DE TIEMPO REAL.
//void SET_REGISTROS_RTC(void);

// Estructuras

struct DATOS_X {
 unsigned char DATO_XX[2080]; // guarda templete original // 1 byte
 };
struct DATOS_X DATOSX;

/*
struct DATO_dup {
 unsigned int DATOS_crip[2080]; // respalda el criptograma // 1 byte
 };
struct DATO_dup DATO_t; // para respaldar huella encriptada
*/

struct POSICION {
 unsigned short int CAR[950]; // guarda posicion //* short usa 2 bytes en lugar de 4
 };
struct POSICION POSS;


// DECLARACION DE FUNCIONES
void I2C_INICIALIZACION(void);
void I2C_ENVIA_DATO(unsigned char DATO);
void I2C_POSICION_DATO(unsigned char ACCION);
void I2C_POSICION_CARACTER(unsigned char COLUMNA, unsigned char FILA);
void I2C_START();
void I2C_STOP();
void I2C_LLAMAR_ESCLAVO();
void I2C_LLAMAR_LEERESCLAVO();
void I2C_FUCNIONES_LCD(unsigned char FUNCION);

// FUNCIONES CONFIGURACION LCD 
void START_GPO();
void BUZZER_ON_OFF(unsigned char ACTIVIDAD);
void LED_AMARILLO_ON_OFF(unsigned char ACTIVIDAD1);
void LED_ROJO_ON_OFF(unsigned char ACTIVIDAD2);
void LED_VERDE_ON_OFF(unsigned char ACTIVIDAD3);
void RELAY_ON_OFF(unsigned char ACTIVIDAD4);

/* FUNCIONES DE VISUALIZACION EN LCD */
void VIS_INICIO();
//void VIS_SW_MENU_1(void);		// VISUALIZA FUNCION DEL SWITCHES DENTRO DE MENÚ.
void VIS_MENU_INICIAL(void); 	// visualiza menu para iniciar programa
void VIS_INICIO_AGREG_USU(void);// VISUALIZA INICIO CUANDO SE DE DE ALTA UN USUARIO
void VIS_ERROR();				// VISUALIZA MENSAJE DE ERROR, que algo salio mal en el codigo
//void VIS_DETECTAR_HUELLA();
void VIS_ACEPTADO();    // visualiza que el usuario puede abrir la puerta
void VIS_REGISTRADO();  // visualiza que se registraron con exito
void VIS_RECHAZADO();   // visualiza mensaje de que el usuario es rechazado
void VIS_LEYENDO(); // visualiza mensaje espere antes de iniciar la encriptacion
//void VIS_OK();  // OK
void VIS_ENCRIPTANDO(); // Encriptando
void VIS_DESENCRIPTANDO(); // Desencriptando
void VIS_VERIFICAR(); // mensaje para abrir la puerta

/* FUNCIONES DE ENCRIPTADO */
void SUM_TXT(); // Sumar todos los caracteres segun ASCII, No hace falta convertir a decimal (es automatico)
void LLAVE();  	// Dividir llave en subsecciones para CI y PC.
void DET_CI(); 	// Determinar valores para Mapeo Log 2D a partir de la llave 32 dig Hex


/* FUNCIONES PARA MEMORIA FLASH */
void INIC_MEMORIA_FLASH(void);			// INICIALIZA MEMORIA FLASH.
void ESCRIBIR_DATO_FLASH(unsigned int DIREC_MEMORIA, unsigned int DATO_F);// ESCRIBE DATO DE 32 BITS EN FLASH.
void RECUPERAR_DATO_FLASH(unsigned int DIREC_MEMORIA);// RECUPERA UN DATO DE FLASH DE LA DIRECCIÓN INDICADA.
void BORRAR_DATO_FLASH(unsigned int DIREC_MEMORIA);// BORRA UN DATO DE FLASH DE LA DERECCIÓN INIDICADA.
void BORRAR_DATOS_FLASH(void);					// BORRA MAS DE UN DATO DE LA FLASH.
void ENCRIPTAR(void); // hace todos los procesos necesarios para encriptar templete y guardar en flash
void DESENCRIPTAR(void); // hace el proceso de desencriptar templete de la flash y guarda en RAM


/* ESTRUCTURAS */

// Sub-llaves
struct A_KEY { char CAR[8]; }; struct  A_KEY  SEC_A; //  A (8 dig hex)
struct B_KEY { char CAR[8]; }; struct  B_KEY  SEC_B; //  B (8 dig hex)
struct C_KEY { char CAR[8]; }; struct  C_KEY  SEC_C; //  C (8 dig hex)
struct D_KEY { char CAR[8]; }; struct  D_KEY  SEC_D; //  D (8 dig hex)

// Almacenamiento de Datos caoticos
struct CAOSX1 { double CAR[2072]; }; struct  CAOSX1  DAT_CAOTX1; // [XX] Indica # datos caoticos X a almacenar // 8bytes
struct CAOSX2 { double CAR[2172]; }; struct  CAOSX2  DAT_CAOTX2; // [XX] Indica # datos caoticos X a almacenar // 8bytes

// Permutacion:
struct PERM_SEC { unsigned short int CAR[2072]; }; struct  PERM_SEC  PERM; // Vector para Permutacion
// Para Numeros que no estan en PERM
struct NO_ESTA { unsigned short int CAR[950]; }; struct  NO_ESTA  NUM_NOP; // Vector para Numeros que no estan

// INICIO DEL PROGRAMA

int main(void)
{
                       
// INICIALIZACION DE LEDS DE LA TARJETA

// Enable signals as GPIO 

MCF_GPIO_PTCPAR = 0
|MCF_GPIO_PTCPAR_DTIN0_GPIO
|MCF_GPIO_PTCPAR_DTIN1_GPIO
|MCF_GPIO_PTCPAR_DTIN2_GPIO
|MCF_GPIO_PTCPAR_DTIN3_GPIO; 

MCF_GPIO_DDRTC = 15; // hace los puertos de salida
MCF_GPIO_PORTTC  = 0 ; // ENCIEDO LOS LEDS QUE QUIERO.

// configuracion para interupcion 
MCF_GPIO_PNQPAR = 0x4404; // LO QUE HACE ES ACTIVAR SALIDAS DE INTERUPCION, antes 404
MCF_EPORT_EPPAR = 0x4404; // HACE LOS INTERRUPCIONES POR flanco de subida, antes 404
MCF_EPORT_EPDDR = 0x102;  // EPORT DE ENTRADA antes 22
MCF_EPORT_EPIER = 0x102;  // HABILITA INTERRUPCIONES antes 22

/* INICIALIZA MODULO DE MEMORIA FLASH */
INIC_MEMORIA_FLASH();
BORRAR_DATOS_FLASH(); // BORRAR DATOS


MCF_CLOCK_SYNCR = 0x4107; // RELOJ 8MHZ * 6 = 48MHZ 
ini_rs232(); 	// Inicializacion RS-232 BAUDRATE ACTUAL = 115200 48MHZ

INIC_RTC(); // inicializa el reloj de tiempo real para que funcione con esa frecuancia de reloj

//MCF_CLOCK_SYNCR = 0x3007; // RELOJ 8MHZ * 10 = 80MHZ
//ini_rs232(0x41); 	// Inicializacion RS-232 BAUDRATE ACTUAL = 38400 80MHZ	

// INICIALIZA I2C	
I2C_INICIALIZACION();   // INICIALIZA PUERTOS, I2C1.
I2C_START();			// ENVIAR SENAL DE START.
I2C_LLAMAR_ESCLAVO();	// ENVIAR DIRECCION DEL ESCLAVO.
I2C_FUCNIONES_LCD(0x99); //	cambia el brillo de la pantalla
I2C_FUCNIONES_LCD(0x58);	// BORRAR PANTALLA.
START_GPO(); // CONFIGURA GPO DE LCD
VIS_INICIO();  // Visualiza en LCD informacion de inicio
VIS_MENU_INICIAL(); // muestra opciones de menu en la pantalla
	
	
	//** prueba las salidas GPIO de la pantalla LCD **
	
	// led amarillo
	LED_AMARILLO_ON_OFF(0x57); // enciende
	BUZZER_ON_OFF(0x57); // enciende
	cpu_pause(100000);// pausa 100 ms
	BUZZER_ON_OFF(0x56); // apaga
	LED_AMARILLO_ON_OFF(0x56); // apaga
	
	// led rojo
	LED_ROJO_ON_OFF(0x57); // enciende
	BUZZER_ON_OFF(0x57); // enciende
	cpu_pause(100000);// pausa 100 ms
	BUZZER_ON_OFF(0x56); // apaga
	LED_ROJO_ON_OFF(0x56); // apaga
		
	// led verde
	LED_VERDE_ON_OFF(0x57); // enciende
	BUZZER_ON_OFF(0x57); // enciende
	cpu_pause(100000);      // pausa 100 ms
	BUZZER_ON_OFF(0x56); // apaga
	LED_VERDE_ON_OFF(0x56); // apaga
	
	// cerrojo electronico
//	RELAY_ON_OFF(0x57); // enciende
//	cpu_pause(100000);  // pausa 100 ms
//	RELAY_ON_OFF(0x56); // apaga
	cpu_pause(1000000);  // pausa 100 ms	
	
	    	/*  INICIO DE PROGRAMA PRINCIPAL */				
for(;;) 
{	
	// prgunta si han presionado SW1
	if(MCF_EPORT_EPFR & MCF_EPORT_EPFR_EPF1) // SW1
	{
		LED_VERDE_ON_OFF(0x57);
		BUZZER_ON_OFF(0x57); // enciende buzzer
		LED_VERDE_ON_OFF(0x56);
		LED_VERDE_ON_OFF(0x57);
		LED_VERDE_ON_OFF(0x56);
		LED_VERDE_ON_OFF(0x57);
		LED_VERDE_ON_OFF(0x56);
		BUZZER_ON_OFF(0x56); // apaga buzzer
		MCF_GPIO_PORTTC  = 1 ;
		timer=0x3A; // 0x30=0 0x3A=10 en ASCII del LCD
		R_BYTE_ERROR=0; // datos obsoletos
		VIS_INICIO_AGREG_USU(); // Muestra en pantalla instrucciones de colocar dedo en lector
		while(R_BYTE_ERROR != 0x40 && timer>=0x30 ) // escanea hasta encontrar huella a menos que pasen mas de 10 degundos
		{
			cpu_pause(200000); // 200ms para no saturar el bus
			check_finger();
			if(MCF_RTC_RTCISR &  MCF_RTC_RTCISR_1HZ) // cada segundo actualiza timer LCD.
			{
				timer=timer-1;
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				I2C_POSICION_CARACTER(16,4); // PARA POSICIONAR CARACTER
				I2C_ENVIA_DATO(timer);	 // muestra en LCD contador RTC
				MCF_RTC_RTCISR = 0xFFFFFFFF;// limpia bandera RTC
			}
		} // Fin del while para encontrar dedo
		if (R_BYTE_ERROR==0x40) // en caso de que no encuentre huella en 10 segundos no entra
		{
			I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
			I2C_POSICION_CARACTER(3,3); //direcciona el puntero
			VIS_LEYENDO(); // mensaje a la pantalla
			for(MUESTRA=0;MUESTRA<=2;MUESTRA++) // toma tres muestras
			{
				MCF_GPIO_PORTTC  = 2 ; // muestro estado en leds
				get_image();
				LED_VERDE_ON_OFF(0x57);
				LED_VERDE_ON_OFF(0x56);
				LED_VERDE_ON_OFF(0x57);
				LED_VERDE_ON_OFF(0x56);
				LED_VERDE_ON_OFF(0x57);
				LED_VERDE_ON_OFF(0x56);
				if(R_BYTE_ERROR == 0x40)
				{
					MCF_GPIO_PORTTC  = 1 ;
					process_image();
					sample_to_RAM(MUESTRA);
				}
				else
				{
					MUESTRA--;
				}
			} //tiene tres muestras en la RAM del lector de huellas
			
			MCF_GPIO_PORTTC  = 4 ; // indica estado en leds
			template_to_MC(); // 2072 bytes del FAM al microcontrolador
			I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
			I2C_POSICION_CARACTER(3,2); 
			VIS_ENCRIPTANDO();
			
	    	ENCRIPTAR();// encripta templete y guarda en memoria flash del micro
	    	
			LED_VERDE_ON_OFF(0x57);
			LED_VERDE_ON_OFF(0x56);
			LED_VERDE_ON_OFF(0x57);
			LED_VERDE_ON_OFF(0x56);
			
			usuarios_guardados=usuarios_guardados+1; // para saber cuantos perfiles de usuariso hay guardados
			I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
			I2C_POSICION_CARACTER(2,3); 
			VIS_REGISTRADO(); // mensaje de que fue registrado
			
			BUZZER_ON_OFF(0x57);
			cpu_pause(40000);
			BUZZER_ON_OFF(0x56);
			cpu_pause(40000);
			BUZZER_ON_OFF(0x57);
			cpu_pause(40000);
			BUZZER_ON_OFF(0x56);
			
			cpu_pause(2000000); // Muestra el mensaje en pantalla durante 1 segundos antes de regresar al menu de espera
			MCF_GPIO_PORTTC  = 8 ; //indica que ha terminado	
		} //templete procesado y guardado en memoria RAM del microcontrolador
		else
		{
			I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
			I2C_POSICION_CARACTER(1,1); 
			VIS_ERROR(); // si no encontro huella en menos de 10 segundos manda mensaje de error
			cpu_pause(1000000);
		}
		// Evito mandar datos a la pantalla mientras el micro espera que pulsen los botones
		I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
		VIS_INICIO();  			 // Visualiza en LCD informacion de inicio
		VIS_MENU_INICIAL(); 	 // muestra opciones de menu en la pantalla
		MCF_EPORT_EPFR |= MCF_EPORT_EPFR_EPF1; 	// limpio interrupciones del boton
	} // termina el IF de que han presionado el boton SW1
	
	// Para el SW2 ya se debe tener un usuario guardado antes, si no mando un mensaje de error
	if (MCF_EPORT_EPFR & MCF_EPORT_EPFR_EPF7) // Verificar que el usuario que desea entrar esté registrado
	{
		LED_VERDE_ON_OFF(0x57);
		BUZZER_ON_OFF(0x57); // enciende buzzer
		LED_VERDE_ON_OFF(0x56);
		LED_VERDE_ON_OFF(0x57);
		LED_VERDE_ON_OFF(0x56);
		LED_VERDE_ON_OFF(0x57);
		LED_VERDE_ON_OFF(0x56);
		BUZZER_ON_OFF(0x56); // apaga buzzer
		if(usuarios_guardados>0) // primero hay que ver que si hay usuarios en el sistema
		{
			MCF_GPIO_PORTTC  = 1;
			timer=0x3A; // 0x3A-1=0x39[ASCII] = 9
			R_BYTE_ERROR=0; //borra dato obsoleto
			VIS_VERIFICAR(); // Muestra en pantalla instrucciones de colocar dedo en lector
			while(R_BYTE_ERROR != 0x40 && timer>0x30) // escanea hasta encontrar huella a menos que pasen mas de 10 segundos
			{
				cpu_pause(200000); // 200ms para no saturar el bus
				check_finger();
				if(MCF_RTC_RTCISR &  MCF_RTC_RTCISR_1HZ) // cada segundo actualiza timer LCD.
				{
					timer=timer-1;
					I2C_POSICION_CARACTER(16,4); // PARA POSICIONAR CARACTER
					I2C_ENVIA_DATO(timer);	     // muestra en LCD contador RTC
					MCF_RTC_RTCISR = 0xFFFFFFFF; // limpia bandera RTC
				}
			} // Fin del while para encontrar dedo
			
			if(R_BYTE_ERROR==0x40) // solo si detecta huella desencripta a comparar
			{
				I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
				I2C_POSICION_CARACTER(3,2); 
	    		//VIS_LEYENDO(); // mensaje a la pantalla antes de desencriptar
	    		VIS_DESENCRIPTANDO();
				DESENCRIPTAR(); // lee flash y desencripta templete***************************************
				// El templete recuperado SE GUARDA EN EL MISMO VECTOR que cuando de lee desde el modulo al micro con "template_to_MC"
				// Envia templete de la RAM del micro a la RAM del lector de huellas
			}
			if(R_BYTE_ERROR==0x40) // solo si tiene nuevo templete lo guarda
			{
				templete_a_FAM_RAM(); // envia templete desencriptado a la RAM1 del FAM
			}
			else
			{
				I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
				I2C_POSICION_CARACTER(1,1); 
				VIS_ERROR();// si no encontro huella en 10 segundos manda mensaje de error
				cpu_pause(1000000);
			}
			// verifica muestra actual con templete en RAM1 del FAM	
			MCF_GPIO_PORTTC = 1;
			if(R_BYTE_ERROR == 0x40) // si no detectó huella en 10 segundos no va entrar
			{
				get_image(); // captura una imagen
				MCF_GPIO_PORTTC = 3;
				while(R_BYTE_ERROR == 0x40) // si no procesa bien la imagen lo vuelve a hacer
				{
					process_image(); // convierte la  imagen en una muestra
					MCF_GPIO_PORTTC = 7;
					if(R_BYTE_ERROR == 0x40) // entra solo cuando la muestra esté lista para comparar
					{	
						verificacion(); //funcion que hace la verificacion entre la muestra actual y el templete de la RAM1
										// debe haber primero una muestra en la RAM1 del FAM para que funcione
						
						if(R_BYTE_ERROR == 0x40) // si da OK es que si es la misma persona
						{
							MCF_GPIO_PORTTC = 15;
							I2C_FUCNIONES_LCD(0x58); //BORRAR PANTALLA
							I2C_POSICION_CARACTER(6,3); 
							VIS_ACEPTADO(); //Mensaje de aceptado
							R_BYTE_ERROR=0; //para no volver a procesar imagen
							RELAY_ON_OFF(0x57); //energiza relay para abrir el cerrojo electronico
							LED_VERDE_ON_OFF(0x57);//led OK
							BUZZER_ON_OFF(0x57); // enciende sonido
							cpu_pause(3000000); //espera 3 segundos con el mensaje de aceptado
							LED_VERDE_ON_OFF(0x56); //apaga led OK
							RELAY_ON_OFF(0x56); //apaga relay
							BUZZER_ON_OFF(0x56); // apaga sonido
							
						} // fin de mostrar resultado
						if(R_BYTE_ERROR == 0x45)
						{
							MCF_GPIO_PORTTC = 1; // usuario desconocido
							I2C_FUCNIONES_LCD(0x58); // borra pantalla
							I2C_POSICION_CARACTER(2,3); // coloca cursor
							VIS_RECHAZADO(); // rechazado
							LED_ROJO_ON_OFF(0x57);
							BUZZER_ON_OFF(0x57);
							cpu_pause(40000);//50ms
							BUZZER_ON_OFF(0x56);
							cpu_pause(40000);//50ms
							BUZZER_ON_OFF(0x57);
							cpu_pause(40000);//50ms
							BUZZER_ON_OFF(0x56);
							cpu_pause(2000000); // espera 2 segundos con el mensaje de error
							LED_ROJO_ON_OFF(0x56);
						} // Mostrar resultado
					} // Verificacion
				} // Procesar imagen
			} // Capturar imagen
			else // en caso de que no se envie el templete al lector para hacer la comparacion despues
			{
				MCF_GPIO_PORTTC=12; // no envió el templete a verificar
				I2C_FUCNIONES_LCD(0x58);
				I2C_POSICION_CARACTER(1,1);
				VIS_ERROR(); //
				cpu_pause(1000000); // espera con el mensaje de error
			}	
		} // IF que pregunta si hay usuarios guardados en la flash del microcontrolador
		else
		{
			MCF_GPIO_PORTTC=11; // hacer algo para indicar que no hay usuarios asi que no se verifica nada y evita errores
			I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
			I2C_POSICION_CARACTER(1,1);		// PARA POSICIONAR CARACTER // 
			VIS_ERROR(); // NO HAY USUARIOS REGISTRADOS
			cpu_pause(1000000); // espera con el mensaje de error
		}
		// antes de salir al modo de espera, cambio la pantalla
		// Evito mandar datos a la pantalla mientras el micro espera que pulsen los botones
		RELAY_ON_OFF(0x56);// desenergiza relay para asegurar el cerrojo
		I2C_FUCNIONES_LCD(0x58); // BORRAR PANTALLA.
		VIS_INICIO();  			 // Visualiza en LCD informacion de inicio
		VIS_MENU_INICIAL(); 	 // muestra opciones de menu en la pantalla
		MCF_EPORT_EPFR |= MCF_EPORT_EPFR_EPF7; // limpia bandera de interrupcion al final para que no se repita el proceso
	} //if de que han presionado boton SW2	
}// for infinito
}// main de configuracion

		/* FUNCION DE RETARDO PARA CUALQUIER TIEMPO */


void cpu_pause(unsigned long usecs)
{
    MCF_DTIM3_DTRR = (usecs - 1);			// ACTIVAR DTIM3.
    MCF_DTIM3_DTER = MCF_DTIM_DTER_REF;
    MCF_DTIM3_DTMR = 0						// SELECCIONAR RELOJ.
        | MCF_DTIM_DTMR_PS(SYSTEM_CLOCK_KHZ / 1000)
        | MCF_DTIM_DTMR_ORRI
        | MCF_DTIM_DTMR_FRR
        | MCF_DTIM_DTMR_CLK_DIV1
        | MCF_DTIM_DTMR_RST;

    while ((MCF_DTIM3_DTER & MCF_DTIM_DTER_REF) == 0) 
    {};										// ESPERAR LOS MICROSEGUNDOS ESPECIFICADOS.

    MCF_DTIM3_DTMR = 0;						// DESACTIVAR TIMER.
    
}


void ini_rs232()
{
	MCF_UART_UCR(0) = MCF_UART_UCR_RESET_TX; // Reset Transmitter
	MCF_UART_UCR(0) = MCF_UART_UCR_RESET_RX; // Reset Receiver
	MCF_UART_UCR(0) = MCF_UART_UCR_RESET_MR; // Reset Mode Register
	MCF_UART_UMR(0) = (0       // No parity, 8-bits per character
		| MCF_UART_UMR_PM_NONE
		| MCF_UART_UMR_BC_8 );
	MCF_UART_UMR(0) = (0  // No echo or loopback, 1 stop bit
		| MCF_UART_UMR_CM_NORMAL
		| MCF_UART_UMR_SB_STOP_BITS_1);
	MCF_UART_UCSR(0) = (0  // Set Rx and Tx baud by SYSTEM CLOCK
		| MCF_UART_UCSR_RCS_SYS_CLK
		| MCF_UART_UCSR_TCS_SYS_CLK);
	MCF_UART_UIMR(0) = 0; // Mask all UART interrupts
	MCF_UART_UBG1(0) = 0;  	  // Calculate baud settings
	MCF_UART_UBG2(0) = 0x0D;  // CON RELOJ DE 48 MHZ
	MCF_UART_UCR(0) = (0  // Enable receiver and transmitter
		| MCF_UART_UCR_TX_ENABLED
		| MCF_UART_UCR_RX_ENABLED);		
}

void INIC_RTC(void)
{
		MCF_RTC_RTCCTL = 128;			// ABILITO RELOJ TIEMPO REAL.
		MCF_RTC_RTCIENR = 62;			// HABILITA INTERUPCIONES EN SEG, MIN, HORA, DIA.
		MCF_CLOCK_RTCCR = 0x57; 		// SELECCIONA FUENTE DE RELOJ OSCILADOR RTC
     
	    MCF_RTC_RTCGOCU = 0x00000000; 	// DIVIDE EL RELOJ ENTRE 8191 PARA OBTENER 1 HZ
	    MCF_RTC_RTCGOCL = 0x00001FFF;   // 1FFF PARA UN SEG. 1aff

	 	MCF_RTC_RTCISR = 0xFFFFFFFF; 	// LIMPIAR BANDERAS DE INTERRUPCION

		return;
}

void check_finger()
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 
	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
		 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x4B; /* Send the character */ // COMANDO
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // FLAG/ERROR    
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x4B;
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 


	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;
	

	return;
	
}

/********************************************************************/

/********************************************************************/
/*
 * Envia y recibe: captura huella
 	retorna:
 		COM: NO DOSIS
 		PARAM1: CONTRASTE
 		PARAM2: PIXELES BLANCOS
 		ERROR: 0x40 OK; 0x41 NO IMAGEN
 * 
 */
void get_image()
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0;

	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
		 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x49; /* Send the character */ // COMANDO
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // FLAG/ERROR    
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x49;
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 

//	cpu_pause(10000);
		
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;
	
	return;
	
	
}

/********************************************************************/

/********************************************************************/
/*
 * Envia y recibe: procesa la huella
 	retorna:
 		ERROR: 0x40 ok; 0x42 Poca calidad; 0x43 pocos puntos. 
 * 
 */
void process_image()
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 

	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
		 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x50; /* Send the character */ // COMANDO
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // FLAG/ERROR    
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x50;    
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 
	
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;
	return;
	
}


/********************************************************************/

/********************************************************************/
/*
 * Envia y recibe: solicita muestra
 	retorna:
 		PARAM1: No de muestra 0-9
 		ERROR: 0x40
 */

void sample_to_RAM(unsigned char NO_Muestra)
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 

	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
		 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x53; /* Send the character */ // COMANDO
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = NO_Muestra; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // FLAG/ERROR    
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x53 + NO_Muestra;
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 
	
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES	
	
		// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;
	
	return;
	
	
}



void template_to_MC()
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 
	i=0, LONG_M1=0 ,LONG_M2=0 ; //para la transmision de los 2072bytes

	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
		 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x54; /* Send the character */ // COMANDO
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x01; /* Send the character */ // FLAG/ERROR    
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x54 + 0x01;
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 
	
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	CHECKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;

	LONG_M1 = LONG_M1 | R_BYTE_21;
	LONG_M2 = (LONG_M2 | R_BYTE_22) << 8;

	LONG_MUES = LONG_M2 | LONG_M1;

	for(i=0;i<LONG_MUES;i++)
	{	
		while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
		DATOSX.DATO_XX[i]=MCF_UART0_URB;
	}
	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;
	
	
	return;
}

void templete_a_FAM_RAM()
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 
	i=0, LONG_M1=0 ,LONG_M2=0, CHECKSUM1=0; //para la transmision de los 2072 bytes

//	for(i=0;i<LONG_MUES;i++) // 2072 bytes
//	{	
//		DATO_t.DATOS_prueb[i]=DATOSX.DATO_XX[i];
//	}
	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
	
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x55; /* Send the character */ // COMANDO para enviar templete del micro a FAM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x01; /* Send the character */ // PARAM1 para guardar en RAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x18; /* Send the character */ // PARAM2 longitud
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x08; /* Send the character */ // PARAM2 longitud
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // FLAG   
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x55 + 0x01 + 0x08 + 0x18; // inicio + comando + param1 + param2 + FLAG
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 
	
	// Ahora le enviamos el templete
		for(i=0;i<LONG_MUES;i++) // 2072 bytes
	{	
		while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){}; // espera que se desocupe el bus
		MCF_UART0_UTB=DATOSX.DATO_XX[i]; // envia un byte por el buz del UART *** cambiar el vector por el desencriptado
		//MCF_UART0_UTB=DATO_t.DATOS_prueb[i];
		CHECKSUM1=CHECKSUM1+DATOSX.DATO_XX[i]; // calcula el checksum
		//CHECKSUM1=CHECKSUM1+DATO_t.DATOS_prueb[i]; 
	}
	while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};
	MCF_UART0_UTB = CHECKSUM1; /* Send the character */  // CHECK SUM; suma todo el vector del templete y lo divide entre 256
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE
    
	// despues de enviar el templete, ahora nos envia info el FAM para ver si lo recibió bien
	
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;

	return;
	
}


void verificacion() // para verificar que la muestra actual es igual que el templete guardado en la RAM 1 del FAM
{
	CHECKSUM=0;R_BYTE_INI=0;R_BYTE_COM=0;R_BYTE_11=0;R_BYTE_12=0;R_BYTE_13=0;R_BYTE_14=0; // Inicializaciones 
	R_BYTE_21=0;R_BYTE_22=0;R_BYTE_23=0;R_BYTE_24=0;R_BYTE_ERROR=0;R_BYTE_CHKSUM=0;R_BYTE_END=0; 

	// TRANSMITIR PAQUETE AL LECTOR DE HUELLA 13 BYTES
	
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x40; /* Send the character */ // INICIO PAQUETE
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x52; /* Send the character */ // COMANDO para comparacio en FAM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x01; /* Send the character */ // PARAM1 compara con templete en RAM 1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM1 
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */ // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x00; /* Send the character */  // PARAM2
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x02; /* Send the character */ // FLAG/ERROR   para comparar con templete en guardado en la RAM 
    // Determinar CHECK_SUM 
	CHECKSUM = 0x40 + 0x52 + 0x01 + 0x02;
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = CHECKSUM; /* Send the character */  // CHECK SUM
    while (!(MCF_UART_USR(0) & MCF_UART_USR_TXRDY)){};/* Wait until space is available in the FIFO */
    MCF_UART0_UTB = 0x0D; /* Send the character */ // FIN PAQUETE 
	
	// RECIBIR DEL LECTOR DE HUELLA 13 BYTES
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_INI = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_COM = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_11 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_12 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_13 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_14 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_21 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_22 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_23 = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_24 = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_ERROR = MCF_UART0_URB;
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_CHKSUM = MCF_UART0_URB;	
	while (!(MCF_UART_USR(0) & MCF_UART_USR_RXRDY)){};
	R_BYTE_END = MCF_UART0_URB;

	return;
	
}


void I2C_INICIALIZACION(void)
{
	MCF_GPIO_PUCPAR = 0						// ABILITO SALIDA GPIO PUERTO UC PARA SENALES I2C.
	 |MCF_GPIO_PUCPAR_URTS2_SDA1			// I USE PIN 10 AND 12, BACK IN TARGET (40 CONECTOR)
	 |MCF_GPIO_PUCPAR_UCTS2_SCL1;			// CONECTOR 10 = PORT UC GPIO SDA_1
											// CONECTOR 12 = PORT UC GPIO SCL_1	

	MCF_I2C1_I2FDR = 0x3B;					// SELECCIONAR FRECUENCIA DE 46 KHz. 48MHz/1024. 
	MCF_I2C1_I2CR = 0 						// ACTIVAR EL MODULO I2C.
	| MCF_I2C_I2CR_IEN;

	if( MCF_I2C1_I2SR & MCF_I2C_I2SR_IBB)	// SI EL BUS ESTA OCUPADO, MANDA UN STOP Y ACTIVA MODULO.
	{
		MCF_I2C1_I2CR = 0;					// LIMPIA REGISTRO DE CONTROL.		
		MCF_I2C1_I2CR = MCF_I2C_I2CR_IEN |	// ACTIVAR MODULO Y ENVIAR UN START.
					    MCF_I2C_I2CR_MSTA;
	  
	  	temp = MCF_I2C1_I2DR;				// BASURA
		MCF_I2C1_I2SR = 0;					// LIMPIAR REGISTRO DE ESTADO.
		MCF_I2C1_I2CR = 0;					// LIMPIAR REGISTRO DE CONTROL.		
		MCF_I2C1_I2CR = 0 					// ACTIVAR EL MODULO OTRA VES.
		| MCF_I2C_I2CR_IEN;	
	}

	MCF_I2C1_I2CR |= MCF_I2C_I2CR_IIEN;		// ACTIVAR INTERRUPCIONES.
	
	return;
}

void I2C_ENVIA_DATO(unsigned char DATO)
{
    MCF_I2C1_I2DR = DATO;  							// ENVIAR DATO.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	return;
	
}

/* 
DESCRIPCION DE COMANDOS PARA 'ACCION':

	  LIMPIAR PANTALLA				0x58
	  CAMBIAR INICIO DE PANTALLA	0x40
	  POSICION DE CURSOR			0x47 [columna][fila]
	  POSICION HOME					0x48
	  MOVER CURSOR ATRAS			0x4C
	  MOVER CURSOR A DELANTE		0x4D
	  ACTIVAR LINEA BAJA DE CURSOR	0x4A
	  DES LINEA BAJA DE CURSOR		0x4B
	  PARPADEAR CURSOR ON			0x53
	  PARPADEAR CURSOR OFF			0x54
*/

void I2C_POSICION_DATO(unsigned char ACCION)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
   	MCF_I2C1_I2DR = ACCION;  						// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	return;
}


void I2C_POSICION_CARACTER(unsigned char COLUMNA, unsigned char FILA)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = 0x47;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = COLUMNA; 
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = FILA; 
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	return;
}

void I2C_START()
{
	MCF_I2C1_I2CR |= MCF_I2C_I2CR_MTX;				// MODO ENVIAR.
	MCF_I2C1_I2CR |= MCF_I2C_I2CR_MSTA;				// GENERAR START.

	return;
}


void I2C_STOP()
{
    MCF_I2C1_I2CR &= ~MCF_I2C_I2CR_MSTA;			// GENERAR SENAL DE STOP.

    return;
}


void I2C_LLAMAR_ESCLAVO()
{
	MCF_I2C1_I2DR = ESCLAVO; 						// ENVIAR DIRECCION DEL ESCLAVO.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.

	return;
}


void I2C_LLAMAR_LEERESCLAVO()
{
	MCF_I2C1_I2DR = LEER_ESCLAVO; 						// ENVIAR DIRECCION DEL ESCLAVO.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.

	return;
}

/* 
DESCRIPCION DE COMANDOS PARA 'FUNCION':
	
		ENCENDER LCD		0x42[MINUTOS]
		APAGAR LCD			0x46
		BRILLO				0x99[BRILLO]
		GUARDAR BRILLO		0x98[BRILLO]
		CONTRASTE			0x50[CONTRASTE]
		GUARDAR CONTRASTE	0x91[CONTRASTE]
*/

void I2C_FUCNIONES_LCD(unsigned char FUNCION)
{
		MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO.
	    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
		MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	   	MCF_I2C1_I2DR = FUNCION;  						// ENVIAR ACCION.
	    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
		MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.

		if(MCF_I2C1_I2DR == 0x42)
		{
			MCF_I2C1_I2DR = 0;  							// ENCENDER LCD.
		    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
			MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;	
		}

		if(MCF_I2C1_I2DR == 0x99 || MCF_I2C1_I2DR == 0x98)
		{
			MCF_I2C1_I2DR = BRILLO_LCD;  					// PONER BRILLO A LCD.
		    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
			MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;		
		}
		
		if(MCF_I2C1_I2DR == 0x50 || MCF_I2C1_I2DR == 0x91)
		{
			MCF_I2C1_I2DR = CONTRASTE_LCD;  				// PONER CONTRASTE A LCD.
		    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
			MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;	
		}
	return;
}


void START_GPO()
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = 0xC3;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = 0x05; // PODER USAR GPO1 (buzzer), 2 (led verde), 3 (led amarillo), 4 (led rojo) 
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = 0x00; // OFF todos
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	cpu_pause(100000);	
	return;
}

void BUZZER_ON_OFF(unsigned char ACTIVIDAD)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = ACTIVIDAD;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = 0x04; // PODER USAR GPO1 
	cpu_pause(40000);
	return;
}

void LED_AMARILLO_ON_OFF(unsigned char ACTIVIDAD1)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = ACTIVIDAD1;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = 0x02; // PODER USAR GPO1 
	cpu_pause(20000);
	return;
}

void LED_ROJO_ON_OFF(unsigned char ACTIVIDAD2)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = ACTIVIDAD2;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = 0x03; // PODER USAR GPO1 
	cpu_pause(20000);
	return;
}

void LED_VERDE_ON_OFF(unsigned char ACTIVIDAD3)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = ACTIVIDAD3;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = 0x01; // PODER USAR GPO1 
	cpu_pause(20000);
	return;
}

void RELAY_ON_OFF(unsigned char ACTIVIDAD4)
{
	MCF_I2C1_I2DR = 0xFE;							// SE DEBE ENVIAR PRIMERO
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.
	MCF_I2C1_I2DR = ACTIVIDAD4;  							// ENVIAR ACCION.
    while( !(MCF_I2C1_I2SR & MCF_I2C_I2SR_IIF )){};	// ESPERAR HASTA COMPLETAR ENVIO.
	MCF_I2C1_I2SR &= ~MCF_I2C_I2SR_IIF;				// LIMPIAR BANDERA DE INT.	
	MCF_I2C1_I2DR = 0x05; // PODER USAR GPO1 
	cpu_pause(20000);
	return;
}


/* FUNCIONES DE VISUALIZACION EN LCD 20x4*/
// ***********************************************
void VIS_INICIO()
{
	I2C_POSICION_CARACTER(6,1); // POSICION EN LCD
	I2C_ENVIA_DATO(0x43); // C
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x4E); // N
	I2C_ENVIA_DATO(0x54); // T
	I2C_ENVIA_DATO(0x52); // R
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x4C); // L
	I2C_ENVIA_DATO(0x20); //
	I2C_ENVIA_DATO(0x44); // D
	I2C_ENVIA_DATO(0x45); // E

	I2C_POSICION_CARACTER(2,2);
	I2C_ENVIA_DATO(0x41); // A
	I2C_ENVIA_DATO(0x43); // C
	I2C_ENVIA_DATO(0x43); // C
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x53); // S
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x20); // 
	I2C_ENVIA_DATO(0x53); // S
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x47); // G
	I2C_ENVIA_DATO(0x55); // U
	I2C_ENVIA_DATO(0x52); // R
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x20); // 
	I2C_ENVIA_DATO(0x50); // P
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x52); // R
	
	I2C_POSICION_CARACTER(3,3);
	I2C_ENVIA_DATO(0x48); // H
	I2C_ENVIA_DATO(0x55); // U
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x4C); // L
	I2C_ENVIA_DATO(0x4C); // L
	I2C_ENVIA_DATO(0x41); // A
	I2C_ENVIA_DATO(0x20); // 
	I2C_ENVIA_DATO(0x20); // 
	I2C_ENVIA_DATO(0x44); // D
	I2C_ENVIA_DATO(0x49); // I
	I2C_ENVIA_DATO(0x47); // G
	I2C_ENVIA_DATO(0x49); // I
	I2C_ENVIA_DATO(0x54); // T
	I2C_ENVIA_DATO(0x41); // A
	I2C_ENVIA_DATO(0x4C); // L

	return;
}

void VIS_MENU_INICIAL(void)
{
	I2C_POSICION_CARACTER(1,4);		// PARA POSICIONAR CARACTER
	I2C_ENVIA_DATO(0x53);		    // S
	I2C_ENVIA_DATO(0x57);		    // W
	I2C_ENVIA_DATO(0x32);		    // 2
	I2C_ENVIA_DATO(0x3A);		    // :
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x62);		    // b
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x69);		    // i
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x20);		    // espacio
	I2C_ENVIA_DATO(0x20);		    // espacio
	I2C_ENVIA_DATO(0x53);		    // S
	I2C_ENVIA_DATO(0x57);		    // W
	I2C_ENVIA_DATO(0x31);		    // 1
	I2C_ENVIA_DATO(0x3A);		    // :
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x67);		    // g
	I2C_ENVIA_DATO(0x72);		    // r

	return;	// REGRESAR.
}

void VIS_INICIO_AGREG_USU(void)
{
	I2C_FUCNIONES_LCD(0x58);	// LIMPIAR PANTALLA DE LCD.
	I2C_POSICION_CARACTER(2,1);		// PARA POSICIONAR CARACTER
	I2C_ENVIA_DATO(0x2A);		    // *
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x47);		    // G
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x45);		    // E
	I2C_ENVIA_DATO(0x47);		    // G
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x53);		    // S
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x41);		    // A	
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x2A);		    // *

	I2C_POSICION_CARACTER(1,2);		// PARA POSICIONAR CARACTER
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x43);		    // C
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x6C);		    // l
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x71);		    // q
	I2C_ENVIA_DATO(0x75);		    // u
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x73);		    // s
	I2C_ENVIA_DATO(0x75);		    // u
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x64);		    // d	
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	
	I2C_POSICION_CARACTER(2,3);		// PARA POSICIONAR CARACTER // 
	I2C_ENVIA_DATO(0x73);		    // s	
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x62);		    // b
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x6C);		    // l
	I2C_ENVIA_DATO(0x20);		    // 		
	I2C_ENVIA_DATO(0x6C);		    // l	
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x63);		    // c
	I2C_ENVIA_DATO(0x74);		    // t
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x72);		    // r

	I2C_POSICION_CARACTER(5,4);		// PARA POSICIONAR CARACTER // 
	I2C_ENVIA_DATO(0x45);		    // E	
	I2C_ENVIA_DATO(0x73);		    // s
	I2C_ENVIA_DATO(0x70);		    // p
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n	
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x3A);		    // :
	return;
}

void VIS_VERIFICAR(void)
{
I2C_FUCNIONES_LCD(0x58);	// LIMPIAR PANTALLA DE LCD.
	I2C_POSICION_CARACTER(2,1);		// PARA POSICIONAR CARACTER
	I2C_ENVIA_DATO(0x2A);		    // *
	I2C_ENVIA_DATO(0x56);		    // V
	I2C_ENVIA_DATO(0x45);		    // E
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x46);		    // F
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x43);		    // C
	I2C_ENVIA_DATO(0x41);		    // A	
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x53);		    // S
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x41);		    // A	
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x2A);		    // *
	I2C_POSICION_CARACTER(1,2);		// PARA POSICIONAR CARACTER
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x43);		    // C
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x6C);		    // l
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x71);		    // q
	I2C_ENVIA_DATO(0x75);		    // u
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x73);		    // s
	I2C_ENVIA_DATO(0x75);		    // u
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x64);		    // d	
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	
	I2C_POSICION_CARACTER(2,3);		// PARA POSICIONAR CARACTER // 
	I2C_ENVIA_DATO(0x73);		    // s	
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x62);		    // b
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x6C);		    // l
	I2C_ENVIA_DATO(0x20);		    // 		
	I2C_ENVIA_DATO(0x6C);		    // l	
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x63);		    // c
	I2C_ENVIA_DATO(0x74);		    // t
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x72);		    // r

	I2C_POSICION_CARACTER(5,4);		// PARA POSICIONAR CARACTER // 
	I2C_ENVIA_DATO(0x45);		    // E	
	I2C_ENVIA_DATO(0x73);		    // s
	I2C_ENVIA_DATO(0x70);		    // p
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n	
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x3A);		    // :
	return;
}

void VIS_ERROR()
{
	I2C_ENVIA_DATO(0x45);		    // E
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x21);		    // !
}

void VIS_LEYENDO()
{
	I2C_ENVIA_DATO(0x4C);		    // L
	I2C_ENVIA_DATO(0x65);		    // e	
	I2C_ENVIA_DATO(0x79);		    // y
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
}

void VIS_ENCRIPTANDO()
{
	I2C_ENVIA_DATO(0x45); 			// E
	I2C_ENVIA_DATO(0x6E); 			// n
	I2C_ENVIA_DATO(0x63);		    // c
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x69);		    // i
	I2C_ENVIA_DATO(0x70);		    // p
	I2C_ENVIA_DATO(0x74);		    // t
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x20); 			// 
	I2C_ENVIA_DATO(0x79); 			// y
	I2C_POSICION_CARACTER(3,3);
	I2C_ENVIA_DATO(0x67); 			// g
	I2C_ENVIA_DATO(0x75);		    // u
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
			
	return;
}
void VIS_DESENCRIPTANDO()
{
	I2C_ENVIA_DATO(0x44);		    // D
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x73);		    // s
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x6E); 			// n
	I2C_ENVIA_DATO(0x63);		    // c
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x69);		    // i
	I2C_ENVIA_DATO(0x70);		    // p
	I2C_ENVIA_DATO(0x74);		    // t
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x20); 			// 
	I2C_ENVIA_DATO(0x79); 			// y
	I2C_POSICION_CARACTER(3,3);
	I2C_ENVIA_DATO(0x76); 			// v
	I2C_ENVIA_DATO(0x65);		    // e
	I2C_ENVIA_DATO(0x72);		    // r
	I2C_ENVIA_DATO(0x69);		    // i
	I2C_ENVIA_DATO(0x66);		    // f
	I2C_ENVIA_DATO(0x69);		    // i
	I2C_ENVIA_DATO(0x63);		    // c
	I2C_ENVIA_DATO(0x61);		    // a
	I2C_ENVIA_DATO(0x6E);		    // n
	I2C_ENVIA_DATO(0x64);		    // d
	I2C_ENVIA_DATO(0x6F);		    // o
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
	I2C_ENVIA_DATO(0x2E);		    // .
			
	return;
}



void VIS_RECHAZADO()
{
	I2C_ENVIA_DATO(0x41); // A
	I2C_ENVIA_DATO(0x43); // C
	I2C_ENVIA_DATO(0x43); // C
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x53); // S
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x20); //
	I2C_ENVIA_DATO(0x44); // D
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x4E); // N
	I2C_ENVIA_DATO(0x45); // E
	I2C_ENVIA_DATO(0x47); // G
	I2C_ENVIA_DATO(0x41); // A
	I2C_ENVIA_DATO(0x44); // D
	I2C_ENVIA_DATO(0x4F); // O
	I2C_ENVIA_DATO(0x21); // !
			
	return;
}

void VIS_ACEPTADO()
{
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x43); 			// C
	I2C_ENVIA_DATO(0x45); 			// E
	I2C_ENVIA_DATO(0x50);		    // P
	I2C_ENVIA_DATO(0x54);		    // T
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x44);		    // D
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x21);		    // !
			
	return;
}

void VIS_REGISTRADO()
{
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x53); 			// S
	I2C_ENVIA_DATO(0x55);		    // U
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x20);		    // 	
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x45); 			// E
	I2C_ENVIA_DATO(0x47);		    // G
	I2C_ENVIA_DATO(0x49);		    // I
	I2C_ENVIA_DATO(0x53);		    // S
	I2C_ENVIA_DATO(0x54);		    // T
	I2C_ENVIA_DATO(0x52);		    // R
	I2C_ENVIA_DATO(0x41);		    // A
	I2C_ENVIA_DATO(0x44);		    // D
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x21);		    // !
			
	return;
}

/*
void VIS_OK()
{
	I2C_ENVIA_DATO(0x20);		    // 
	I2C_ENVIA_DATO(0x4F);		    // O
	I2C_ENVIA_DATO(0x4B);		    // K
			
	return;
}
*/

 void ENCRIPTAR()
 {
 	MCF_GPIO_PORTTC = 3; 
	// LEER LLAVE SECRETA
	LLAVE(); 

	// CONVERTIR LLAVE DE HEX A DEC Y DETERMINAR RANGOS 
	OP_CI = 'A'; // A
	DET_CI();
	VAL_A = CAL_CI;
	OP_CI = 'B'; // B
	DET_CI();
	VAL_B = CAL_CI;
	OP_CI = 'C'; // C
	DET_CI();
	VAL_C = CAL_CI;
	OP_CI = 'D'; // D
	DET_CI();
	VAL_D = CAL_CI;
	
		
	// ***************************************************************
	//       CALCULO DE MAPA LOGISTICO 2 PARA SUM_TXT
	// ***************************************************************

//	V_PA2 = fmod(VAL_A + VAL_B,1) * 0.001;
//	V_CI2 = fmod(VAL_C + VAL_D,1);
    V_SUM = VAL_A + VAL_B;
	V_PA2 = (VAL_A + VAL_B - V_SUM) * 0.1; // VAL_A + VAL_B - (1 * int(VAL_A + VAL_B/1)); por ser mod1 no divide ni multiplica *n
	V_SUM = VAL_C + VAL_D;
	V_CI2 = (VAL_C + VAL_D - V_SUM) * 0.1; // suma flotantes y resta parte entera quedando siempre valores menores a 1
	// para encriptar con mapa lotka-volterra se multiplican ambas variables por 0.1 para que las ecuaciones no se indeterminen**
	
	//a2_L1 = 3.999 + V_PA2; // logistico
	a2_L1 = 3.5;// lotka
	x2_L1 = V_CI2;// logistico
	
	MAP_X2 = x2_L1;
	//DAT_CAOTX2.CAR[0] = MAP_X2; // Condicion inicial logistico
	DAT_CAOTX2.CAR[0] = V_CI2;// lotka, condicion inicial, fragmento de llave
	ITE2 = LONG_MUES + 100; // Numero de Iteraciones
	d=3.9 + V_PA2; //lotka, condicion inicial, fragmento de llave
	Y2=0.2;//lotka // ** b es una constante arbitraria que escala el rango del mapa en Y2, escrito en la ecuacion
	for(i=1;i<ITE2;i++) // longitud de huella+100 cantidad de iteraciones en mapa caotico
	{
		ANT_X2 = MAP_X2;		
		//MAP_X2 = a2_L1 * ANT_X2 * (1 - ANT_X2);//logistico
		MAP_X2 = a2_L1 * ANT_X2 * (1 - ANT_X2)-3.2*ANT_X2*Y2;//lotka-voltera ec1
		Y2=d*ANT_X2*Y2;//ec 2, Y2 tiene el valor anterior
		
		// GRABAR DATO CAOTICO**
		//DAT_CAOTX2.CAR[i] = MAP_X2; logistico
		DAT_CAOTX2.CAR[i] = Y2; // lotka-voterra
	}
	
	SUM_TXT();		
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// ***************************************************************
	//       CALCULO DE MAPA LOGISTICO 1 PARA PERMUTACION Y DIFUSION
	// ***************************************************************
		
//	V_PA1 = fmod(VAL_A + VAL_B + TXT_SUM,1) * 0.001;
	V_SUM = VAL_A + VAL_B + TXT_SUM;
	V_PA1 = (VAL_A + VAL_B + TXT_SUM - V_SUM) * 0.1; // cambia multiplicacion de .001 a .1 para lotka-volterra 
	
//	V_CI1 = fmod(VAL_C + VAL_D + TXT_SUM,1);
	V_SUM = VAL_C + VAL_D + TXT_SUM; // toma parte entera
	V_CI1 = (VAL_C + VAL_D + TXT_SUM - V_SUM) * 0.1; // 
	
	//a1_L1 = 3.999 + V_PA1; // logistico
	a1_L1 = 3.5; // lotka-volterra
	//x1_L1 = V_CI1; // logistico
	x1_L1 = V_CI1; //lotka-volterra
		
	MAP_X1 = x1_L1;
	ITE1 = 2072; // Numero de Iteraciones - 1
	j=0;
	
	d=3.8 + V_PA1; // lotka-volterra cond inicial + fragmento de llave
	// b es una constante arbitraria que escala el rango del mapa en Y1
	Y1=0.1; // condicion inicial lotka-volterra
	for(i=0;i<ITE1;i++) // antes era i=1 Iteraciones para datos caoticos; 2072 ya no entra quedan 2071 desde 0 = 2072 diferentes
	{
		ANT_X1 = MAP_X1;		
		//MAP_X1 = a1_L1 * ANT_X1 * (1 - ANT_X1); // mapa logistico
		
		MAP_X1 = a1_L1 * ANT_X1 * (1 - ANT_X1)-3.2*ANT_X1*Y1;//lotka-voltera ec1
		Y1=d*ANT_X1*Y1;//ec 2, Y2 tiene el valor anterior
		// Grabar datos
//		if(i >= 2900) // Los valores corresponden a MatLab. 4300/muestra 2900/ Templete
//		{
			DAT_CAOTX1.CAR[i] = MAP_X1;
//			j++; // si no se usa mas en otra parte borrar la variable del encabezado*****
//		}
	}

				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);		
	// **********************************************
	//       DETERMINAR SECUENCIA DE PERMUTACION
	// **********************************************
	for(i=0;i<LONG_MUES;i++) // LONG_TXT, 2072; desde 0 a 2071 = 2072 diferentes
	{
		V_SUM = DAT_CAOTX1.CAR[i] * (LONG_MUES-1) + 0.5; // +1**; // 2071 *¿? el +1 causa error en variable BUS y PERM.CAR
	//	V_SUM = DAT_CAOTX1.CAR[2099+1-LONG_MUES+i] * (LONG_MUES-1) + 1 + 0.5; // El rango es para que. 699/muestra 2099/templete
   		// + 0.5 es para compensar el truncamiento
   		PERM.CAR[i] = V_SUM;
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// *******************************************************
	//       BUSCAR POCISIONES REPETIDAS DE PERM Y QUE NUMERO
	// *******************************************************
	
	k = 0;
	BUS = 0;
	NO_rep = 0;
	
	for(i=1;i<LONG_MUES;i++) // 1 a 2071  2071 en total, el primero no puede estar repetido*
	{
		EN = 0;
		BUS = PERM.CAR[i];
		for(j=0;j<i;j++)// menor que
		{
			if(BUS == PERM.CAR[j] && EN == 0)
			{
				//	POS_R.CAR[k]= i;				// Posiciones repetidas de PERM1
				POS_R = i; //+1;//
				//ESCRIBIR_DATO_FLASH(DIRE_REP, POS_R);// AGREGAR DIREC_MEMORIA, DATO.
				//DIRE_REP+=4;
				
				// guarda posicion del repetido**
				POSS.CAR[k]=POS_R;
				k++;
				//	VAL_R.CAR[k]= PERM.CAR[i];	    // Valores repetidos
				NO_rep++;	// Cuenta cuantos repetidos hay
				EN = 1;
			}
		}
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// BUSCAR VALORES QUE NO ESTEN EN PERM
	SALTAR = 0;
	k = 0;
	for(PREG_NUM=0 ; PREG_NUM<LONG_MUES ; PREG_NUM++) //** cambiado para comenzar en cero
	{
		// Pregunta si el numero esta en PERM, si esta no se usa.
		for(j=0;j<LONG_MUES;j++)
		{
			if(PREG_NUM == PERM.CAR[j]) 
			{
				SALTAR = 1;
				j=LONG_MUES; // Si encuentra el primero, se sale.
			}
		}
			
		if(SALTAR == 0)
		{
			NUM_NOP.CAR[k] = PREG_NUM;
			k++;
		}
		SALTAR = 0;
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// Actualizar PERM**
//	en_dos = round(NO_rep / 2);// redondeo con truncamiento implicito con variable int
	en_dos = (NO_rep/2)+0.5;
	cambio=1;
	S=0;
	//DIRE_REP = 0x00053000;// ya no se usa
	k=0;	
	for(i=0;i<NO_rep;i++)
	{
		if(cambio == 1)
		{
			// RECUPERAR_DATO_FLASH(DIRE_REP);
			// DIRE_REP+=4;
			POS = POSS.CAR[k];//-1;//** valor que no esta en repetidos
			k++;
			//POS = DATO_RECU;// - 1; // -1 ya que se cuenta desde posicion 0// no negativos
			
			//POS = POS_R.CAR[i];
			PERM.CAR[POS] = NUM_NOP.CAR[S];
			cambio = 0;
		}
		else
		{
			//RECUPERAR_DATO_FLASH(DIRE_REP);// AGREGAR DIREC_MEMORIA.
			//DIRE_REP+=4;
			
			// ya no se usa memoria flash
			POS = POSS.CAR[k]; //-1//*** lee valor que esta en los repetidos
			//POS = DATO_RECU - 1; // -1 ya que se cuenta desde posicion 0
			//POS = POS;// - 1;//**** ya comienza en cero			k++;
				
			//POS = POS_R.CAR[i];
			PERM.CAR[POS] = NUM_NOP.CAR[S + en_dos];
			S++;
			cambio = 1;
		}
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);	
	// *****************************************************
	//       SECUENCIAS Y PROCESO DE DIFUCION Y PERMUTACION; hace mod1 y round*255
	// *****************************************************
	
	// *****************************************************
	// *****************************************************

	for(i=0;i<LONG_MUES;i++) // cambia i=1 por i=0 para no saltarse el byte 0; cuando i=2072 ya no entra
	{
	//	X_ORIG = DAT_CAOTX1.CAR[2072-LONG_MUES+i]; // toma un dato del mapa(double)
    //  X_ALTE = X_ORIG * 100; // se elimino round**** es el mod(caos*100,255) de matlab
    
		X_ALTE = DAT_CAOTX1.CAR[i] * 100; // se elimino round 04/04/17 elimine la resta, daba 0 + i

		//X_DIFU = fmod(X_ALTE + TXT_SUM,1);
		V_SUM = X_ALTE + TXT_SUM;//mod1 toma parte entera
		X_DIFU = X_ALTE + TXT_SUM - V_SUM; // resta parte entera quedando decimales
		
		// X_REDO = round(X_DIFU * 255);
		X_REDO = (X_DIFU * 255) + 0.5; //+1**¿? // suma 0.5 para compensar truncamiento
			
		// Proceso de PERMUTACION Y DIFUSION
		
		//DIF_PERM_P.CAR[i-1] = fmod(HUELLA.CAR[PERM.CAR[i-1]-1] + X_REDO,255);
		//DIF_PERM_P = fmod(HUELLA.CAR[PERM.CAR[i-1]-1] + X_REDO,255);
		
		V_SUM=(DATOSX.DATO_XX[PERM.CAR[i]]+X_REDO)/255; // trunca division para la formula [ a - (n * int(a/n)) ] toma entero a/n sin redondear
	    // obsoleto //DIF_PERM_P= DATOSX.DATO_XX[PERM.CAR[i]]+X_REDO-(255 * V_SUM); // mod(a,n)=mod(perm,255) (2072 btes de criptograma)
	    
	    AUX_SAVE = DATOSX.DATO_XX[PERM.CAR[i]]+X_REDO-(255 * V_SUM); // mod(a,n)=mod(perm,255)  (2072 btes de criptograma)
	    //DATO_t.DATOS_crip[i]= AUX_SAVE; // hace un duplicado de los datos en RAM
	    //DATOSX.DATO_XX[PERM.CAR[i]]=0; // deespues de usar un byte de la huella se borra***
	    
	    // 04/04/17 le quite el -1 al subindice de los vectores de PERM.CAR, se hace negativo y da error al leer

		// GRABAR EN MEMORIA FLASH
		// ALMACENA ENTRE LAS DIRECCIONES:
		// 0x00050000 Y 0x00060000. PUEDE ALMACENAR 14000 DATOS

		ESCRIBIR_DATO_FLASH(DIRE_ENC, AUX_SAVE); //** guarda variable de 2 Bytes revisar si ahora puedo saltar 2 en vez de 4bytes
		//ESCRIBIR_DATO_FLASH(DIRE_ENC, DIF_PERM_P);// AGREGAR DIREC_MEMORIA, DATO.
		DIRE_ENC+=4;

	}	

	// *****************************************************
	//       GUARDAR DATO DE TXT_SUM
	// *****************************************************
	
	DIF_PERM_P = MOD_SUM2; // Se guarda al final del vector ( valor Z )

	ESCRIBIR_DATO_FLASH(DIRE_ENC, DIF_PERM_P);// AGREGAR DIREC_MEMORIA, DATO.
	DIRE_ENC+=4; // para cuando se quiera guardar la siguiente huella
	
//	USB_PLAIN = 1; // Para guardat cipher text en USB (decimal)
	// AGREGAR GUARDAR EN USB
	//borrada una funcion para guardar en usb


   //for (i=0;i<LONG_MUES;i++)
   //{
   // 	DATOSX.DATO_XX[i] = 0; // borra el templete por seguridad
   //}

 }// fin del encriptado
 
 void LLAVE()
{
	// INTRODUCIR LLAVE PC A (8 DIG HEX) 0 - 7
	SEC_A.CAR[0] = KEY[7]; // Menos significativo
	SEC_A.CAR[1] = KEY[6];
	SEC_A.CAR[2] = KEY[5];
	SEC_A.CAR[3] = KEY[4];
	SEC_A.CAR[4] = KEY[3];
	SEC_A.CAR[5] = KEY[2];
	SEC_A.CAR[6] = KEY[1];
	SEC_A.CAR[7] = KEY[0]; // Mas significativo

	// INTRODUCIR LLAVE PC B (10 DIG HEX) 35 - 44
	SEC_B.CAR[0] = KEY[15]; // Menos significativo
	SEC_B.CAR[1] = KEY[14];
	SEC_B.CAR[2] = KEY[13];
	SEC_B.CAR[3] = KEY[12];
	SEC_B.CAR[4] = KEY[11];
	SEC_B.CAR[5] = KEY[10];
	SEC_B.CAR[6] = KEY[9];
	SEC_B.CAR[7] = KEY[8];

	// INTRODUCIR LLAVE PC C (10 DIG HEX) 45 - 54
	SEC_C.CAR[0] = KEY[23]; // Menos significativo
	SEC_C.CAR[1] = KEY[22];
	SEC_C.CAR[2] = KEY[21];
	SEC_C.CAR[3] = KEY[20];
	SEC_C.CAR[4] = KEY[19];
	SEC_C.CAR[5] = KEY[18];
	SEC_C.CAR[6] = KEY[17];
	SEC_C.CAR[7] = KEY[16];

	// INTRODUCIR LLAVE PC D (10 DIG HEX) 55 - 64
	SEC_D.CAR[0] = KEY[31]; // Menos significativo
	SEC_D.CAR[1] = KEY[30];
	SEC_D.CAR[2] = KEY[29];
	SEC_D.CAR[3] = KEY[28];
	SEC_D.CAR[4] = KEY[27];
	SEC_D.CAR[5] = KEY[26];
	SEC_D.CAR[6] = KEY[25];
	SEC_D.CAR[7] = KEY[24];

	return;
}


void DET_CI()
{
	// Limpiar variables
	SUM_KEY = 0;
	VAL_16 = 0;
	VAL_KEY = 0;
	LONG_KEY = 8;
	CAL_CI = 0;
	
	// Convertir a Hex
	for(i=0;i<LONG_KEY;i++) // valido para 8 digitos hexadecimal
		{
				
			switch(OP_CI)	// Determinar valor en tipo Long de valor hexadecimal
			  {	
				case 'A':CAR_CI = SEC_A.CAR[i];break;			
				case 'B':CAR_CI = SEC_B.CAR[i];break;
				case 'C':CAR_CI = SEC_C.CAR[i];break;
				case 'D':CAR_CI = SEC_D.CAR[i];break;			

				default:break;}
							
			switch(CAR_CI)	// Determinar valor en tipo Long de valor hexadecimal
			  {	case '0':VAL_KEY = 0;break;
				case '1':VAL_KEY = 1;break;
				case '2':VAL_KEY = 2;break;			
				case '3':VAL_KEY = 3;break;
				case '4':VAL_KEY = 4;break;
				case '5':VAL_KEY = 5;break;			
				case '6':VAL_KEY = 6;break;			
				case '7':VAL_KEY = 7;break;
				case '8':VAL_KEY = 8;break;			
				case '9':VAL_KEY = 9;break;			
				case 'A':VAL_KEY = 10;break;
				case 'B':VAL_KEY = 11;break;
				case 'C':VAL_KEY = 12;break;			
				case 'D':VAL_KEY = 13;break;			
				case 'E':VAL_KEY = 14;break;			
				case 'F':VAL_KEY = 15;break;
				default:break;}
				
			switch(i)	// Determinar valor en tipo Long de mumtiplicador
			  {	case 0:VAL_16 = 1;break;
				case 1:VAL_16 = 16;break;
				case 2:VAL_16 = 256;break;			
				case 3:VAL_16 = 4096;break;
				case 4:VAL_16 = 65536;break;
				case 5:VAL_16 = 1048576;break;			
				case 6:VAL_16 = 16777216;break;
				case 7:VAL_16 = 268435456;break;
				default:break;}
			  
			  SUM_KEY = SUM_KEY + (VAL_KEY * VAL_16);
		}

	// CALCULO DE DECIMAL DE A,B,C,D (/4294967296+1)
		CAL_CI = (SUM_KEY / (4294967296 + 1)); // VALORES: 0 < A,B,C,D < 1	

	return;
}


void SUM_TXT()
{
	j=1;
	SUM_MEN = 0;
	
	MOD_SUM1 = 0;
	MOD_SUM2 = 0;	
	TXT_SUM = 0;
	
	
	for(i=0;i<LONG_MUES;i++) // se sustituye long_txt por long_mues
	{	
		//SUM_MEN = SUM_MEN + (HUELLA.CAR[i] * DAT_CAOTX2.CAR[ITE2-1-i]) + DAT_CAOTX2.CAR[ITE2-1-i];
		SUM_MEN = SUM_MEN + (DATOSX.DATO_XX[i] * DAT_CAOTX2.CAR[ITE2-1-i]) + DAT_CAOTX2.CAR[ITE2-1-i];
	}	

//	MOD_SUM1 = fmod(SUM_MEN,1);
	V_SUM = SUM_MEN; // copia parte entera, se trunca lo que no es entero
	MOD_SUM1 = SUM_MEN - V_SUM; // resta variable entera a la variable flotante quedando solo la parte decimal
//	MOD_SUM2 = round(MOD_SUM1*255);	
// en lugar de usar round voy a determinar el valor yo mismo
    V_SUM = MOD_SUM1*255+0.5; // sumo al valor 0.5 y guardo valor entero truncado, es decir redondea hacia abajo
    MOD_SUM2 = V_SUM;// esta variable solo sirve para conservar el valor para despues
	TXT_SUM = MOD_SUM2 / 255; // Valor que se suma en etapa difusion

	return;
}


// ************ Funciones para memoria flash *************

void INIC_MEMORIA_FLASH(void)
{
	// SE DIVIDE FRECUENCIA DE 80 MHZ ENTRE 8 = 10 MHZ.
	// SE DIVIDE 10MHZ ENTRE 54 ((0x35)+1) = 185.18 KHz.
	// LA FRECUANCIA DEBE ESTAR ENTRE 150 Y 200 KHz
	// PARA EL BUEN FUNCIONAMIENTO DEL MODULO.
	MCF_CFM_CFMCLKD |=
	MCF_CFM_CFMCLKD_DIV(0x35) | MCF_CFM_CFMCLKD_PRDIV8; 

	MCF_CFM_CFMPROT = 0x00; 	// DESACTIVA PROTECCIÓN DE FLASH.
	MCF_CFM_CFMUSTAT |= (MCF_CFM_CFMUSTAT_PVIOL|MCF_CFM_CFMUSTAT_ACCERR);//  LIMPIA ERRORES.
	return;	// REGRESAR.
}

void ESCRIBIR_DATO_FLASH(unsigned int DIREC_MEMORIA, unsigned int DATO_F)
{
	MCF_CFM_CFMUSTAT = (MCF_CFM_CFMUSTAT_PVIOL | MCF_CFM_CFMUSTAT_ACCERR); // LIMPIAR ERRORES.
	while (!(MCF_CFM_CFMUSTAT & MCF_CFM_CFMUSTAT_CBEIF)){};	// ESPERAR HASTA QUE ESTE DISPONOBLE EL BUFER.
		
	(*(volatile vuint32 *)(FLASH_START_ADDRESS + DIREC_MEMORIA)) =  DATO_F;// APUNTAR A UNA DIRECCION DE
																	 	 // MEMORIA Y GUARDAR EL DATO.	
	MCF_CFM_CFMCMD = MCF_CFM_CFMCMD_WORD_PROGRAM;			// GUARDA EL DATO
	MCF_CFM_CFMUSTAT |= MCF_CFM_CFMUSTAT_CBEIF;				// LIMPIA BANDERA DE FIN DE PROCESO.
	return;	// REGRESAR.
}

void RECUPERAR_DATO_FLASH(unsigned int DIREC_MEMORIA)
{
	DATO_RECU = (*(volatile vuint32 *)(FLASH_START_ADDRESS+DIREC_MEMORIA));// LEE EL DATO EN LA DIRECCIÓN.
	return;	// REGRESAR.
}

void BORRAR_DATO_FLASH(unsigned int DIREC_MEMORIA)
{
	MCF_CFM_CFMUSTAT = (MCF_CFM_CFMUSTAT_PVIOL | MCF_CFM_CFMUSTAT_ACCERR); // LIMPIAR ERRORES.
	while (!(MCF_CFM_CFMUSTAT & MCF_CFM_CFMUSTAT_CBEIF)){};// ESPERAR HASTA QUE ESTE DISPONOBLE EL BUFER.

	(*(volatile vuint32 *)(CFM_IPS_FLASH_ADDR+DIREC_MEMORIA)) =  -1;// APUNTA A UNA DIRECCIÓN DE MEMORIA.
    MCF_CFM_CFMCMD = MCF_CFM_CFMCMD_PAGE_ERASE;						// ENVÍA COMANDO PARA BORRAR MEMORIA.

	MCF_CFM_CFMUSTAT |= MCF_CFM_CFMUSTAT_CBEIF;						// LIMPIA BANDERA DE PROCESO TERMINADO.
	return;	// REGRESAR.
}

void DESENCRIPTAR()
{
	// inicia funcion de desencriptado, se reutilizan todos los vectores para ahorrar memoria
 	MCF_GPIO_PORTTC = 3; 
 	
 	// recuperar Z, lee direccion 0x52060 ¿?
 	RECUPERAR_DATO_FLASH(DIRE_ENC-4); // lee Z, el ULTIMO byte del criptograma 2072 iniciando desde 0 = 2073 Bytes
 	MOD_SUM2 = DATO_RECU; // guarda el valor para el resto del programa
 	TXT_SUM = MOD_SUM2 / 255; // valor a usar en el mapa caotico de 0 a 1
	// LEER LLAVE SECRETA
	LLAVE(); 

	// CONVERTIR LLAVE DE HEX A DEC Y DETERMINAR RANGOS 
	OP_CI = 'A'; // A
	DET_CI();
	VAL_A = CAL_CI;
	OP_CI = 'B'; // B
	DET_CI();
	VAL_B = CAL_CI;
	OP_CI = 'C'; // C
	DET_CI();
	VAL_C = CAL_CI;
	OP_CI = 'D'; // D
	DET_CI();
	VAL_D = CAL_CI;
	
	// ***************************************************************
	//   CALCULO DE MAPA LOGISTICO 2 para desencriptar no se calcula, se usa el valor Z
	// *************************************************************** 
	
	// ***************************************************************
	//       CALCULO DE MAPA LOGISTICO 1 PARA PERMUTACION Y DIFUSION
	// ***************************************************************
		
//	V_PA1 = fmod(VAL_A + VAL_B + TXT_SUM,1) * 0.001;
	V_SUM = VAL_A + VAL_B + TXT_SUM;
	V_PA1 = (VAL_A + VAL_B + TXT_SUM - V_SUM) * 0.1; // cambia multiplicacion para lotka-volterra
	
//	V_CI1 = fmod(VAL_C + VAL_D + TXT_SUM,1);
	V_SUM = VAL_C + VAL_D + TXT_SUM; // toma parte entera
	V_CI1 = (VAL_C + VAL_D + TXT_SUM - V_SUM) * 0.1; 
	
//	a1_L1 = 3.999 + V_PA1; //logistico
    a1_L1 = 3.5; // lotka-volterra
	x1_L1 = V_CI1;
	
	MAP_X1 = x1_L1;
	ITE1 = 2072; // Numero de Iteraciones, 3000 en matlab
	j=0;
	Y1=0.1; // cond inicial
	d=3.8+V_PA1; // cond inicial, fragmento de llave
	for(i=0;i<ITE1;i++) // antes era i=1 Iteraciones para datos caoticos; 2072 ya no entra quedan 2071 desde 0 = 2072 diferentes
	{
		ANT_X1 = MAP_X1;		
		//MAP_X1 = a1_L1 * ANT_X1 * (1 - ANT_X1); // mapa logistico
		
		MAP_X1 = a2_L1 * ANT_X1 * (1 - ANT_X1)-3.2*ANT_X1*Y1;//lotka-voltera, Ec 1
		Y1=d*ANT_X1*Y1;// Ec 2
		// Grabar datos
//		if(i >= 2900) // Los valores corresponden a MatLab. 4300/muestra 2900/ Templete
//		{
			DAT_CAOTX1.CAR[i] = MAP_X1;
//			j++;
//		}
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);


	// **********************************************
	//       DETERMINAR SECUENCIA DE PERMUTACION
	// **********************************************
	for(i=0;i<LONG_MUES;i++) // LONG_TXT, 2072; desde 0 a 2071 = 2072 diferentes
	{
		V_SUM = DAT_CAOTX1.CAR[i] * (LONG_MUES-1) + 0.5; // +1**; // 2071 *¿? el +1 causa error en variable BUS y PERM.CAR
	//	V_SUM = DAT_CAOTX1.CAR[2099+1-LONG_MUES+i] * (LONG_MUES-1) + 1 + 0.5; // El rango es para que. 699/muestra 2099/templete
   		// + 0.5 es para compensar el truncamiento
   		PERM.CAR[i] = V_SUM;
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// *******************************************************
	//       BUSCAR POCISIONES REPETIDAS DE PERM Y QUE NUMERO
	// *******************************************************
	
	k = 0;
	BUS = 0;
	NO_rep = 0;
	
	for(i=1;i<LONG_MUES;i++) // 1 a 2071 *** 2071 en total, el primero no puede estar repetido*
	{
		EN = 0;
		BUS = PERM.CAR[i];
		for(j=0;j<i;j++)
		{
			if(BUS == PERM.CAR[j] && EN == 0)
			{
				//POS_R = i;
				POSS.CAR[k]=i; //guarda posicion del repetido
				k++;
				NO_rep++;
				EN = 1;
			}
		}
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// BUSCAR VALORES QUE NO ESTEN EN PERM
	SALTAR = 0;
	k = 0;
	for(PREG_NUM=0 ; PREG_NUM<LONG_MUES ; PREG_NUM++)
	{
		// Pregunta si el numero esta en PERM, si esta no se usa.
		for(j=0;j<LONG_MUES;j++)
		{
			if(PREG_NUM == PERM.CAR[j]) 
			{
				SALTAR = 1;
				j=LONG_MUES;
			}
		}
			
		if(SALTAR == 0)
		{
			NUM_NOP.CAR[k] = PREG_NUM;
			k++;
		}
		SALTAR = 0;
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// Actualizar PERM
	en_dos = (NO_rep/2)+0.5;// redondea
	cambio=1;
	S=0;
	k=0;	
	for(i=0;i<NO_rep;i++)
	{
		if(cambio == 1)
		{
			POS = POSS.CAR[k];//-1;//** guarda valor que no esta en repetidos
			k++;
			PERM.CAR[POS] = NUM_NOP.CAR[S];
			cambio = 0;
		}
		else
		{
			POS = POSS.CAR[k];
			PERM.CAR[POS] = NUM_NOP.CAR[S + en_dos];
			S++;
			cambio = 1;
		}
	}
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
	// *****************************************************
	//     REVIERTE LA DIFUCION Y PERMUTACION; hace mod1 y round*255
	// *****************************************************
	
	// Se inicializa la direccion donde se encuentra el criptograma en la memoria flash
	//DIRE_ENC=0x00050000; // primer byte el criptograma, para mas de un usuario sumar +2073*numero de usuario, iniciando en 0 por Z
	//DIRE_ENC=DIRE_ENC+(2073*4*(usuarios_guardados-1));
	DIRE_ENC=DIRE_ENC-0x2064;// regresa direccion al inicio del templete actual**

	for(i=0;i<LONG_MUES;i++)// se debe obtener exactamente los mismos valores del vector que en el encriptado para PERM.CAR y X_REDO
	{
		// Difusion
		X_ALTE = DAT_CAOTX1.CAR[i] * 100; // se elimino round; 04/04/17 borrado 2072-LONG_MUES por que siempre es 0
		V_SUM = X_ALTE + TXT_SUM;//mod1 toma parte entera
		X_DIFU = X_ALTE + TXT_SUM - V_SUM; // resta parte entera quedando decimales
		X_REDO = X_DIFU * 255 + 0.5 ; //+1**¿? // suma 0.5 para compensar truncamiento, escala de 0 a 255
		
		//**************************************************************************************
		// Las modificaciones para DESENCRIPTAR son cambiar la suma del mapa caotico por una resta,
		// el vector de permutacion PERM.CAR[i] sigue siendo la posicion en el vector de la huella, ahora recupera los datos.
		//**************************************************************************************
	    
	    // V_SUM = (DATOSX.DATO_XX[PERM.CAR[i]-1]+X_REDO)/255; // trunca division para la formula [ a - (n * int(a/n)) ] toma entero a/n sin redondear
	    // DIF_PERM_P = DATOSX.DATO_XX[PERM.CAR[i]-1]+X_REDO-(255 * V_SUM);// mod(a,n)=mod(perm,255) *** (2072 btes de criptograma)
	    
	    // El vector que recupera la huella desencriptada es DATOSX.DATO_XX, la direccion donde se coloca es PERM.CAR[i]
	    RECUPERAR_DATO_FLASH(DIRE_ENC); // lee un byte de criptograma de la flash
	    //04/07/17
	    //Cambiado para que lea RAM en lugar de memoria flash, toma bye de criptograma y le resta caos y lo coloca en la posision del templete original
	    
	    //08/04/17 En caso de que la resta sea negativa se suma 255 antes de guardar para evitar datos corruptos
	    //AUX_MOD = DATO_t.DATOS_crip[i] - X_REDO; // Variable tipo int con signo que se usa para ver si se hace negativa la resta
	    AUX_MOD = DATO_RECU - X_REDO; // Variable tipo int con signo que se usa para ver si se hace negativa la resta
	    if (AUX_MOD<0) // int con signo para ver si se hace negativa la resta
	    {
	    	DATOSX.DATO_XX[PERM.CAR[i]] = 255 + AUX_MOD;// sumando 255 da resultado positivo menor a 255
	    }
	    else
	    {
	    	DATOSX.DATO_XX[PERM.CAR[i]] = AUX_MOD; // si no es negativo, es menor a 255 y lo guarda
	    }
	    // DATOSX.DATO_XX[PERM.CAR[i]] = DATO_t.DATOS_crip[i] - X_REDO;
	    
	    
	    // DATOSX.DATO_XX[PERM.CAR[i]] = DATO_RECU - X_REDO; //desencripta un byte por iteracion y revierte la permutacion-difusion
	    // 04/04/17 borrados los dos -1 en la direccion por que en algun momento se hace negativo
	    
	    DIRE_ENC+=4; // suma 4 para ir al siguiente bloque de memoria flash y termina en la direccion final del Templete + Z ***

	}	
				LED_AMARILLO_ON_OFF(0x57);
				LED_AMARILLO_ON_OFF(0x56);
DIRE_ENC+=4;// para poder leer el valor Z sin volver a encriptar
// FIN DE DESENCRIPTADO
// EL TEMPLETE ESTA EN LA RAM LISTA PARA SER ENVIADA A COMPARAR CON LA MUESTRA ACTUAL CON LA FUNCION "verificar()".
}

void BORRAR_DATOS_FLASH(void)
{

	DIREC_INICIAL = 0x00050000;
	DIREC_FINAL =   0x00056000;


	// BORRAR MEMORIA UTILIZADA, POR PAGINAD DE 0x1000 EN 0x1000.
	for(DIREC_INICIAL; DIREC_INICIAL<=DIREC_FINAL ; DIREC_INICIAL+=0x1000)
	  {
		MCF_CFM_CFMUSTAT = (MCF_CFM_CFMUSTAT_PVIOL | MCF_CFM_CFMUSTAT_ACCERR); // LIMPIAR ERRORES.
		while (!(MCF_CFM_CFMUSTAT & MCF_CFM_CFMUSTAT_CBEIF)){};// ESPERAR HASTA QUE ESTE DISPONOBLE EL BUFER.

		(*(volatile vuint32 *)(CFM_IPS_FLASH_ADDR+DIREC_INICIAL)) =  -1;// APUNTA A UNA DIRECCIÓN DE MEMORIA.
    	MCF_CFM_CFMCMD = MCF_CFM_CFMCMD_PAGE_ERASE;					// ENVÍA COMANDO PARA BORRAR MEMORIA.
    	MCF_CFM_CFMUSTAT |= MCF_CFM_CFMUSTAT_CBEIF;					// LIMPIA BANDERA DE PROCESO TERMINADO. 

	  }

	return;	// REGRESAR.
}
