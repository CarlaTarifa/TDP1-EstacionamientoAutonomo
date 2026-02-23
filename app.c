#include "sapi.h"
#include <stdio.h> 

#define UART_BT       UART_232 
#define BAUD_RATE_BT  9600     

// --- CENTRADO FINAL ---
#define TOL_CENTRADO_CM      2
#define PULSO_CENTRADO_MS    120
#define PAUSA_CENTRADO_MS    180
#define MAX_PASOS_CENTRADO   35

// --- SENSORES ---
#define TRIG_IZQ     GPIO0  
#define ECHO_IZQ     GPIO2

#define TRIG_DER     T_COL1   
#define ECHO_DER     GPIO1

#define TRIG_FRENTE  GPIO4   
#define ECHO_FRENTE  LCD1   

#define TRIG_ATRAS   LCDRS   
#define ECHO_ATRAS   LCD4   

// --- MOTORES ---
#define M1_PWM       PWM2     //izquierdo
#define M1_IN1       GPIO3
#define M1_IN2       GPIO5

#define M2_PWM       PWM0   //derecho  
#define M2_IN1       T_FIL3
#define M2_IN2       T_FIL0

#define VEL_AVANCE    120      
#define VEL_REVERSA   120      
#define VEL_MANIOBRA  130      

//calibracion
const uint32_t DIST_MINIMA       = 15;
const uint32_t DIST_MINIMA_ATRAS = 19;   
const uint32_t DIST_HUECO        = 25;      
const int      TICKS_HUECO_MIN   = 3;

//tiempos de maniobra
const int      T_ALINEAR         = 400;   
const int T_MAX_REVERSA = 60; 

typedef enum { LADO_IZQUIERDO, LADO_DERECHO } LadoEstacionamiento;
typedef enum {
    MODO_ESPERA = 0,
    MODO_BUSCANDO,      
    MODO_ALINEAR,       
    MODO_RETORNO_HUECO,  
    MODO_GIRO_IN,           
    MODO_ATRAS,           
    MODO_ACOMODO_ADELANTE, 
    MODO_GIRO_OUT,        
    MODO_ESTACIONADO,   
    MODO_OBSTACULO,
    MODO_CENTRANDO
} EstadoAuto;

EstadoAuto estado = MODO_ESPERA;
LadoEstacionamiento ladoElegido = LADO_IZQUIERDO;

uint32_t d_Izq = 0, d_Frente = 0, d_Der = 0, d_Atras = 0;
int contador_espacio_izq = 0;
int contador_espacio_der = 0;
int timer_seguridad = 0; 

int ticks_hueco_medido = 0;
float timer_maniobra = 0; 
int ticks_giro_realizado = 0;
int distancia_lateral_inicial = 0; 
int ticks_adelante_realizado = 0;  
int pasos_centrado = 0;
char bufferMsg[100];

// --- FUNCION LEER  ---
static uint32_t leer_cm(gpioMap_t trig, gpioMap_t echo) {
    gpioWrite(trig, OFF); delayInaccurateUs(2);
    gpioWrite(trig, ON);  delayInaccurateUs(10);
    gpioWrite(trig, OFF);
    uint32_t to = 40000; 
    while(gpioRead(echo) == OFF) { if(to-- == 0) return 0; }
    uint32_t start = cyclesCounterRead();
    while(gpioRead(echo) == ON) {
        if((cyclesCounterRead() - start) > (204000000 / 33)) return 0;
    }
    return (uint32_t)((float)(cyclesCounterRead() - start) / 204.0f / 58.0f);
}

// --- FUNCIONES DE MOVIMIENTO ---
void frenar(void) {
    pwmWrite(M1_PWM, 0); pwmWrite(M2_PWM, 0);
    gpioWrite(M1_IN1,0); gpioWrite(M1_IN2,0); gpioWrite(M2_IN1,0); gpioWrite(M2_IN2,0);
}

void avanzar(void) {
    gpioWrite(M1_IN1,1); gpioWrite(M1_IN2,0);
    gpioWrite(M2_IN1,1); gpioWrite(M2_IN2,0);
    pwmWrite(M1_PWM, VEL_AVANCE); pwmWrite(M2_PWM, VEL_AVANCE);
}

void retroceder(void) {
    gpioWrite(M1_IN1,0); gpioWrite(M1_IN2,1);
    gpioWrite(M2_IN1,0); gpioWrite(M2_IN2,1);
    pwmWrite(M1_PWM, VEL_REVERSA); pwmWrite(M2_PWM, VEL_REVERSA);
}

void meter_cola_logic(void) {
    if (ladoElegido == LADO_IZQUIERDO) {
       //giro horario
        gpioWrite(M1_IN1,1); gpioWrite(M1_IN2,0); 
        gpioWrite(M2_IN1,0); gpioWrite(M2_IN2,1); 
    } else {
        gpioWrite(M1_IN1,0); gpioWrite(M1_IN2,1); 
        gpioWrite(M2_IN1,1); gpioWrite(M2_IN2,0); 
    }
    pwmWrite(M1_PWM, VEL_MANIOBRA); pwmWrite(M2_PWM, VEL_MANIOBRA);
}

void enderezar_trompa_logic(void) {
    if (ladoElegido == LADO_IZQUIERDO) {
       //giro antihoorario
        gpioWrite(M1_IN1,0); gpioWrite(M1_IN2,1);
        gpioWrite(M2_IN1,1); gpioWrite(M2_IN2,0); 
    } else {
        gpioWrite(M1_IN1,1); gpioWrite(M1_IN2,0); 
        gpioWrite(M2_IN1,0); gpioWrite(M2_IN2,1); 
    }
    pwmWrite(M1_PWM, VEL_MANIOBRA); pwmWrite(M2_PWM, VEL_MANIOBRA);
}

void meter_trompa_adelante_logic(void) {
    if (ladoElegido == LADO_IZQUIERDO) {
        gpioWrite(M1_IN1,0); gpioWrite(M1_IN2,0); //rueda izquierda quieta
        gpioWrite(M2_IN1,1); gpioWrite(M2_IN2,0); //rueda derecha avanza
        pwmWrite(M1_PWM, 0); pwmWrite(M2_PWM, VEL_MANIOBRA);
    } else {
        gpioWrite(M1_IN1,1); gpioWrite(M1_IN2,0); 
        gpioWrite(M2_IN1,0); gpioWrite(M2_IN2,0); 
        pwmWrite(M1_PWM, VEL_MANIOBRA); pwmWrite(M2_PWM, 0);
    }
}

int main(void) {
    boardConfig();
    cyclesCounterConfig(204000000);
    uartConfig(UART_BT, BAUD_RATE_BT); 

    pwmConfig(0, PWM_ENABLE);
    pwmConfig(M1_PWM, PWM_ENABLE_OUTPUT); pwmConfig(M2_PWM, PWM_ENABLE_OUTPUT);
    gpioConfig(M1_IN1, GPIO_OUTPUT); gpioConfig(M1_IN2, GPIO_OUTPUT);
    gpioConfig(M2_IN1, GPIO_OUTPUT); gpioConfig(M2_IN2, GPIO_OUTPUT);

    gpioConfig(TRIG_IZQ, GPIO_OUTPUT); gpioConfig(ECHO_IZQ, GPIO_INPUT);
    gpioConfig(TRIG_DER, GPIO_OUTPUT); gpioConfig(ECHO_DER, GPIO_INPUT);
    gpioConfig(TRIG_FRENTE, GPIO_OUTPUT); gpioConfig(ECHO_FRENTE, GPIO_INPUT);
    gpioConfig(TRIG_ATRAS, GPIO_OUTPUT); gpioConfig(ECHO_ATRAS, GPIO_INPUT); 

    gpioConfig(TEC1, GPIO_INPUT); gpioConfig(TEC2, GPIO_INPUT);
    gpioConfig(LEDR, GPIO_OUTPUT); gpioConfig(LEDG, GPIO_OUTPUT); gpioConfig(LEDB, GPIO_OUTPUT);

    frenar();
    for(int i=0; i<6; i++) { gpioToggle(LEDB); delay(500); }
    gpioWrite(LEDB, OFF);
    uartWriteString(UART_BT, "LISTO. PULSE TEC1\r\n");

    while(1) {
        if (!gpioRead(TEC2)) { 
            frenar(); 
            uartWriteString(UART_BT, "RESET\r\n");
            estado = MODO_ESPERA; 
            contador_espacio_izq = 0; contador_espacio_der = 0;
            timer_seguridad = 0;
            delay(500); 
        }

        // Lecturas secuenciales para no interferir
        d_Izq    = leer_cm(TRIG_IZQ, ECHO_IZQ);delay(15);
        d_Der    = leer_cm(TRIG_DER, ECHO_DER);delay(15);
        d_Frente = leer_cm(TRIG_FRENTE, ECHO_FRENTE);delay(15);
        d_Atras  = leer_cm(TRIG_ATRAS, ECHO_ATRAS);delay(15);
 
       sprintf(bufferMsg, "I:%lu | D:%lu | F:%lu | A:%lu | Estado:%d\r\n", d_Izq,d_Der, d_Frente, d_Atras, estado);
        uartWriteString(UART_BT, bufferMsg);

        switch(estado) {
            case MODO_ESPERA:
                frenar();
                 gpioWrite(LEDB, ON); gpioWrite(LEDG, OFF); gpioWrite(LEDR, OFF); 
                if (!gpioRead(TEC1)) { 
                    gpioWrite(LEDB, ON); delay(500); gpioWrite(LEDB,OFF); delay(500);
                    contador_espacio_izq = 0; contador_espacio_der = 0;
                    estado = MODO_BUSCANDO; 
                }
                break;

            case MODO_BUSCANDO:
                avanzar();
                gpioWrite(LEDB, OFF); gpioWrite(LEDG, ON); 

                 if (d_Frente > 0 && d_Frente < DIST_MINIMA) {
                    estado = MODO_OBSTACULO; 
                }
            
                if (d_Izq > DIST_HUECO || d_Izq == 0) {
                    contador_espacio_izq++;
                    gpioToggle(LEDG); 
                } else {
                    if (contador_espacio_izq > TICKS_HUECO_MIN) {
                        ladoElegido = LADO_IZQUIERDO;
                        frenar(); 
                        ticks_hueco_medido = contador_espacio_izq;
                        distancia_lateral_inicial = d_Izq;
                       
                        sprintf(bufferMsg, "HUECO: %d ticks | LAT: %d cm\r\n", ticks_hueco_medido, distancia_lateral_inicial);
                        uartWriteString(UART_BT, bufferMsg);

                        delay(1000);
                        timer_maniobra = 0; 
                        timer_seguridad = 0;
                        estado = MODO_RETORNO_HUECO;
                        break; 
                    } else contador_espacio_izq = 0;
                }

                // LADO DERECHO
                if (d_Der > DIST_HUECO || d_Der == 0) {
                    contador_espacio_der++;
                    gpioToggle(LEDG);
                } else {
                    if (contador_espacio_der > TICKS_HUECO_MIN) {
                        ladoElegido = LADO_DERECHO;
                        frenar();
                       ticks_hueco_medido = contador_espacio_der;
                        distancia_lateral_inicial = d_Der;

                       sprintf(bufferMsg, "HUECO: %d ticks | LAT: %d cm\r\n", ticks_hueco_medido, distancia_lateral_inicial);
                        uartWriteString(UART_BT, bufferMsg);
 
                       delay(1000);
                        timer_maniobra = 0; 
                        timer_seguridad = 0;
                        estado = MODO_RETORNO_HUECO;
                        break;
                    } else contador_espacio_der = 0;
                }
                break;

            case MODO_RETORNO_HUECO:
                retroceder();
                timer_maniobra++; 
                if (timer_maniobra >= 4) { 
                    frenar();
                    delay(800);
                    timer_maniobra = 0; 
                    timer_seguridad = 0; 
                    estado = MODO_GIRO_IN;
                }
                break;

            case MODO_GIRO_IN:
                meter_cola_logic();
                timer_maniobra++; 
                 float ticks_giro_in = ticks_hueco_medido / 2; 
                if (ticks_giro_in > 6) { ticks_giro_in = 6; }
                if (ticks_giro_in < 5) { ticks_giro_in = 5; }

                if (timer_maniobra >= ticks_giro_in) {
                    frenar(); 
                    delay(800);
                    
                    ticks_giro_realizado = timer_maniobra; 
                    timer_seguridad = 0; 
                    estado = MODO_ATRAS;
                }

                break;
            case MODO_ALINEAR:
                uartWriteString(UART_BT, "Alineando..\r\n");
                avanzar();
                delay(T_ALINEAR);
                frenar(); delay(800); 
                estado = MODO_GIRO_IN;
                break;


            case MODO_ATRAS:
                 uartWriteString(UART_BT, "Reversa diagonal hacia el lugar...\r\n");
                retroceder();
                 int limite_reversa = ticks_hueco_medido / 2; 
                
                //lim max
                if (limite_reversa > 6) { 
                    limite_reversa = 6; 
                }

                // lim min
                if (limite_reversa < 5) {
                    limite_reversa = 5; 
                }

                if (distancia_lateral_inicial > 10) { //ajuste de reversa segun que tan lejos esta el auto de los obstaculos
                    int extra_profundidad = (distancia_lateral_inicial - 10) / 2;
                    if (extra_profundidad > 2) {
                        extra_profundidad = 2;
                    }
                    limite_reversa += extra_profundidad; 
                }

                if (timer_seguridad >= limite_reversa) {
                    frenar();
                    uartWriteString(UART_BT, "Reversa diagonal finalizada\r\n");
                    delay(800);
                    
                    timer_maniobra = 0; 
                    estado = MODO_ACOMODO_ADELANTE; 
                }
                timer_seguridad++; 
                break;


            case MODO_ACOMODO_ADELANTE:
                uartWriteString(UART_BT, "Acomodando trompa hacia el cordon...\r\n");

                meter_trompa_adelante_logic(); 
                timer_maniobra++;
                if (timer_maniobra >= 3) {
                    frenar(); 
                    delay(800);
                   ticks_adelante_realizado = timer_maniobra; 
                    timer_maniobra = 0;
                    estado = MODO_GIRO_OUT;
                }
                break;

            case MODO_GIRO_OUT:
                uartWriteString(UART_BT, "Enderezando trompa (reversa)...\r\n");
                enderezar_trompa_logic();
                timer_maniobra++;
            
               int ticks_giro_out = ticks_giro_realizado - 2; 

                if (ticks_giro_out < 3) {
                    ticks_giro_out = 3; 
                }

                if (timer_maniobra >= ticks_giro_out) {
                    frenar();
                    uartWriteString(UART_BT, "Auto alineado en el hueco\r\n");
                    delay(800);
                    pasos_centrado = 0;
                    estado = MODO_CENTRANDO;
                }
                break;
            case MODO_ESTACIONADO:
                frenar();
                gpioToggle(LEDB); gpioToggle(LEDG);
                if (!gpioRead(TEC1)) estado = MODO_ESPERA; 
                //pasos_centrado = 0;
                //estado = MODO_CENTRANDO;
                break;

            case MODO_OBSTACULO:
                frenar();
                gpioWrite(LEDR, ON); 
                if (d_Frente > DIST_MINIMA) {
                    delay(1000);
                    estado = MODO_ESPERA; 
                }
                break;
            case MODO_CENTRANDO: 
                frenar();

                uint32_t f = leer_cm(TRIG_FRENTE, ECHO_FRENTE); delay(15);
                uint32_t a = leer_cm(TRIG_ATRAS,  ECHO_ATRAS);  delay(15);

                if (f == 0 || a == 0) {
                    pasos_centrado++;
                    uartWriteString(UART_BT, "CENTRANDO: lectura invalida\r\n");
                    if (pasos_centrado > MAX_PASOS_CENTRADO) estado = MODO_ESTACIONADO;
                    break;
                }

                uint32_t promedio = (f + a) / 2;

                sprintf(bufferMsg, "CENTRANDO -> F:%lu | A:%lu | Prom:%lu\r\n",
                        (unsigned long)f, (unsigned long)a, (unsigned long)promedio, pasos_centrado);
                uartWriteString(UART_BT, bufferMsg);

                // Ya centrado
                uint32_t errF = (f > promedio) ? (f - promedio) : (promedio - f);
                uint32_t errA = (a > promedio) ? (a - promedio) : (promedio - a);
                if (errF <= TOL_CENTRADO_CM && errA <= TOL_CENTRADO_CM) {
                    uartWriteString(UART_BT, "CENTRANDO: OK\r\n");
                    estado = MODO_ESTACIONADO;
                    break;
                }

                pasos_centrado++;
                if (pasos_centrado > MAX_PASOS_CENTRADO) {
                    uartWriteString(UART_BT, "CENTRANDO: timeout\r\n");
                    estado = MODO_ESTACIONADO;
                    break;
                }

                // Mas espacio adelante ? avanzar (pero respetar distancia minima adelante)
                if (f > a && f > (promedio + TOL_CENTRADO_CM)) {
                    if (f <= DIST_MINIMA + TOL_CENTRADO_CM) {  // <-- guard correcto
                        uartWriteString(UART_BT, "CENTRANDO: limite adelante\r\n");
                        estado = MODO_ESTACIONADO;
                        break;
                    }
                    uartWriteString(UART_BT, "CENTRANDO: avanzando\r\n");
                    avanzar();
                    delay(PULSO_CENTRADO_MS);
                    frenar();
                    delay(PAUSA_CENTRADO_MS);
                    break;
                }

                // Mas espacio atras ? retroceder (pero respetar distancia minima atras)
                if (a > f && a > (promedio + TOL_CENTRADO_CM)) {
                    if (a <= DIST_MINIMA_ATRAS + TOL_CENTRADO_CM) {  // <-- guard correcto
                        uartWriteString(UART_BT, "CENTRANDO: limite atras\r\n");
                        estado = MODO_ESTACIONADO;
                        break;
                    }
                    uartWriteString(UART_BT, "CENTRANDO: retrocediendo\r\n");
                    retroceder();
                    delay(PULSO_CENTRADO_MS);
                    frenar();
                    delay(PAUSA_CENTRADO_MS);
                    break;
                }

                break;
            }    

        delay(50); 
    }
}