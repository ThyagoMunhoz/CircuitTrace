#ifndef coolant_control_h
#define coolant_control_h

#define REFRIGERACAO_SEM_SINCRONIA     false
#define REFRIGERACAO_FORCAR_SINCRONIA  true

#define ESTADO_REFRIGERACAO_DESLIGADO   0  
#define ESTADO_REFRIGERACAO_INUNDACAO   PL_COND_FLAG_COOLANT_FLOOD
#define ESTADO_REFRIGERACAO_NEVOA       PL_COND_FLAG_COOLANT_MIST

void refrigeracao_iniciar();
uint8_t refrigeracao_obter_estado();
void refrigeracao_parar();
void refrigeracao_definir_estado(uint8_t modo);
void refrigeracao_sincronizar(uint8_t modo);

#endif