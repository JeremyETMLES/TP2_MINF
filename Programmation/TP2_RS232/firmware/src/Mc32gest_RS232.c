// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'�mission et de r�ception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoy� r�ponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

// Structure d�crivant le message
typedef struct {
    int8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    int8_t MsbCrc;
    int8_t LsbCrc;
} StruMess;


// Struct pour �mission des messages
StruMess TxMess;
// Struct pour r�ception des messages
StruMess RxMess;

// Declaration des FIFO pour r�ception et �mission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de r�ception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'�mission
S_fifo descrFifoTX;

// D�claration des constantes pour GetMessage
#define INITCRC 0xFFFF
//#define STARTCHAR 0xAA
#define SIZEBYTE 8
#define NBCYLE_MAX 10


// Initialisation de la communication s�rielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de r�ception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'�mission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit �mission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message re�u donc local (data non modifi�)
// Valeur de retour 1  = message re�u donc en remote (data mis � jour)
int GetMessage(S_pwmSettings *pData)
{
    int commStatus = 0; // �tat de la communication
    static uint8_t i = NBCYLE_MAX;   // Compteur de cycle sans message
    uint8_t NbCharToRead = 0;   // Sauvegarde du nombre de caract�res du message � lire
    U_manip16 crc;    // Sauvegarde de la valeur du CRC sur 16 bits
    uint16_t ValCrc16 = INITCRC; // Variable de calcul du CRC sur 16 bits    
    
    // Lecture et d�codage fifo r�ception
    //Sauvegarder la taille du message dans NbCharToRead;
    NbCharToRead = GetReadSize(&descrFifoRX);
	if (NbCharToRead >= MESS_SIZE)
	{
		//R�cup�rer la valeur re�ue en la sauvegardant � l'adresse de debut;
        GetCharFromFifo(&descrFifoRX, &RxMess.Start);
        //RxMess.Start = RxMess.Start ; 
		if (RxMess.Start == STX_code)
		{
            // R�cup�rer la valeur re�ue en la sauvegardant � l'adresse de vitesse dans pDataTemporaire;
            GetCharFromFifo(&descrFifoRX, &RxMess.Speed);
            // R�cup�rer la valeur re�ue en la sauvegardant � l'adresse de angle dans pDataTemporaire;
            GetCharFromFifo(&descrFifoRX, &RxMess.Angle);
            // R�cup�rer la valeur re�ue en la sauvegardant � l'adresse de crc;
            GetCharFromFifo(&descrFifoRX, &RxMess.MsbCrc);
            // D�caler crc � gauche de 8;
            crc.val = RxMess.MsbCrc << SIZEBYTE;
            // R�cup�rer la valeur re�ue en la sauvegardant � l'adresse de lsb;
            GetCharFromFifo(&descrFifoRX, &RxMess.LsbCrc);
			crc.shl.lsb = RxMess.LsbCrc;
            
            // Calcul du CRC
			ValCrc16 = updateCRC16(ValCrc16, STX_code);
			ValCrc16 = updateCRC16(ValCrc16, RxMess.Speed);
			ValCrc16 = updateCRC16(ValCrc16, RxMess.Angle);
            
            // Si le message est correcte
			if (crc.val == ValCrc16)
			{
                // Mise � jour de pData
                pData->SpeedSetting = RxMess.Speed;
                pData->absSpeed = abs(RxMess.Speed);
				pData->AngleSetting = RxMess.Angle;
                
                // Communication OK, 0 cycles NOK
				i = 0;
                //commStatus = 1;
			}
		}
	}
    // Si le message est incorrect/pas de message pendant 10 cycles
	if (i >= NBCYLE_MAX)
	{
        //i = 0;
        commStatus = 0;
	}
	else
	{
        // Le d�lai n'est pas encore d�pass�.
		i++;
		commStatus = 1;
	}    
    
    // Gestion controle de flux de la r�ception
    if(GetWriteSpace(&descrFifoRX) >= (2*MESS_SIZE)) {
        // autorise �mission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage


// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
    int8_t freeSize;
    
    uint16_t ValCrc16 = 0xFFFF;
       
    // Gestion du controle de flux
    // si on a un caract�re � envoyer et que CTS = 0
    freeSize = GetWriteSpace(&descrFifoTX);
    
    //(RS232_CTS == 0) &&
    if ( (freeSize >= MESS_SIZE))
    {
        // Autorise int �mission    
        //PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        //================ CRC ==================//
        
        ValCrc16 = updateCRC16(ValCrc16, STX_code);
        ValCrc16 = updateCRC16(ValCrc16, pData->SpeedSetting);
        ValCrc16 = updateCRC16(ValCrc16, pData->AngleSetting);

        //=======================================//

        // Traitement �mission � introduire ICI
        // Formatage message et remplissage fifo �mission
        TxMess.Start  = STX_code;
        TxMess.Speed  = pData->SpeedSetting;
        TxMess.Angle  = pData->AngleSetting;
        TxMess.MsbCrc = ValCrc16 >> 8;
        TxMess.LsbCrc = ValCrc16 & 0xFF;
        
        //Depos des messages dans le fifo
        PutCharInFifo (&descrFifoTX, TxMess.Start);     //Byte de start
        PutCharInFifo (&descrFifoTX, TxMess.Speed);     //Byte pour vitesse
        PutCharInFifo (&descrFifoTX, TxMess.Angle);     //Byte pour angle
        PutCharInFifo (&descrFifoTX, TxMess.MsbCrc);    //Byte pour le MSB du CRC
        PutCharInFifo (&descrFifoTX, TxMess.LsbCrc);    //Byte pour le LSB du CRC
        
        
    }
    
    freeSize = GetWriteSpace(&descrFifoTX);
    if ((RS232_CTS == 0) && (freeSize > 0))
    {        
        // Autorise int �mission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}


// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la r�ponse g�n�r�e dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;
    
    int8_t charFifoUsart;   // Variable de r�cup�ration d'une data de la Fifo Usart
    
    int8_t byteTransmis;    // Variable de transmission d'une data de la Fifo Usart

    // Variable pour l'interruption Tx
    uint8_t sizeBufferSoft = 0;
    uint8_t bufferHardFull = 0;
     
    // Marque d�but interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur � la r�ception.
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
        {
            PLIB_USART_ReceiverByteReceive(USART_ID_1);
        }
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {

        // Oui Test si erreur parit� ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            // Traitement RX � faire ICI
            // Lecture des caract�res depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            
            // Tant qu'il y a des donn�es re�ues
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                // R�cup�ration message Fifo Usart
                charFifoUsart = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                // Copie sur Fifo software
                PutCharInFifo(&descrFifoRX, charFifoUsart);
            }
            
            
                         
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        
        // Traitement controle de flux reception � faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        // Gestion controle de flux de la r�ception
        if(GetWriteSpace(&descrFifoRX) < (2*MESS_SIZE)) {
            // interdit �mission par l'autre
            RS232_RTS = 0;
        }

        
    } // end if RX

    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) {

        // Traitement TX � faire ICI
        // Envoi des caract�res depuis le fifo SW -> buffer HW
            
        // Avant d'�mettre, on v�rifie 3 conditions :
        //  Si CTS = 0 autorisation d'�mettre (entr�e RS232_CTS)
        //  S'il y a un carat�res � �mettre dans le fifo
        //  S'il y a de la place dans le buffer d'�mission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
       sizeBufferSoft = GetReadSize(&descrFifoTX);
       bufferHardFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
        
        if(RS232_CTS == 0)
        {
            //PLIB_USART_TransmitterEnable(USART_ID_1);
            
            if(sizeBufferSoft && bufferHardFull == false)
            {
                while(RS232_CTS == 0 && sizeBufferSoft && bufferHardFull == false)
                {
                    GetCharFromFifo(&descrFifoTX, &byteTransmis);
                    PLIB_USART_TransmitterByteSend(USART_ID_1, byteTransmis);
                    sizeBufferSoft = GetReadSize(&descrFifoTX);
                    bufferHardFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
                }
            }

        }
        else
        {
            PLIB_USART_TransmitterDisable(USART_ID_1);
        }
  
        LED5_W = !LED5_R; // Toggle Led5
		
        // disable TX interrupt (pour �viter une interrupt. inutile si plus rien � transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        // Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
 }




