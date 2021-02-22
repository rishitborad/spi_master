/*************************************************************
 * File name: xbowspBITStatus.h
 *
 *  public named BIT flags
 *  public functions
 *
 * $Rev: 15645 $
* $Date: 2010-07-21 15:53:14 -0700 (Wed, 21 Jul 2010) $
* $Author: by-dmathis $
*
 *
 ************************************************************/

#if ! defined BITSTATUS_H
#define BITSTATUS_H
#include "dmu.h"
/*
 * 'class' depth/location in hierarchy
 */
typedef enum {    

     BITCAT_START,
#   define BIT_CAT(name, mema) name,
#   include "xbowsp_bit.def"
#   undef  BIT_CAT
        N1_BITCAT
 
} CAT_BIT;

#define BF(maj, min)   ( ( (maj) << 16) | ( min ) ) 
#define MIN_ALL 0xFFFF

typedef enum {
/* hardware error bits */
     /* low level voltage flags */
     BITERR_TWOVOLTIOUP = BF(BITERR_LOWVOLT, 1),
     BITERR_TWOVOLTDUP  = BF(BITERR_LOWVOLT, 2),
     BITERR_TWOFIVEREF  = BF(BITERR_LOWVOLT, 4),
     BITERR_THREEVOLT   = BF(BITERR_LOWVOLT, 8),
     BITERR_FIVEVOLTA   = BF(BITERR_LOWVOLT, 0x10),
     BITERR_FIVEVOLTAG  = BF(BITERR_LOWVOLT, 0x20),
     BITERR_FIVEVOLTD   = BF(BITERR_LOWVOLT, 0x40),
     BITERR_SEVENVOLTPWR = BF(BITERR_LOWVOLT, 0x80),

     /* low level sensor current flags */
     BITERR_ACCL1CURRENT  = BF(BITERR_LOWSENS, 1),
     BITERR_ACCL2CURRENT  = BF(BITERR_LOWSENS, 2),
     BITERR_ACCL3CURRENT  = BF(BITERR_LOWSENS, 4),
     BITERR_RATE1CURRENT  = BF(BITERR_LOWSENS, 8),
     BITERR_RATE2CURRENT  = BF(BITERR_LOWSENS, 0x10),
     BITERR_RATE3CURRENT  = BF(BITERR_LOWSENS, 0x20),
     BITERR_STATICPRESSEXCITE = BF(BITERR_LOWSENS, 0x40),
     BITERR_DYNPRESSEXCITE = BF(BITERR_LOWSENS, 0x80),
     BITERR_OATCURRENT    = BF(BITERR_LOWSENS, 0x100),

     BITERR_INERTIALCURRENT = (BITERR_ACCL1CURRENT | 
			       BITERR_ACCL2CURRENT | 
			       BITERR_ACCL3CURRENT |
			       BITERR_RATE1CURRENT | 
			       BITERR_RATE2CURRENT | 
			       BITERR_RATE3CURRENT),

     /* low level sensor temperature flags */
     BITERR_ACCEL1TEMP   = BF(BITERR_LOWENV, 1),
     BITERR_ACCEL2TEMP   = BF(BITERR_LOWENV, 2),
     BITERR_ACCEL3TEMP   = BF(BITERR_LOWENV, 4),
     BITERR_RATE1TEMP    = BF(BITERR_LOWENV, 8),
     BITERR_RATE2TEMP    = BF(BITERR_LOWENV, 0x10),
     BITERR_RATE3TEMP    = BF(BITERR_LOWENV, 0x20),
     BITERR_STATICPRESSTEMP = BF(BITERR_LOWENV, 0x40),
     BITERR_DYNAMICPRESSTEMP = BF(BITERR_LOWENV, 0x80),

     BITERR_INERTIALTEMP = (BITERR_ACCEL1TEMP | 
			    BITERR_ACCEL2TEMP | 
			    BITERR_ACCEL3TEMP |
			    BITERR_RATE1TEMP | 
			    BITERR_RATE2TEMP | 
			    BITERR_RATE3TEMP),
    
     /* low level internal comm flags */
     BITERR_SENSORDATAERROR = BF(BITERR_LOWICOMM, 1),
     BITERR_A2DERROR        = BF(BITERR_LOWICOMM, 2),
     BITERR_IOUP_NOCOMM     = BF(BITERR_LOWICOMM, 4), 
     BITERR_IOUP_NOLOAD     = BF(BITERR_LOWICOMM, 8),
     BITERR_LOWICOMM_NF      =  BITERR_IOUP_NOLOAD,
                    
     /* mid level hw flags */
     BITERR_HWVOLT          = BF(BITERR_MIDHW, 1),
     BITERR_HWINERTIAL_SENS = BF(BITERR_MIDHW, 2),
     BITERR_HWINERTIAL_ENV  = BF(BITERR_MIDHW, 4),
     BITERR_HWICOMM         = BF(BITERR_MIDHW, 8),
    
     BITERR_HWAD_ENV        = BF(BITERR_MIDHW, 0x80),

     /* Flow up flags */
     BITERR_HWMID_ALL = ( BITERR_HWVOLT | 
			  BITERR_HWINERTIAL_SENS | 
			  BITERR_HWINERTIAL_ENV | 
			  BITERR_HWICOMM),


/* external comm error bits */
#define TRANSOVR   1
#define RCVOVR     2
#define PARITYERR  4
#define FRAMERR    8
#define BREAKERR  16 
#define CBUFOVR   32     

     /* low user comm */
     BITERR_USER_TRANSBUFOVERFLOW = BF(BITERR_LOWUCOMM, TRANSOVR),
     BITERR_USER_RECBUFOVERFLOW   = BF(BITERR_LOWUCOMM, RCVOVR),
     BITERR_USER_FRAMINGERROR     = BF(BITERR_LOWUCOMM, FRAMERR),
     BITERR_USER_BREAKDETECT      = BF(BITERR_LOWUCOMM, BREAKERR),
     BITERR_USER_PARITYERROR      = BF(BITERR_LOWUCOMM, PARITYERR),
     BITERR_USER_CBUFOVFERROR     = BF(BITERR_LOWUCOMM, CBUFOVR),

     /* mag comm */
     BITERR_MAG_TRANSBUFOVERFLOW = BF(BITERR_LOWMCOMM, TRANSOVR),
     BITERR_MAG_RECBUFOVERFLOW   = BF(BITERR_LOWMCOMM, RCVOVR),
     BITERR_MAG_FRAMINGERROR     = BF(BITERR_LOWMCOMM, FRAMERR),
     BITERR_MAG_BREAKDETECT      = BF(BITERR_LOWMCOMM, BREAKERR),
     BITERR_MAG_PARITYERROR      = BF(BITERR_LOWMCOMM, PARITYERR),
     BITERR_MAG_CBUFOVFERROR     = BF(BITERR_LOWMCOMM, CBUFOVR),

    /* port 0 comm */
     BITERR_PORT0_TRANSBUFOVERFLOW = BF(BITERR_LOWPORT0, TRANSOVR),
     BITERR_PORT0_RECBUFOVERFLOW   = BF(BITERR_LOWPORT0, RCVOVR),
     BITERR_PORT0_FRAMINGERROR     = BF(BITERR_LOWPORT0, FRAMERR),
     BITERR_PORT0_BREAKDETECT      = BF(BITERR_LOWPORT0, BREAKERR),
     BITERR_PORT0_PARITYERROR      = BF(BITERR_LOWPORT0, PARITYERR),
     BITERR_PORT0_CBUFOVFERROR     = BF(BITERR_LOWPORT0, CBUFOVR),

   /* port 1 comm */
     BITERR_PORT1_TRANSBUFOVERFLOW = BF(BITERR_LOWPORT1, TRANSOVR),
     BITERR_PORT1_RECBUFOVERFLOW   = BF(BITERR_LOWPORT1, RCVOVR),
     BITERR_PORT1_FRAMINGERROR     = BF(BITERR_LOWPORT1, FRAMERR),
     BITERR_PORT1_BREAKDETECT      = BF(BITERR_LOWPORT1, BREAKERR),
     BITERR_PORT1_PARITYERROR      = BF(BITERR_LOWPORT1, PARITYERR),
     BITERR_PORT1_CBUFOVFERROR     = BF(BITERR_LOWPORT1, CBUFOVR),
    /* port 2 comm */
     BITERR_PORT2_TRANSBUFOVERFLOW = BF(BITERR_LOWPORT2, TRANSOVR),
     BITERR_PORT2_RECBUFOVERFLOW   = BF(BITERR_LOWPORT2, RCVOVR),
     BITERR_PORT2_FRAMINGERROR     = BF(BITERR_LOWPORT2, FRAMERR),
     BITERR_PORT2_BREAKDETECT      = BF(BITERR_LOWPORT2, BREAKERR),
     BITERR_PORT2_PARITYERROR      = BF(BITERR_LOWPORT2, PARITYERR),
     BITERR_PORT2_CBUFOVFERROR     = BF(BITERR_LOWPORT2, CBUFOVR),
   /* port 3 comm */
     BITERR_PORT3_TRANSBUFOVERFLOW = BF(BITERR_LOWPORT3, TRANSOVR),
     BITERR_PORT3_RECBUFOVERFLOW   = BF(BITERR_LOWPORT3, RCVOVR),
     BITERR_PORT3_FRAMINGERROR     = BF(BITERR_LOWPORT3, FRAMERR),
     BITERR_PORT3_BREAKDETECT      = BF(BITERR_LOWPORT3, BREAKERR),
     BITERR_PORT3_PARITYERROR      = BF(BITERR_LOWPORT3, PARITYERR),
     BITERR_PORT3_CBUFOVFERROR     = BF(BITERR_LOWPORT3, CBUFOVR),

     /* mid level comm flags */
     BITERR_UCOMM = BF(BITERR_MIDCOMM, 1),
     BITERR_MCOMM = BF(BITERR_MIDCOMM, 2),

     BITERR_COMMID_ALL = (BITERR_UCOMM | 
			  BITERR_MCOMM),

/* software error bits */
     /* algorithm errors */ 
     BITERR_OVERRANGE            = BF(BITERR_LOWALGO, 2),
     BITERR_OVR_FLT              = BF(BITERR_LOWALGO, 8), /* filtered   BITERR_OVERRANGE   */
     
     BITERR_LOWALGO_ALL = (BITERR_OVR_FLT),

     /* data errors */
     BITERR_CALDATAERROR        = BF(BITERR_LOWDATA, 1),

     BITERR_OATOUTOFRANGE       = BF(BITERR_LOWDATA, 4),
     BITERR_SEPROMFAIL          = BF(BITERR_LOWDATA, 8),
     BITERR_LOWDATA_NF          = (BITERR_CALDATAERROR | 
				   				   BITERR_SEPROMFAIL ), /* unfiltered */

     /* mid level software errors */
     BITERR_SWALGO               = BF(BITERR_MIDSW, 1), 
     BITERR_SWDATA               = BF(BITERR_MIDSW, 2),


     BITERR_SWMID_ALL            = (BITERR_SWALGO | 
				    BITERR_SWDATA),

     /* high level hierarchy error */
     BITERR_FAIL = BF(BITERR_TOP, 1),
     BITERR_HW   = BF(BITERR_TOP, 2),       /* excludes air data temp */
     BITERR_COMM = BF(BITERR_TOP, 4),
     BITERR_SW   = BF(BITERR_TOP, 8),


     /* observe MUST use ORing */
     /* comm never is in the top fail */
     BITERR_TOPALL= (BITERR_HW | 
		     BITERR_SW),

     BITERR_TOPPKT = (BITERR_FAIL | 
		      BITERR_HW | 
		      BITERR_COMM | 
		      BITERR_SW),

/**********************************************************************/
/*                     STATUS  */

     /* status hardware mid level */
     BITSTAT_UNLOCKEDEEPROM   = BF(BITSTAT_MIDHW, 8),
     BITSTAT_OATFAIL          = BF(BITSTAT_MIDHW, 0x20),  /* filtered raw counts, disconnected/shorted */
     BITSTAT_EXTMAGFAIL       = BF(BITSTAT_MIDHW, 0x40),
     BITSTAT_INTAIRDATA       = BF(BITSTAT_MIDHW, 0x80), /* intermediate result flag */
     BITSTAT_HWMID            = BF(BITSTAT_MIDHW, 0x8000),
     /* to set BITSTAT_INTAIRDATA ... */
     BITSTAT_INTAIRDATASRC1 = (BITERR_STATICPRESSEXCITE | 
			       BITERR_DYNPRESSEXCITE),
     BITSTAT_INTAIRDATASRC2 = (BITERR_STATICPRESSTEMP | 
			       BITERR_DYNAMICPRESSTEMP),
     BITSTAT_HWMID_ALL =   (BITSTAT_UNLOCKEDEEPROM | 
			    BITSTAT_OATFAIL |
			    BITSTAT_EXTMAGFAIL| 
			    BITSTAT_INTAIRDATA),
     
     /* status sensor mid level */
     BITSTAT_OVERRANGE          = BF(BITSTAT_MIDSENS, 1),
     BITSTAT_MAGALIGNINVALID    = BF(BITSTAT_MIDSENS, 4),
     BITSTAT_MAGSN              = BF(BITSTAT_MIDSENS, 0x10), 
     BITSTAT_NOAIRDATAAIDING    = BF(BITSTAT_MIDSENS, 0x20),
     BITSTAT_SENSMID            = BF(BITSTAT_MIDSENS, 0x8000),
     BITSTAT_MAGALIGN =   (BITSTAT_MAGSN | 
			   BITSTAT_MAGALIGNINVALID ),
     BITSTAT_SENSMID_ALL =    (BITSTAT_OVERRANGE | 
			       BITSTAT_MAGALIGNINVALID |
			       BITSTAT_MAGSN ),

     /* status mid comm */
     BITSTAT_NOEXTERNALMAG      = BF(BITSTAT_MIDCOMM, 2),   /* communication failure */
     BITSTAT_NOEXTERNALAIRDATA  = BF(BITSTAT_MIDCOMM, 4),
     BITSTAT_COMMID            = BF(BITSTAT_MIDCOMM, 0x8000),
     BITSTAT_COMMID_ALL = (BITSTAT_NOEXTERNALMAG  | 
		       BITSTAT_NOEXTERNALAIRDATA),

     /* mid software */
     BITSTAT_ALGORITHMINIT      = BF(BITSTAT_MIDSW, 1),
     BITSTAT_ATTITUDEONLYALGORITHM = BF(BITSTAT_MIDSW, 4),
     BITSTAT_TURNSWITCH         = BF(BITSTAT_MIDSW, 8),
     BITSTAT_NOMAGHEADINGREF    = BF(BITSTAT_MIDSW, 0x20), 
     BITSTAT_VGFORCE            = BF(BITSTAT_MIDSW, 0x200),   
     BITSTAT_SWMID              = BF(BITSTAT_MIDSW, 0x8000),
     BITSTAT_SWMID_ALL =    (BITSTAT_ALGORITHMINIT | 
			 BITSTAT_TURNSWITCH |
			 BITSTAT_NOMAGHEADINGREF | 
			 BITSTAT_VGFORCE ),

 /* top STATUS */

    /* high level STATUS fails */
     BITFAIL_MASTERFAIL = BF(BITSTAT_TOP, 1), /* is BITERR_MasterFail */
     
     BITMODE_ALGOINIT = BF(BITSTAT_TOP, 2),
     BITFAIL_ATTITUDE = BF(BITSTAT_TOP, 4),
     BITFAIL_HEADING  = BF(BITSTAT_TOP, 8),
     BITFAIL_AIRDATA  = BF(BITSTAT_TOP, 0x10),
     BITFAIL_MAGALIGN = BF(BITSTAT_TOP, 0x20),
     BITFAIL_OATFAIL  = BF(BITSTAT_TOP, 0x40),   /* is BITSTAT_OAT */
     BITMODE_VGMODE   = BF(BITSTAT_TOP, 0x80),
   
     /* STATUS copied into top ERROR -- do not reassign bit positions  */
     BITSTAT_MASTERSTATUS    = BF(BITSTAT_TOP, 0x100),
     BITSTAT_HARDWARESTATUS  = BF(BITSTAT_TOP, 0x200),
     BITSTAT_COMSTATUS       = BF(BITSTAT_TOP, 0x400),
     BITSTAT_SOFTWARESTATUS  = BF(BITSTAT_TOP, 0x800),
     BITSTAT_SENSORSTATUS    = BF(BITSTAT_TOP, 0x1000), 

     BITSTAT_TOPALL =  (BITSTAT_HARDWARESTATUS | 
			BITSTAT_COMSTATUS |
                        BITSTAT_SOFTWARESTATUS | 
			BITSTAT_SENSORSTATUS),
 
     /* not packet status */
     BITSTAT_REMOTEMAGFAIL   = BF(BITSTAT_TOP, 0x2000),
     BITMODE_OPMODE          = BF(BITSTAT_TOP, 0x4000),
    

     BITSTAT_TOPPKT =  (BITSTAT_MASTERSTATUS | 
			BITSTAT_HARDWARESTATUS |
                        BITSTAT_COMSTATUS | 
			BITSTAT_SOFTWARESTATUS |
                        BITSTAT_SENSORSTATUS | 
			BITSTAT_REMOTEMAGFAIL |
                        BITMODE_OPMODE)
        
     
} BITFLAG;



extern void BITinit(void) ;


/* low level interface */
extern BOOL setBITflag(BITFLAG err, BOOL on) ;
extern int16_t getBITflag(BITFLAG err) ;
extern int32_t getBITflags(BITFLAG err);


/* top level handlers */
extern void handleBITandStatus(void) ;
extern void BIT_init(void);  
extern void loadBITlimits(void);


/* helper for output packets */
extern uint16_t getTopERROR(void) ;
extern uint16_t getTopSTATUS(void);       
#define  packetWord(category)       getBITflags(BF(category, MIN_ALL))
#define  hardwarePowerBIT()         packetWord(BITERR_LOWVOLT)
#define  hardwareSensorBIT()        packetWord(BITERR_LOWSENS) 
#define  hardwareEnvironmentalBIT() packetWord(BITERR_LOWENV)
#define  hwInternalCommBIT()        packetWord(BITERR_LOWICOMM)
#define  hardwareBIT()              packetWord(BITERR_MIDHW)
#define  comSerialABIT()            packetWord(BITERR_LOWUCOMM)
#define  comSerialBBIT()            packetWord(BITERR_LOWMCOMM)
#define  comBIT()                   packetWord(BITERR_LOWMCOMM)
#define  softwareAlgorithmBIT()     packetWord(BITERR_LOWALGO)
#define  softwareDataBIT()          packetWord(BITERR_LOWDATA)
#define  softwareBIT()              packetWord(BITERR_MIDSW)
#define  hardwareStatus()           packetWord(BITSTAT_MIDHW)
#define  comStatus()                packetWord(BITSTAT_MIDCOMM) 
#define  softwareStatus()           packetWord(BITSTAT_MIDSW)
#define  sensorStatus()             packetWord(BITSTAT_MIDSENS)

/* flag setting helpers */
extern void logUARTerrorBIT(uint16_t port, uint16_t comErrs);

#endif