
from machine import Pin
from machine import I2C
import time

#model = 'MS5837_02BA'
_model = 'MS5837_30BA'

SCL=22
SDA=23

TSYS01_ADDR = const(0x77)

MS5837_ADDR=const(0x76)
MS5837_RESET=const(0x1E)
MS5837_ADC_READ=const(0x00)
MS5837_PROM_READ=const(0xA0)
MS5837_CONVERT_D1_8192=const(0x4A)
MS5837_CONVERT_D2_8192=const(0x5A)

# Oversampling options
OSR_256  = 0
OSR_512  = 1
OSR_1024 = 2
OSR_2048 = 3
OSR_4096 = 4
OSR_8192 = 5

# kg/m^3 convenience
DENSITY_FRESHWATER = 997
DENSITY_SALTWATER = 1029

# Conversion factors (from native unit, mbar)
UNITS_Pa     = 100.0
UNITS_hPa    = 1.0
UNITS_kPa    = 0.1
UNITS_mbar   = 1.0
UNITS_bar    = 0.001
UNITS_atm    = 0.000986923
UNITS_Torr   = 0.750062
UNITS_psi    = 0.014503773773022

# Valid units
UNITS_Centigrade = 1
UNITS_Farenheit  = 2
UNITS_Kelvin     = 3


MS5837_30BA = 0
MS5837_02BA = 1

# needed?
#from ctypes import c_uint8,c_uint16

def reset():
    writeto(MS5837_ADDR,MS5837_RESET)
    time.sleep(0.01)
    

def readfrom_mem_2(address,register):
    data = i2c.readfrom_mem(address,register,2)
    value = data[0] << 8 | data[1]
    return value


def writeto(address,value):
    temp=bytearray(1)
    temp[0]=value
    i2c.writeto(address,temp)

def getCalibration():
    C=[]
    for i in range(0,8):
        register=MS5837_PROM_READ+i*2
        C.append(readfrom_mem_2(MS5837_ADDR,register))
    return C

def convert():
    writeto(MS5837_ADDR,MS5837_CONVERT_D1_8192)
    time.sleep(0.02)
    writeto(MS5837_ADDR,MS5837_ADC_READ)

def readfrom_3(address):
    data = i2c.readfrom(address,3)
    value = 0
    value = data[0]
    value = (value << 8) | data[1]
    value = (value << 8) | data[2]
    return value

def read():
    writeto(MS5837_ADDR,MS5837_CONVERT_D1_8192)
    time.sleep(0.02)
    writeto(MS5837_ADDR,MS5837_ADC_READ)

    D1 = readfrom_3(MS5837_ADDR)

    writeto(MS5837_ADDR,MS5837_CONVERT_D2_8192)
    time.sleep(0.02)
    
    writeto(MS5837_ADDR,MS5837_ADC_READ)
    
    D2 = readfrom_3(MS5837_ADDR)
    return D1,D2

def calculate(C,D1,D2):
    dT = 0
    SENS = 0
    OFF = 0
    SENSi = 0
    OFFi = 0
    Ti = 0
    OFF2 = 0
    SENS2 = 0
    

    
    dT = D2-C[5]*256

    if ( _model == 'MS5837_02BA' ):
        SENS = C[1]*65536+C[3]*dT/128
        OFF = C[2]*131072+C[4]*dT/64
        P = D1*SENS/2097152-OFF/32768
    else:
        SENS = C[1]*32768+C[3]*dT/256
        OFF = C[2]*65536+C[4]*dT/128
        P = D1*SENS/2097152-OFF/8192


    TEMP = 2000+dT*C[6]/8388608

    if ( _model == 'MS5837_02BA' ):
        if((TEMP/100)<20):         #Low temp
            Ti = 11*dT*dT/34359738368
            OFFi = (31*(TEMP-2000)*(TEMP-2000))/8
            SENSi = (63*(TEMP-2000)*(TEMP-2000))/32
        
    else:
        if((TEMP/100)<20):         #Low temp
            Ti = (3*dT*dT)/8589934592;
            OFFi = (3*(TEMP-2000)*(TEMP-2000))/2
            SENSi = (5*(TEMP-2000)*(TEMP-2000))/8
            if((TEMP/100)<-15):    #Very low temp
                OFFi = OFFi+7*(TEMP+1500)*(TEMP+1500)
                SENSi = SENSi+4*(TEMP+1500)*(TEMP+1500)
        elif((TEMP/100)>=20):    #High temp
            Ti = 2*(dT*dT)/(137438953472)
            OFFi = (1*(TEMP-2000)*(TEMP-2000))/16
            SENSi = 0

    
    OFF2 = OFF-OFFi           #Calculate pressure and temp second order
    SENS2 = SENS-SENSi
    
    if ( _model == 'MS5837_02BA' ):
        TEMP = (TEMP-Ti)
        P = (((D1*SENS2)/2097152-OFF2)/32768)/100
    else:
        TEMP = (TEMP-Ti)
        P = (((D1*SENS2)/2097152-OFF2)/8192)/10
    
    #print(TEMP)
    #print("C",C)
    #print("dT",dT)
    #print("C6",C[6])
    return TEMP,P

def temperature(TEMP):
    return TEMP/100.

def pressure(P,conversion):
    return P*conversion



#    if ( model == 'MS5837_02BA' ):
#        SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l
#        OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l
#        P = (D1*SENS/(2097152l)-OFF)/(32768l)
#    else:
#        SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l
#        OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l
#        P = (D1*SENS/(2097152l)-OFF)/(8192l)

#    TEMP = 2000l+int64_t(dT)*C[6]/8388608LL


def crc4(n_prom):
    n_rem = 0
    
    n_prom[0] = ((n_prom[0]) & 0x0FFF)
    n_prom.append(0)

    for i in range(16):
        if i%2 == 1:
            n_rem ^= ((n_prom[i>>1]) & 0x00FF)
        else:
            n_rem ^= (n_prom[i>>1] >> 8)
            
        for n_bit in range(8,0,-1):
            if n_rem & 0x8000:
                n_rem = (n_rem << 1) ^ 0x3000
            else:
                n_rem = (n_rem << 1)

    n_rem = ((n_rem >> 12) & 0x000F)


    return n_rem ^ 0x00





i2c = I2C(-1, Pin(SCL), Pin(SDA))

reset()
#20481
#5

C= getCalibration()

# check crc4

crc4_read = (C[0] & 0xF000) >> 12
crc4_calc=crc4(C)

print(crc4_read,crc4_calc)

#print(C)

D1,D2 = read()

TEMP,P = calculate(C,D1,D2)

print("temperature=",temperature(TEMP))
print("pressure=",pressure(P,UNITS_mbar)) #mbar

#print(temp)
