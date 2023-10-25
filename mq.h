const int calibrationLed=13;
const int MQ_PIN=A0;
int RL_VALUE=1;
float RO_CLEAN_AIR_FACTOR=9.86;

int CALIBARAION_SAMPLE_TIMES=50;
int CALIBRATION_SAMPLE_INTERVAL=500;

int READ_SAMPLE_INTERVAL=50;
int READ_SAMPLE_TIMES=5;

long iPPM_LPG = 0;
long iPPM_CO = 0;
long iPPM_Smoke = 0;

#define   GAS_LPG     0
#define   GAS_CO      1
#define   GAS_SMOKE   2

float     LPGCurve[3]     =   {2.3, 0.21, -0.47};

float     COCurve[3]      =   {2.3,0.72, -0.34};

float     SmokeCurve[3]   =   {2.3, 0.53, -0.44};

float     Ro   = 10;



float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}





float MQCalibration (int mq_pin)
{
  int i;
  float val=0;

  for(i=0; i<CALIBARAION_SAMPLE_TIMES;i++){
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;
  val = val/RO_CLEAN_AIR_FACTOR;
  return val;
}



float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0; i<READ_SAMPLE_TIMES; i++){
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  rs = rs/READ_SAMPLE_TIMES;

  return rs;
}

long MQGetGasPercentagee(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2])+pcurve[0])));
}


float MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if(gas_id == GAS_LPG){
    return MQGetGasPercentagee (rs_ro_ratio, LPGCurve);
  } else if (gas_id == GAS_CO){
     return MQGetGasPercentagee (rs_ro_ratio, COCurve);
  } else if (gas_id == GAS_SMOKE){
     return MQGetGasPercentagee (rs_ro_ratio, SmokeCurve);
  }
  return 0;
}


void Mqsetup()
{
  pinMode(calibrationLed, OUTPUT);
  digitalWrite(calibrationLed, HIGH);

  Ro= MQCalibration (MQ_PIN);
  digitalWrite(calibrationLed, LOW);

  delay(2000);
}

void Mqsense()
{
  iPPM_LPG = 0;
  iPPM_CO = 0;
  iPPM_Smoke = 0;

  iPPM_LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  iPPM_CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);

}