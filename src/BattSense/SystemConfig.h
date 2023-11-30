#define CHARGER_CTRL PD2//PC1
#define CHARGER_STAT PA14//C12  //pd2/rx pc12/tx

#define RSHUNT 0.002

#define ACSIP_SDA PB7
#define ACSIP_SCL PB6

#define _93MA   4
#define _187MA  5
#define _280MA  6
#define _374MA  7
#define _467MA  8
#define _654MA  9
#define _794MA  10
#define _935MA  11

struct PowerMonitor{
  float voltage;
  float current;
  float power;
};
