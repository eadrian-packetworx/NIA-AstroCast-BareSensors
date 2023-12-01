void ChargeCurrent(byte current){
  for(int x = 0; x < current; x++){
    chargePulse();
  }
};

void chargePulse(){
  digitalWrite(CHARGER_CTRL, HIGH);
  delayMicroseconds(500);
  digitalWrite(CHARGER_CTRL, LOW);
  delayMicroseconds(500);
}
