/**
  The pozyx ready to range demo (c) Pozyx Labs
  please check out https://www.pozyx.io/Documentation/Tutorials/getting_started
  
  This demo requires two pozyx devices and one Arduino. It demonstrates the ranging capabilities and the functionality to 
  to remotely control a pozyx device. Place one of the pozyx shields on the Arduino and upload this sketch. Move around
  with the other pozyx device.
  
  This demoe measures the range between the two devices. The closer the devices are to each other, the more LEDs will
  burn on both devices.
*/
#include <iostream>
#include <unistd>
#include <cstdint>
#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>


////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint16_t destination_id = 0x6670;     // the network id of the other pozyx device: fill in the network id of the other device
int range_step_mm = 1000;      // every 1000mm in range, one LED less will be giving light.

////////////////////////////////////////////////

int setup(){
  if(Pozyx.begin() == POZYX_FAILURE){
    std::cout << "ERROR: Unable to connect to POZYX shield\n";
    std::cout << "Reset required\n";
    return 1;
  }
  
  std::cout << "------------POZYX RANGING V1.0------------\n";
  std::cout << "NOTES:\n";
  std::cout << "- Change the parameters:\n\tdestination_id (target device)\n\trange_step (mm)\n\n";
  std::cout << "- Approach target device to see range and\n led control\n";
  std::cout << "------------POZYX RANGING V1.0------------\n\n";
  std::cout << "START Ranging:\n";
  
  // make sure the pozyx system has no control over the LEDs, we're the boss
  uint8_t configuration_leds = 0x0;    
  Pozyx.regWrite(POZYX_CONFIG_LEDS, &configuration_leds, 1);
  
  // do the same with the remote device
  Pozyx.remoteRegWrite(destination_id, POZYX_CONFIG_LEDS, &configuration_leds, 1);
  return 0;
}

void loop(){
  
  int status = 1;
  device_range_t range;
  
  // let's do ranging with the destination 
  status &= Pozyx.doRanging(destination_id, &range);
  
  if (status == POZYX_SUCCESS){
    std::cout << range.timestamp;
    std::cout << "ms \t";
    std::cout << range.distance; 
    std::cout << "mm \t";
    
    // now control some LEDs; the closer the two devices are, the more LEDs will be lit
    if (ledControl(range.distance) == POZYX_FAILURE){
      std::cout << "ERROR: setting (remote) leds\n";
    }    
  }
  else{
      std::cout << "ERROR: ranging\n";
  }
}

int ledControl(uint32_t range){
  int status = 1;
  
  // set the LEDs of this pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm));
  status &= Pozyx.setLed(3, (range < 2*range_step_mm));
  status &= Pozyx.setLed(2, (range < 3*range_step_mm));
  status &= Pozyx.setLed(1, (range < 4*range_step_mm));

  // set the LEDs of the remote pozyx device
  status &= Pozyx.setLed(4, (range < range_step_mm), destination_id);
  status &= Pozyx.setLed(3, (range < 2*range_step_mm), destination_id);
  status &= Pozyx.setLed(2, (range < 3*range_step_mm), destination_id);
  status &= Pozyx.setLed(1, (range < 4*range_step_mm), destination_id);

  // status will be zero if setting the LEDs failed somewhere along the way
  return status;
}

int main() {
  if(!setup()) {
    while(true) {
      loop();
    }
  }
  std::cout << "Aborted\n";
}