// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize

#include <iostream>
#include <unistd.h>
#include <cstdint>
#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include "Wire.h"


////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x1156, 0x256B, 0x3325, 0x4244};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[4] = {2750, 2000, 1900, 2350};              // anchor z-coordinates in mm
bool bProcessing = false;                                   // set this to true to output data for the processing sketch         

// only required for manual anchor calibration. Please change this to the coordinates measured for the anchors
int32_t anchors_x[4] = {0, 10000, 1000, 9000};              // anchor x-coorindates in mm
int32_t anchors_y[4] = {0, 0, 7000, 8000};                  // anchor y-coordinates in mm

////////////////////////////////////////////////

// function to print the coordinates to the serial monitor
void printCoordinates(coordinates_t coor){
  std::cout << "x_mm: " << coor.x << "\ty_mm: " << coor.y << "\tz_mm: " << coor.z << std::endl;
}

// function to print out positoining data + ranges for the processing sketch
void printCoordinatesProcessing(coordinates_t coor){
  
  // get the network id and print it
  uint16_t network_id;
  Pozyx.getNetworkId(&network_id);
  
  std::cout << "POS,0x" << std::hex <<  network_id << "," << coor.x << "," << coor.y << "," << coor.z << ","; 
  
  // get information about the positioning error and print it
  pos_error_t pos_error;
  Pozyx.getPositionError(&pos_error);
  
  std::cout << pos_error.x << "," << pos_error.y << "," << pos_error.z << "," << pos_error.xy << "," << pos_error.xz << "," << pos_error.yz;
  
  // read out the ranges to each anchor and print it 
  for (int i=0; i < num_anchors; i++){
    device_range_t range;
    Pozyx.getDeviceRangeInfo(anchors[i], &range);
    std::cout << "," << range.distance << "," << range.RSS;
  }
  std::cout << std::endl;
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult(){
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  std::cout << "list size: " << status*list_size << std::endl;
  
  if(list_size == 0){
    std::cout << "Calibration failed.\n" << Pozyx.getSystemError() << std::endl;
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  std::cout << "Calibration result:\n" << "Anchors found: " << list_size << std::endl;
  
  coordinates_t anchor_coor;
  for(int i=0; i<list_size; i++)
  {
    
    std::cout << "ANCHOR," << "0x" << std::hex << device_ids[i] << ",";
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    std::cout << anchor_coor.x << "," << anchor_coor.y << "," << anchor_coor.z << std::endl;
  }    
}

// function to manually set the anchor coordinates
void SetAnchorsManual(){
 
 int i=0;
 for(i=0; i<num_anchors; i++){
   device_coordinates_t anchor;
   anchor.network_id = anchors[i];
   anchor.flag = 0x1; 
   anchor.pos.x = anchors_x[i];
   anchor.pos.y = anchors_y[i];
   anchor.pos.z = heights[i];
   Pozyx.addDevice(anchor);
 }
 
}


int setup(){
  if(Pozyx.begin() == POZYX_FAILURE){
    std::cerr << "ERROR: Unable to connect to POZYX shield\n";
    std::cerr << "Reset required\n";
    return 1;
  }
  
  std::cout << "----------POZYX POSITIONING V1.0----------\n";
  std::cout << "NOTES:\n";
  std::cout << "- No parameters required.\n\n";
  std::cout << "- System will auto start calibration\n\n";
  std::cout << "- System will auto start positioning\n";
  std::cout << "----------POZYX POSITIONING V1.0----------\n\n";
  std::cout << "Performing auto anchor calibration:\n";

  // clear all previous devices in the device list
  Pozyx.clearDevices();
     
  int status = Pozyx.doAnchorCalibration(POZYX_2_5D, 10, num_anchors, anchors, heights);
  if (status != POZYX_SUCCESS){
    std::cerr << status << std::endl;
    std::cerr << "ERROR: calibration\n";
    std::cerr << "Reset required\n";
    return 1;
  }
  
  // if the automatic anchor calibration is unsuccessful, try manually setting the anchor coordinates.
  // fot this, you must update the arrays anchors_x, anchors_y and heights above
  // comment out the doAnchorCalibration block and the if-statement above if you are using manual mode
  //SetAnchorsManual();

  printCalibrationResult();
  usleep(1000);

  std::cout << "Starting positioning: \n";
  return 0;

}

void loop(){
  
  coordinates_t position;  
  int status = Pozyx.doPositioning(&position, POZYX_2_5D, 1000);
  
  if (status == POZYX_SUCCESS)
  {
    // print out the result
    if(!bProcessing){
      printCoordinates(position);
    }else{    
      printCoordinatesProcessing(position);
    }
  }
}

int main() {
  if(!setup()) {
    while(true) {
      loop();
    }
  }
  std::cout << "Aborted\n";
}
