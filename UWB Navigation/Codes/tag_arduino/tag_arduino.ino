

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define REPORT_LOCATION 9
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer


// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 50;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

volatile uint8_t idx=0;

#define REPORT_ANCHOR 4
#define numAnchors  4
const uint8_t tagId = 0;
volatile uint8_t anchorIdx = 1; // Starting Anchor Id. Polls from this number to this number + numAnchors
float curRange;

#define LEN_DATA 20
byte data[LEN_DATA];

DW1000Time timeComputedRange;

float rangeArray[numAnchors];

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  //    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(tagId);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();
  //Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  //Serial.print("Device ID: "); //Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  //Serial.print("Unique ID: "); //Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  //Serial.print("Network ID & Device Address: "); //Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  //Serial.print("Device mode: "); //Serial.println(msg);
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message

  Serial.println("Starting Tag Localization");

  data[1] = anchorIdx;
  data[2] = tagId;

  receiver();
  transmitPoll();
  noteActivity();
}

void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {
  // tag sends POLL and listens for POLL_ACK
  anchorIdx = 1;
  expectedMsgId = POLL_ACK;
  transmitPoll();
  noteActivity();
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
}



void transmitPoll() {
  if(anchorIdx == numAnchors){
    //Serial.println("Sending Ranges");
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    data[1] = REPORT_ANCHOR;
    memcpy(&data[3], rangeArray, 4*numAnchors);
    DW1000.setData(data, 3 + 4*numAnchors);
    DW1000.startTransmit();  
  }
  else{
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    data[1] = anchorIdx;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();  
  }
  
}

void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  data[1] = anchorIdx;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data + 3);
  timePollAckReceived.getTimestamp(data + 8);
  timeRangeSent.getTimestamp(data + 13);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  ////Serial.print("Expect RANGE to be sent @ "); //Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void loop() {
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
      ////Serial.print("Sent POLL @ "); //Serial.println(timePollSent.getAsFloat());
    } else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];

    if((data[2] == tagId) && (data[1] == anchorIdx)){

      if (msgId != expectedMsgId) {
        // unexpected message, start over again
        ////Serial.print("Received wrong message # "); //Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT) {
        expectedMsgId = POLL_ACK;
        memcpy(&curRange, data + 3, 4);
        //                        Serial.print(anchorIdx);Serial.print("   ");Serial.println(curRange);
        rangeArray[anchorIdx-1] = curRange;
        
        for(idx=0;idx<numAnchors;idx++){
          Serial.print(rangeArray[idx]); 
          if(idx!=numAnchors-1) 
            Serial.print(",");
        }
        Serial.println();

        anchorIdx = (anchorIdx==numAnchors)?1:anchorIdx+1;
        
        transmitPoll();
        noteActivity();

      } else if (msgId == RANGE_FAILED) {
        expectedMsgId = POLL_ACK;
        anchorIdx = (anchorIdx==numAnchors)?1:anchorIdx+1;
        transmitPoll();
        noteActivity();
      }          
    }
  }
}
