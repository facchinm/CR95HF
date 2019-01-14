#include "CR95HF.h"

CR95HF reader(4, 2, 3, 5);

void setup() {

  Serial.begin(115200);
  while (!Serial) {}
  reader.begin();
  Serial.println(reader.readSerial());
}


void loop() {

  String id = reader.getID();
  if (id != "") {
    Serial.println(id);
  }

}
