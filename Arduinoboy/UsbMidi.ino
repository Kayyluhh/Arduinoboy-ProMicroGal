
#ifndef USE_TEENSY
void usbMidiSendTwoByteMessage(uint8_t b1, uint8_t b2) {};
void usbMidiSendThreeByteMessage(uint8_t b1, uint8_t b2, uint8_t b3) {};
void usbMidiSendRTMessage(uint8_t b) {};
void usbMidiHandleSysEx(const uint8_t *data, uint16_t length, bool complete) {};
void usbMidiInit() {};
void usbMidiUpdate() {};

#ifdef USE_LEONARDO
// Non-blocking USB-MIDI send, so DIN and USB can both run at the same time.
//
// Why: on AVR, MidiUSB.sendMIDI() calls USB_Send() (ArduinoCore-avr USBCore.cpp),
// which waits in 1 ms steps, up to 250 ms, for room in the IN endpoint whenever
// the host isn't draining it (enumerated, but no app reading the port, or the
// host is busy/asleep). MIDIUSB's own "drop instead of block" check is defined
// as (1) on AVR, so it never drops. Every one of those waits freezes the whole
// mode loop: DIN output and Game Boy clock handling included.
//
// What this does instead:
//  - not enumerated or suspended: skip USB entirely (DIN unaffected)
//  - endpoint has room: send immediately (a listening host drains it fast)
//  - endpoint full: wait at most USB_MIDI_MAX_WAIT_US once; if it's still full,
//    mark the host as stalled and drop packets with no waiting until the
//    endpoint frees up again (checked on every send, so recovery is automatic)

// The endpoint number is a protected member of PluggableUSBModule. A pointer to
// that member, taken inside a derived class, can legally be applied to the
// MidiUSB object without modifying the library.
struct MidiUsbEndpoint : MIDI_ {
  static uint8_t tx() {
    return MidiUSB.*(&MidiUsbEndpoint::pluggedEndpoint) + 1; // IN endpoint = plugged + 1
  }
};

const unsigned long USB_MIDI_MAX_WAIT_US = 1000;
bool usbMidiHostStalled = false;

void sendUsbMidiPacket(midiEventPacket_t packet)
{
  if (!USBDevice.configured() || USBDevice.isSuspended()) {
    usbMidiHostStalled = false;   // re-probe from scratch on next connect/wake
    return;
  }

  const uint8_t ep = MidiUsbEndpoint::tx();
  if (USB_SendSpace(ep) < sizeof(packet)) {
    if (usbMidiHostStalled) return;          // host still not reading: drop, don't wait
    const unsigned long start = micros();
    while (USB_SendSpace(ep) < sizeof(packet)) {
      if (micros() - start > USB_MIDI_MAX_WAIT_US) {
        usbMidiHostStalled = true;           // give up on this packet
        return;
      }
    }
  }

  usbMidiHostStalled = false;
  MidiUSB.sendMIDI(packet);                  // room confirmed, so USB_Send won't wait
  MidiUSB.flush();
}
#endif

#else

void usbMidiSendTwoByteMessage(uint8_t b1, uint8_t b2)
{
    uint8_t stat = b1 & 0xf0;
    uint8_t chan = (b1 & 0x0f)+1;
    if(stat == 0xC0) {
        usbMIDI.sendProgramChange(b2, chan);
    } else if (stat == 0xD0) {
        usbMIDI.sendAfterTouch(b2, chan);
    }
}

void usbMidiSendThreeByteMessage(uint8_t b1, uint8_t b2, uint8_t b3)
{
    uint8_t channel = (b1&0x0F)+1;

    switch(midiData[0] & 0xF0) {
        case 0x80:
          usbMIDI.sendNoteOff(b2, b3, channel);
          usbMIDI.send_now();
          break;
        case 0x90:
          usbMIDI.sendNoteOn(b2, b3, channel);
          usbMIDI.send_now();
          break;
        case 0xA0:
          usbMIDI.sendPolyPressure(b2, b3, channel);
          break;
        case 0xB0:
          usbMIDI.sendControlChange(b2, b3, channel);
          usbMIDI.send_now();
          break;
        case 0xE0:
          unsigned short v = (unsigned short)b3;
          v<<=7;
          v|=(unsigned short)b2;
          usbMIDI.sendPitchBend(v, channel);
          break;
    }
}

void usbMidiSendRTMessage(uint8_t b)
{
    usbMIDI.sendRealTime(b);
}

void usbMidiUpdate()
{
    usbMIDI.read();
}

void usbMidiHandleSysEx(const uint8_t *data, uint16_t length, bool complete)
{
    if(sysexPosition + length >= longestSysexMessage || (length < 3 && complete)) {
        //wrapped!
        sysexPosition = 0;
        return ;
    }

    if(sysexPosition == 0 && complete) {
        memcpy(&sysexData[0], &data[1], length-2);
        sysexPosition += length-2;
    } else if (sysexPosition == 0 && !complete) {
        memcpy(&sysexData[0], &data[1], length-1);
        sysexPosition += length-1;
    } else if (!complete) {
        memcpy(&sysexData[sysexPosition], &data[0], length);
        sysexPosition += length;
    } else {
        memcpy(&sysexData[sysexPosition], &data[0], length-1);
        sysexPosition += length-1;
    }

    if(complete) {
        getSysexData();
    }
}

void usbMidiInit()
{
    usbMIDI.setHandleSysEx(usbMidiHandleSysEx);
}

#endif






















