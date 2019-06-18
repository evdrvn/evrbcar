#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "ssid.h"

#define __JOYSTICK__
#define CS (15)
#define SW_PIN (5)
#define LED_PIN (4)

#include "evrbcar.h" 

static WiFiUDP wifiUdp; 
static int cnt = 0;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

void setup() {
    int i = 0;
    t_evrbcar_cmd_request request;
    t_evrbcar_cmd_response *response;
    int packetSize; 

    pinMode(CS, OUTPUT );
    pinMode(LED_PIN, OUTPUT);
    pinMode(SW_PIN , INPUT);
    digitalWrite(LED_PIN, 1);

    ESP.eraseConfig();
    delay( 500 );

    Serial.begin( 115200 );
    Serial.println( );
    Serial.printf("Serial is %d bps", Serial.baudRate( ) );
    Serial.println( );

    SPI.begin( );
    SPI.setDataMode( SPI_MODE0 );
    SPI.setBitOrder( MSBFIRST );

    WiFi.mode(WIFI_STA);

    for(i = 0; i < SSIDNUM; i++){ 
        Serial.print("Connecting SSID: ");
        Serial.println(STASSID[i]);
        WiFi.begin(STASSID[i], STAPSK[i]);
        while (cnt < 10 && WiFi.status() != WL_CONNECTED) {
            Serial.print('.');
            delay(500);
        }
        if(WiFi.status() == WL_CONNECTED){
            Serial.print("Connected! IP address: ");
            Serial.println(WiFi.localIP());

            request.mode = EVRBCAR_CMD_CONNECT;
            wifiUdp.begin(CLIENT_UDP_PORT);

            while(1){
                wifiUdp.beginPacket("255.255.255.255", EVRBCAR_UDP_PORT);
                wifiUdp.write((uint8_t*)&request, sizeof(request));
                wifiUdp.endPacket();

                Serial.print("sending connect request: ");
                Serial.print(request.mode);
                Serial.print(", port = ");
                Serial.println(EVRBCAR_UDP_PORT);

                packetSize = wifiUdp.parsePacket(); 
                wifiUdp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
                response = (t_evrbcar_cmd_response*)packetBuffer;
                if(packetSize > 0){
                    Serial.print("connect reply: ");
                    Serial.println(response->mode);
                    if(response->mode == EVRBCAR_CMD_CONNECT) break;
                }
                delay(500);
            }  
            
            break;
        }
    }
}

int getMCP3204( int iCh )
{
    int highByte, lowByte;

    // Reading a data from MCP3204
    digitalWrite( CS, LOW );
    highByte = SPI.transfer( 0x06 | ( iCh >> 2 ) );
    highByte = SPI.transfer( iCh << 6 );
    lowByte = SPI.transfer( 0x00 );
    digitalWrite( CS, HIGH );

    return ( highByte & 0x0F ) * 256 + lowByte;
}

float convraw2a(int raw){
    float fvalue = 0.0F;
    float sign = 1.0F;
    fvalue = (raw - 2048.0F) / 2048.0F;
    if(fvalue < 0){
        sign = -1.0F;
        fvalue = fvalue * sign;
    }

    if(fvalue < 0.2) fvalue = 0.0F;
    else if(fvalue < 0.4) fvalue = 0.2F;
    else if(fvalue < 0.7) fvalue = 0.4F;
    else if(fvalue < 0.99) fvalue = 0.7F;
    else fvalue = 1.0F;

    return sign * fvalue;
}

void loop() {
    t_evrbcar_cmd_request request;
    t_evrbcar_cmd_request prev;
    
    int rawX = getMCP3204(0);
    int rawY = getMCP3204(1);

    request.mode = EVRBCAR_CMD_MOVE_TURN;
    request.fvalue[0] = convraw2a(rawX);
    request.fvalue[1] = convraw2a(rawY) * -1.0F;
    request.ivalue[0] = !digitalRead(SW_PIN);

    Serial.printf("Y=%4d(%f), X=%4d(%f), SW=%d \n",
            rawX,
            request.fvalue[0],
            rawY, 
            request.fvalue[1],
            request.ivalue[0]);

    wifiUdp.beginPacket(wifiUdp.remoteIP(), EVRBCAR_UDP_PORT);
    wifiUdp.write((uint8_t*)&request, sizeof(request));
    wifiUdp.endPacket();

    if(prev.ivalue[0] == 0 && request.ivalue[0] == 1) {
        request.mode = EVRBCAR_CMD_SCAN;
        wifiUdp.beginPacket(wifiUdp.remoteIP(), EVRBCAR_UDP_PORT);
        wifiUdp.write((uint8_t*)&request, sizeof(request));
        wifiUdp.endPacket();
    }
    prev = request;

    digitalWrite(LED_PIN, cnt++ % 2);
    delay(100);
}
