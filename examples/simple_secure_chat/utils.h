#ifndef SIMPLE_CHAT_UTILS_H
#define SIMPLE_CHAT_UTILS_H

#include <Arduino.h>

// Debug: print hex dump of string
void printHexDump(const char* label, const char* str) {
  Serial.print(label);
  Serial.print(": [");
  for (const unsigned char* p = (const unsigned char*)str; *p; p++) {
    if (p != (const unsigned char*)str) Serial.print(" ");
    Serial.printf("%02X", *p);
  }
  Serial.println("]");
}

// Remove Czech diacritics from UTF-8 string and convert to ASCII
// All other non-ASCII characters (emojis, etc.) are removed
// Modifies the string in-place
void removeDiacritics(char* str) {
  char* dst = str;
  unsigned char* src = (unsigned char*)str;
  
  while (*src) {
    if (*src < 128) {  // ASCII - just copy
      *dst++ = *src++;
    } else if (*src == 0xC3) {  // UTF-8 prefix for á,é,í,ó,ú,ý
      src++;
      if (*src) {
        switch (*src) {
          // lowercase
          case 0xA1: *dst++ = 'a'; break;  // á
          case 0xA9: *dst++ = 'e'; break;  // é
          case 0xAD: *dst++ = 'i'; break;  // í
          case 0xB3: *dst++ = 'o'; break;  // ó
          case 0xBA: *dst++ = 'u'; break;  // ú
          case 0xBD: *dst++ = 'y'; break;  // ý
          // uppercase
          case 0x81: *dst++ = 'A'; break;  // Á
          case 0x89: *dst++ = 'E'; break;  // É
          case 0x8D: *dst++ = 'I'; break;  // Í
          case 0x93: *dst++ = 'O'; break;  // Ó
          case 0x9A: *dst++ = 'U'; break;  // Ú
          case 0x9D: *dst++ = 'Y'; break;  // Ý
          // default: skip (don't write anything)
        }
        src++;
      }
    } else if (*src == 0xC4) {  // UTF-8 prefix for č,ď,ě
      src++;
      if (*src) {
        switch (*src) {
          // lowercase
          case 0x8D: *dst++ = 'c'; break;  // č = C4 8D
          case 0x8F: *dst++ = 'd'; break;  // ď = C4 8F
          case 0x9B: *dst++ = 'e'; break;  // ě = C4 9B
          // uppercase
          case 0x8C: *dst++ = 'C'; break;  // Č = C4 8C
          case 0x8E: *dst++ = 'D'; break;  // Ď = C4 8E
          case 0x9A: *dst++ = 'E'; break;  // Ě = C4 9A
          // default: skip (don't write anything)
        }
        src++;
      }
    } else if (*src == 0xC5) {  // UTF-8 prefix for ň,ř,š,ť,ů,ž
      src++;
      if (*src) {
        switch (*src) {
          // lowercase
          case 0x88: *dst++ = 'n'; break;  // ň = C5 88
          case 0x99: *dst++ = 'r'; break;  // ř = C5 99
          case 0xA1: *dst++ = 's'; break;  // š = C5 A1
          case 0xA5: *dst++ = 't'; break;  // ť = C5 A5
          case 0xAF: *dst++ = 'u'; break;  // ů = C5 AF
          case 0xBE: *dst++ = 'z'; break;  // ž = C5 BE
          // uppercase
          case 0x87: *dst++ = 'N'; break;  // Ň = C5 87
          case 0x98: *dst++ = 'R'; break;  // Ř = C5 98
          case 0xA0: *dst++ = 'S'; break;  // Š = C5 A0
          case 0xA4: *dst++ = 'T'; break;  // Ť = C5 A4
          case 0xAE: *dst++ = 'U'; break;  // Ů = C5 AE
          case 0xBD: *dst++ = 'Z'; break;  // Ž = C5 BD
          // default: skip (don't write anything)
        }
        src++;
      }
    } else if (*src >= 0xC0 && *src <= 0xDF) {
      // 2-byte UTF-8 sequence (not Czech) - skip both bytes
      src++;
      if (*src) src++;
    } else if (*src >= 0xE0 && *src <= 0xEF) {
      // 3-byte UTF-8 sequence (emojis, Chinese, etc.) - skip all 3 bytes
      src++;
      if (*src) src++;
      if (*src) src++;
    } else if (*src >= 0xF0 && *src <= 0xF7) {
      // 4-byte UTF-8 sequence (more emojis) - skip all 4 bytes
      src++;
      if (*src) src++;
      if (*src) src++;
      if (*src) src++;
    } else {
      // Invalid UTF-8 byte - skip it
      src++;
    }
  }
  *dst = 0;  // null terminate
}

#endif // SIMPLE_CHAT_UTILS_H
